// SPDX-License-Identifier: Apache-2.0
/* Flight display application: Engine module ABI glue, touch/menu logic,
 * simulated air data and periodic diagnostics. Sensor fusion lives in
 * app_sensors (own task); drawing lives in app_pfd. Engine Host accepts
 * source lists only, hence the relative includes. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "include/app_game_runtime.h"
#include "../app_pfd/include/pfd_view.h"
#include "../app_sensors/include/pfd_sensors.h"
#include "../app_link/include/pfd_link.h"
#include "private/game_config.h"
#include "raylib_lite_raylib.h"

#if defined(ESP_PLATFORM)
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "raylib_lite_example_board.h"
#define LOGI(...) ESP_LOGI("pfd", __VA_ARGS__)
#else
#define LOGI(...) do { printf(__VA_ARGS__); printf("\n"); } while (0)
#endif

#define LONG_PRESS_S 0.7
#define TAP_SLOP_PX 24
#define TOAST_S 2.2
#define UI_MAGIC 0x50464455u

static const int s_bright_levels[] = {100, 75, 50, 30};
enum { BRIGHT_LEVELS = sizeof(s_bright_levels) / sizeof(s_bright_levels[0]) };

typedef struct {
    uint32_t magic;
    uint8_t bright_idx;
    uint8_t sim;
} ui_store_t;

typedef struct {
    bool down, long_fired;
    int x, y, x0, y0;
    double press_t;
    bool menu_open;
    int menu_pressed;
    bool phone_open;
    int phone_page;
    int last_ap_clients;
    int last_link_state;
    int bright_idx;
    bool sim;
    const char *toast;
    double toast_until;
    uint32_t magcal_seq;
    uint32_t levelcal_seq;
    bool levelcal_seq_init;
    bool magcal_seq_init;
    /* diagnostics */
    unsigned frames;
    double fps_t0;
    float fps;
    double next_log;
    double started;
    double draw_ms_acc;
    unsigned draw_n;
    char info[96];
    char ap_ssid[40], ap_pass[16], ap_url[32], sta_url[32];
} host_state_t;

#if defined(ESP_PLATFORM)
/* The Board's own IMU worker would compete for the BMI270; app_sensors owns it. */
void raylib_lite_example_game_board_config(raylib_lite_example_board_config_t *config)
{
    if (!config) return;
    *config = (raylib_lite_example_board_config_t){
        .touch_points = 1, .enable_imu = false, .imu_sample_ms = 0,
        .drawbuf_lines = 34, .drawbuf_count = 2,
    };
}

static void ui_load(host_state_t *s)
{
    nvs_handle_t h;
    ui_store_t st;
    size_t len = sizeof(st);
    if (nvs_open("pfd", NVS_READONLY, &h) != ESP_OK) return;
    if (nvs_get_blob(h, "ui", &st, &len) == ESP_OK && len == sizeof(st) && st.magic == UI_MAGIC) {
        s->bright_idx = st.bright_idx < BRIGHT_LEVELS ? st.bright_idx : 0;
        s->sim = st.sim != 0;
    }
    nvs_close(h);
}

static void ui_save(const host_state_t *s)
{
    nvs_handle_t h;
    ui_store_t st = {UI_MAGIC, (uint8_t)s->bright_idx, (uint8_t)s->sim};
    if (nvs_open("pfd", NVS_READWRITE, &h) != ESP_OK) return;
    if (nvs_set_blob(h, "ui", &st, sizeof(st)) == ESP_OK) (void)nvs_commit(h);
    nvs_close(h);
}
#else
static void ui_load(host_state_t *s) { (void)s; }
static void ui_save(const host_state_t *s) { (void)s; }
#endif

/* Engine GetTime() counts presented frames, so it cannot measure FPS.
 * Device: real monotonic clock. Host: logic ticks (deterministic replays). */
static unsigned long s_host_ticks;
static double now_s(void)
{
#if defined(ESP_PLATFORM)
    return esp_timer_get_time() / 1e6;
#else
    return (double)s_host_ticks / GAME_TICK_HZ;
#endif
}

#if !defined(ESP_PLATFORM)
double pfd_host_now_s(void) { return now_s(); }
#endif

static void toast(host_state_t *s, const char *msg)
{
    s->toast = msg;
    s->toast_until = now_s() + TOAST_S;
}

static int initialize(void *value, const char *asset_root)
{
    (void)asset_root;
    host_state_t *s = value;
    memset(s, 0, sizeof(*s));
    s->menu_pressed = -1;
    InitWindow(GAME_WIDTH, GAME_HEIGHT, GAME_NAME);
    SetTargetFPS(GAME_TICK_HZ);
    ui_load(s);
    if (pfd_sensors_start() != 0) LOGI("sensor service failed to start");
    if (pfd_link_start() != 0) LOGI("phone link failed to start");
    s->started = now_s();
    s->fps_t0 = s->started;
    s->next_log = s->started + 1.0;
    LOGI("flight_pfd v2.2 started (bright %d%%, sim %d)", s_bright_levels[s->bright_idx], s->sim);
    return 0;
}

static void shutdown(void *value) { (void)value; }

static void menu_action(host_state_t *s, int item)
{
    switch (item) {
    case PFD_MENU_LEVEL:
        pfd_sensors_command(PFD_CMD_LEVEL);
        s->menu_open = false;
        break;
    case PFD_MENU_MOUNT: {
        /* display mode: upright (default) <-> flat */
        pfd_sens_snapshot_t snap;
        pfd_sensors_read(&snap);
        bool to_upright = snap.mount_mode == 0;
        pfd_sensors_command(to_upright ? PFD_CMD_MOUNT_UPRIGHT : PFD_CMD_MOUNT_FLAT);
        toast(s, to_upright ? "显示方向：竖直（默认）" : "显示方向：平放（屏幕朝上）");
        break;
    }
    case PFD_MENU_MAGCAL:
        pfd_sensors_command(PFD_CMD_MAGCAL_START);
        s->menu_open = false;
        break;
    case PFD_MENU_PHONE: {
        pfd_link_snapshot_t l;
        pfd_link_read(&l);
        s->menu_open = false;
        s->phone_open = true;
        s->phone_page = l.ap_clients > 0 ? 1 : 0;
        s->last_ap_clients = l.ap_clients;
        break;
    }
    case PFD_MENU_PEAK:
        pfd_sensors_command(PFD_CMD_PEAK_RESET);
        toast(s, "峰值已清零");
        s->menu_open = false;
        break;
    case PFD_MENU_BRIGHT:
        s->bright_idx = (s->bright_idx + 1) % BRIGHT_LEVELS;
        ui_save(s);
        break;
    case PFD_MENU_SIM:
        s->sim = !s->sim;
        ui_save(s);
        break;
    case PFD_MENU_CLOSE:
        s->menu_open = false;
        break;
    default:
        break;
    }
}

static void on_tap(host_state_t *s, int x, int y)
{
    pfd_sens_snapshot_t snap;
    pfd_sensors_read(&snap);
    if (snap.state == PFD_SENS_MAGCAL) {
        if (pfd_view_hit_magcal_cancel(x, y)) pfd_sensors_command(PFD_CMD_MAGCAL_CANCEL);
        return;
    }
    if (snap.state == PFD_SENS_LEVELCAL) {
        if (pfd_view_hit_magcal_cancel(x, y)) pfd_sensors_command(PFD_CMD_LEVEL_CANCEL);
        return;
    }
    if (snap.state == PFD_SENS_GYRO_CAL || snap.state == PFD_SENS_STARTING) return;
    if (s->phone_open) {
        int b = pfd_view_hit_phone(x, y);
        if (b == 1) {
            if (s->phone_page == 0) s->phone_open = false;
            else s->phone_page = 0;
        } else if (b == 2) {
            if (s->phone_page == 0) s->phone_page = 1;
            else s->phone_open = false;
        }
        return;
    }
    if (s->menu_open) {
        int item = pfd_view_hit_menu_item(x, y);
        if (item >= 0) menu_action(s, item);
        else if (x < 50 || x > 430 || y < 22 || y > 458) s->menu_open = false;
        return;
    }
    if (pfd_view_hit_menu_button(x, y)) s->menu_open = true;
}

static void input(void *value, const raylib_lite_game_input_v1_t *event)
{
    host_state_t *s = value;
    if (!event || event->type != RAYLIB_LITE_GAME_INPUT_POINTER) return;
    if (event->pressed) {
        if (!s->down) {
            s->down = true;
            s->long_fired = false;
            s->x0 = (int)event->x;
            s->y0 = (int)event->y;
            s->press_t = now_s();
        }
        s->x = (int)event->x;
        s->y = (int)event->y;
        s->menu_pressed = s->menu_open ? pfd_view_hit_menu_item(s->x, s->y) : -1;
    } else if (s->down) {
        s->down = false;
        s->menu_pressed = -1;
        int dx = s->x - s->x0, dy = s->y - s->y0;
        if (!s->long_fired && dx * dx + dy * dy <= TAP_SLOP_PX * TAP_SLOP_PX) on_tap(s, s->x0, s->y0);
    }
}

static void magcal_feedback(host_state_t *s, const pfd_sens_snapshot_t *snap)
{
    if (!s->magcal_seq_init) {
        s->magcal_seq = snap->magcal_result_seq;
        s->magcal_seq_init = true;
        return;
    }
    if (snap->magcal_result_seq == s->magcal_seq) return;
    s->magcal_seq = snap->magcal_result_seq;
    switch (snap->magcal_result) {
    case PFD_MAGCAL_OK:
        toast(s, snap->mag_source == 2 ? "校准完成：两个磁力计取平均" :
                 snap->mag_source == 1 ? "校准完成：使用磁力计 #1" : "校准完成：使用磁力计 #0");
        break;
    case PFD_MAGCAL_POOR_COVERAGE: toast(s, "方向不够全，请多转几个方向再试"); break;
    case PFD_MAGCAL_FIT_FAILED: toast(s, "校准失败：请远离磁铁和金属再试"); break;
    case PFD_MAGCAL_CANCELLED: toast(s, "已取消校准"); break;
    default: break;
    }
}

static void levelcal_feedback(host_state_t *s, const pfd_sens_snapshot_t *snap)
{
    if (!s->levelcal_seq_init) {
        s->levelcal_seq = snap->levelcal_result_seq;
        s->levelcal_seq_init = true;
        return;
    }
    if (snap->levelcal_result_seq == s->levelcal_seq) return;
    s->levelcal_seq = snap->levelcal_result_seq;
    switch (snap->levelcal_result) {
    case PFD_LEVELCAL_OK: toast(s, "校准完成，请竖直拿起"); break;
    case PFD_LEVELCAL_NOT_FLAT: toast(s, "未平放，请重试"); break;
    case PFD_LEVELCAL_MOVING: toast(s, "一直在晃动，请放稳后重试"); break;
    case PFD_LEVELCAL_CANCELLED: toast(s, "已取消校准"); break;
    default: break;
    }
    LOGI("level cal result %d tilt=%.2f align=%.2f", (int)snap->levelcal_result, snap->levelcal_tilt_deg,
         snap->align_deg);
}

static void update(void *value)
{
    host_state_t *s = value;
    s_host_ticks++;
    double now = now_s();
    pfd_sens_snapshot_t snap;
    pfd_sensors_read(&snap);
    magcal_feedback(s, &snap);
    levelcal_feedback(s, &snap);
    if (s->down && !s->long_fired && now - s->press_t >= LONG_PRESS_S) {
        int dx = s->x - s->x0, dy = s->y - s->y0;
        if (dx * dx + dy * dy <= TAP_SLOP_PX * TAP_SLOP_PX && snap.state == PFD_SENS_RUN && !s->phone_open) {
            s->long_fired = true;
            s->menu_open = true;
        }
    }
    if (s->toast && now >= s->toast_until) s->toast = NULL;

    pfd_link_snapshot_t link;
    pfd_link_read(&link);
    if (s->phone_open && s->phone_page == 0 && link.ap_clients > s->last_ap_clients) s->phone_page = 1;
    s->last_ap_clients = link.ap_clients;
    if ((int)link.state != s->last_link_state) {
        static const char *names[] = {"none", "phone connected", "signal lost"};
        LOGI("gps link: %s (packets=%lu)", names[link.state], (unsigned long)link.packets);
        if (link.state == PFD_LINK_ACTIVE) toast(s, "手机定位已连接");
        else if (link.state == PFD_LINK_LOST) toast(s, "手机信号丢失");
        s->last_link_state = (int)link.state;
    }

    if (now - s->fps_t0 >= 1.0) {
        s->fps = (float)(s->frames / (now - s->fps_t0));
        s->frames = 0;
        s->fps_t0 = now;
    }
    if (now >= s->next_log) {
        LOGI("att roll=%+.1f pitch=%+.1f hdg=%.1f g=%.2f peak=%.2f imu=%.0fHz fps=%.1f state=%d mag_src=%d "
             "mount=%d align=%.2f mag_ok=%d/%d sat=%lu/%lu noise=%.2f/%.2fuT resid=%.3f/%.3f draw=%.1fms gps=%d/%lu",
             (double)snap.roll_deg, (double)snap.pitch_deg, (double)snap.heading_deg, (double)snap.g_now,
             (double)snap.g_peak, (double)snap.imu_rate_hz, (double)s->fps, (int)snap.state, snap.mag_source,
             (int)snap.mount_mode, (double)snap.align_deg, snap.mag_ok[0], snap.mag_ok[1], (unsigned long)snap.mag_overflow[0],
             (unsigned long)snap.mag_overflow[1], (double)snap.mag_noise_ut[0], (double)snap.mag_noise_ut[1],
             (double)snap.mag_residual[0], (double)snap.mag_residual[1],
             s->draw_n ? s->draw_ms_acc / s->draw_n : 0.0, (int)link.state, (unsigned long)link.packets);
        s->draw_ms_acc = 0;
        s->draw_n = 0;
        s->next_log = now + (now - s->started < 120.0 ? 1.0 : 10.0);
    }
}

static void fill_view(host_state_t *s, const pfd_sens_snapshot_t *snap, pfd_view_t *v)
{
    double t = now_s();
    memset(v, 0, sizeof(*v));
    v->roll_deg = snap->roll_deg;
    v->pitch_deg = snap->pitch_deg;
    v->heading_deg = snap->heading_deg;
    v->heading_calibrated = snap->mag_calibrated;
    v->g_now = snap->g_now;
    v->g_peak = snap->g_peak;
    pfd_link_snapshot_t L;
    pfd_link_read(&L);
    const bool active = L.state == PFD_LINK_ACTIVE;
    const bool fresh = L.has_fix && L.fix_age_s < 10.0f;
    v->sim = s->sim && !active;
    if (active) {
        v->spd_live = fresh && L.speed_valid;
        v->speed_kmh = L.speed_mps * 3.6f;
        v->spd_status = !fresh ? (L.has_fix ? "定位过期" : "等待定位") : !L.speed_valid ? "无速度" :
                        L.speed_derived ? "GPS推算" : "GPS";
        v->spd_tone = v->spd_live ? PFD_TONE_GREEN : PFD_TONE_AMBER;
        v->alt_live = fresh && L.alt_valid;
        v->alt_m = L.alt_m;
        v->alt_status = !fresh ? (L.has_fix ? "定位过期" : "等待定位") : !L.alt_valid ? "无高度" : "GPS";
        v->alt_tone = v->alt_live ? PFD_TONE_GREEN : PFD_TONE_AMBER;
    } else if (s->sim) {
        v->spd_live = v->alt_live = true;
        v->speed_kmh = (float)(180.0 + 40.0 * sin(t * 0.21) + 12.0 * sin(t * 0.57));
        v->alt_m = (float)(1200.0 + 260.0 * sin(t * 0.09) + 30.0 * sin(t * 0.41));
        v->spd_status = v->alt_status = "模拟";
        v->spd_tone = v->alt_tone = PFD_TONE_AMBER;
    } else if (L.state == PFD_LINK_LOST) {
        v->spd_status = v->alt_status = "信号丢失";
        v->spd_tone = v->alt_tone = PFD_TONE_RED;
    } else {
        v->spd_status = v->alt_status = "无GPS";
    }
    v->gps_link = (int)L.state;
    v->gps_fix = L.has_fix;
    v->lat = L.lat;
    v->lon = L.lon;
    v->gps_acc_m = L.acc_m;
    v->track_valid = active && fresh && L.track_valid;
    v->track_deg = L.track_deg;
    v->phone_page = s->phone_page;
    v->wifi_up = L.wifi_up;
    v->server_up = L.server_up;
    memcpy(s->ap_ssid, L.ap_ssid, sizeof(s->ap_ssid));
    memcpy(s->ap_pass, L.ap_pass, sizeof(s->ap_pass));
    memcpy(s->ap_url, L.ap_url, sizeof(s->ap_url));
    memcpy(s->sta_url, L.sta_url, sizeof(s->sta_url));
    v->ap_ssid = s->ap_ssid;
    v->ap_pass = s->ap_pass;
    v->ap_url = s->ap_url;
    v->sta_url = s->sta_url;
    v->ap_clients = L.ap_clients;
    v->gps_packets = L.packets;
    v->bright_pct = s_bright_levels[s->bright_idx];
    v->menu_pressed = s->menu_pressed;
    v->toast = s->toast;
    v->progress = snap->progress;
    v->magcal_coverage = snap->magcal_coverage;
    v->magcal_seconds = snap->magcal_seconds;
    v->mount_mode = snap->mount_mode;
    v->level_set = snap->level_set;
    v->levelcal_remaining_s = snap->levelcal_remaining_s;
    v->levelcal_moving = snap->levelcal_moving;
    switch (snap->state) {
    case PFD_SENS_STARTING:
    case PFD_SENS_GYRO_CAL: v->overlay = PFD_OVERLAY_BOOT; break;
    case PFD_SENS_MAGCAL: v->overlay = PFD_OVERLAY_MAGCAL; break;
    case PFD_SENS_LEVELCAL: v->overlay = PFD_OVERLAY_LEVELCAL; break;
    case PFD_SENS_ERROR: v->overlay = PFD_OVERLAY_ERROR; v->error = "传感器初始化失败，请重启"; break;
    default:
        v->overlay = s->phone_open ? PFD_OVERLAY_PHONE : s->menu_open ? PFD_OVERLAY_MENU : PFD_OVERLAY_NONE;
        break;
    }
    if (v->overlay == PFD_OVERLAY_MENU) {
        char mag[32];
        if (snap->mag_source < 0) snprintf(mag, sizeof(mag), "磁力计未校准");
        else if (snap->mag_source == 2) snprintf(mag, sizeof(mag), "磁力计 双");
        else snprintf(mag, sizeof(mag), "磁力计 #%d", snap->mag_source);
        snprintf(s->info, sizeof(s->info), "IMU %.0fHz · 帧率 %.0f · %s", (double)snap->imu_rate_hz,
                 (double)s->fps, mag);
        v->info = s->info;
    }
}

static int render(void *value)
{
    host_state_t *s = value;
    BeginDrawing();
    if (!raylib_lite_raylib_frame_available()) {
        EndDrawing();
        return -1;
    }
    int w = 0, h = 0;
    size_t stride = 0;
    uint16_t *fb = raylib_lite_raylib_get_framebuffer(&w, &h, &stride);
    if (fb && w == PFD_W && h == PFD_H) {
        /* Sample attitude as late as possible: lowest motion-to-photon delay. */
        pfd_sens_snapshot_t snap;
        pfd_sensors_read(&snap);
        pfd_view_t v;
        fill_view(s, &snap, &v);
        pfd_gfx_t g = {.fb = fb, .w = w, .h = h, .stride = stride, .bright = v.bright_pct * 256 / 100};
        double t0 = now_s();
        pfd_view_draw(&g, &v);
        s->draw_ms_acc += (now_s() - t0) * 1000.0;
        s->draw_n++;
    }
    EndDrawing();
    s->frames++;
    return 0;
}

static uint32_t state_hash(const void *value)
{
    const host_state_t *s = value;
    return (uint32_t)s->menu_open * 31U + (uint32_t)s->bright_idx * 7U + (uint32_t)s->sim +
           (uint32_t)s->phone_open * 101U + (uint32_t)s->phone_page * 211U;
}

static int state_json(const void *value, char *output, size_t capacity)
{
    if (!output || !capacity) return -1;
    const host_state_t *s = value;
    pfd_sens_snapshot_t snap;
    pfd_sensors_read(&snap);
    pfd_link_snapshot_t L;
    pfd_link_read(&L);
    int size = snprintf(output, capacity,
                        "{\"phase\":\"%s\",\"roll\":%.2f,\"pitch\":%.2f,\"heading\":%.2f,\"menu\":%s,"
                        "\"sim\":%s,\"bright\":%d,\"fps\":%.1f,\"phone\":%s,\"page\":%d,\"gps\":%d,"
                        "\"packets\":%lu,\"spd_valid\":%s,\"spd_kmh\":%.1f,\"derived\":%s,\"alt\":%.1f,"
                        "\"state_hash\":\"%08lx\"}",
                        snap.state == PFD_SENS_RUN ? "running" : "calibrating", (double)snap.roll_deg,
                        (double)snap.pitch_deg, (double)snap.heading_deg, s->menu_open ? "true" : "false",
                        s->sim ? "true" : "false", s_bright_levels[s->bright_idx], (double)s->fps,
                        s->phone_open ? "true" : "false", s->phone_page, (int)L.state, (unsigned long)L.packets,
                        L.speed_valid ? "true" : "false", (double)(L.speed_mps * 3.6f),
                        L.speed_derived ? "true" : "false", (double)L.alt_m, (unsigned long)state_hash(value));
    return size < 0 || (size_t)size >= capacity ? -1 : size;
}

static const raylib_lite_game_module_v1_t s_module = {
    .descriptor = {
        RAYLIB_LITE_GAME_MODULE_ABI_V1, GAME_NAME, GAME_NAME,
        GAME_WIDTH, GAME_HEIGHT, GAME_TICK_HZ, 1
    },
    .state_size = sizeof(host_state_t),
    .initialize = initialize, .shutdown = shutdown, .input = input,
    .update = update, .render = render,
    .state_hash = state_hash, .state_json = state_json,
};

const raylib_lite_game_module_v1_t *app_game_runtime_module(void)
{
    return &s_module;
}
