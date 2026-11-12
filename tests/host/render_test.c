// SPDX-License-Identifier: MIT
/* Render reference frames to PPM for visual review and time the renderer. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "pfd_view.h"

static uint16_t fb[PFD_W * PFD_H];

static void save(const char *path)
{
    FILE *f = fopen(path, "wb");
    fprintf(f, "P6\n%d %d\n255\n", PFD_W, PFD_H);
    for (int i = 0; i < PFD_W * PFD_H; ++i) {
        uint16_t c = fb[i];
        unsigned char rgb[3] = {(unsigned char)(((c >> 11) & 31) << 3), (unsigned char)(((c >> 5) & 63) << 2),
                                (unsigned char)((c & 31) << 3)};
        fwrite(rgb, 1, 3, f);
    }
    fclose(f);
}

int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : ".";
    pfd_gfx_t g = {.fb = fb, .w = PFD_W, .h = PFD_H, .stride = PFD_W, .bright = 256};
    struct { const char *name; pfd_view_t v; } cases[] = {
        {"level", {.roll_deg = 0, .pitch_deg = 0, .heading_deg = 0, .heading_calibrated = true, .g_now = 1.0f, .g_peak = 1.3f, .bright_pct = 100, .menu_pressed = -1}},
        {"roll_right_20_pitch_up_8", {.roll_deg = 20, .pitch_deg = 8, .heading_deg = 275, .heading_calibrated = true, .g_now = 1.12f, .g_peak = 2.04f, .sim = true, .speed_kmh = 182, .alt_m = 1260,
            .spd_live = true, .alt_live = true, .spd_status = "模拟", .alt_status = "模拟", .spd_tone = PFD_TONE_AMBER, .alt_tone = PFD_TONE_AMBER, .bright_pct = 100, .menu_pressed = -1}},
        {"roll_left_50_pitch_down_12", {.roll_deg = -50, .pitch_deg = -12, .heading_deg = 93, .heading_calibrated = false, .g_now = 1.9f, .g_peak = 2.7f, .bright_pct = 100, .menu_pressed = -1}},
        {"menu", {.roll_deg = 10, .pitch_deg = 3, .heading_deg = 45, .heading_calibrated = true, .g_now = 1.0f, .g_peak = 1.0f, .bright_pct = 75, .overlay = PFD_OVERLAY_MENU, .menu_pressed = 1, .mount_mode = 0, .info = "IMU 200Hz · 帧率 38 · 磁力计 #0 残差 2.1%"}},
        {"magcal", {.heading_deg = 10, .g_now = 1.0f, .bright_pct = 100, .overlay = PFD_OVERLAY_MAGCAL, .progress = 0.45f, .magcal_coverage = 12, .magcal_seconds = 9, .menu_pressed = -1}},
        {"levelcal", {.heading_deg = 10, .g_now = 1.0f, .bright_pct = 100, .overlay = PFD_OVERLAY_LEVELCAL, .progress = 0.6f, .levelcal_remaining_s = 0.8f, .levelcal_moving = true, .menu_pressed = -1}},
        {"boot", {.g_now = 1.0f, .bright_pct = 100, .overlay = PFD_OVERLAY_BOOT, .progress = 0.6f, .menu_pressed = -1}},
        {"gps_live", {.roll_deg = -8, .pitch_deg = 4, .heading_deg = 32, .heading_calibrated = true, .g_now = 1.0f, .g_peak = 1.4f, .bright_pct = 100, .menu_pressed = -1,
            .spd_live = true, .speed_kmh = 63.4f, .spd_status = "GPS", .spd_tone = PFD_TONE_GREEN, .alt_live = true, .alt_m = 41, .alt_status = "GPS", .alt_tone = PFD_TONE_GREEN,
            .gps_link = 1, .gps_fix = true, .lat = 31.230461, .lon = 121.473702, .gps_acc_m = 5, .track_valid = true, .track_deg = 50}},
        {"gps_lost", {.heading_deg = 32, .heading_calibrated = true, .g_now = 1.0f, .bright_pct = 100, .menu_pressed = -1,
            .spd_status = "信号丢失", .spd_tone = PFD_TONE_RED, .alt_status = "信号丢失", .alt_tone = PFD_TONE_RED,
            .gps_link = 2, .gps_fix = true, .lat = 31.230461, .lon = 121.473702, .gps_acc_m = 5}},
        {"phone_hotspot", {.heading_deg = 32, .g_now = 1.0f, .bright_pct = 100, .menu_pressed = -1, .overlay = PFD_OVERLAY_PHONE, .phone_page = 0,
            .wifi_up = true, .server_up = true, .ap_ssid = "飞行表-1A2B", .ap_pass = "12345678", .ap_url = "https://192.168.4.1", .sta_url = ""}},
        {"phone_page", {.heading_deg = 32, .g_now = 1.0f, .bright_pct = 30, .menu_pressed = -1, .overlay = PFD_OVERLAY_PHONE, .phone_page = 1,
            .wifi_up = true, .server_up = true, .ap_ssid = "飞行表-1A2B", .ap_pass = "12345678", .ap_url = "https://192.168.4.1", .sta_url = "",
            .gps_link = 1, .gps_fix = true, .gps_acc_m = 6, .gps_packets = 123, .ap_clients = 1}},
        {"inverted_toast", {.roll_deg = 170, .pitch_deg = 30, .heading_deg = 200, .heading_calibrated = true, .g_now = 1.0f, .bright_pct = 100, .toast = "已设为水平", .menu_pressed = -1}},
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        g.bright = cases[i].v.bright_pct * 256 / 100;
        pfd_view_draw(&g, &cases[i].v);
        char path[256];
        snprintf(path, sizeof(path), "%s/%s.ppm", dir, cases[i].name);
        save(path);
    }
    pfd_view_t v = cases[1].v;
    clock_t t0 = clock();
    for (int i = 0; i < 300; ++i) {
        v.roll_deg = (float)(i % 120) - 60;
        v.pitch_deg = (float)(i % 40) - 20;
        v.heading_deg = (float)i;
        pfd_view_draw(&g, &v);
    }
    printf("host render: %.2f ms/frame\n", (double)(clock() - t0) * 1000.0 / CLOCKS_PER_SEC / 300);
    return 0;
}
