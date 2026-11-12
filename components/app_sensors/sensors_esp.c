// SPDX-License-Identifier: MIT
/* Device sensor service: BMI270 at 200 Hz + two BMM150 at 25 Hz each on the
 * Board Manager I2C bus (the Board's own IMU device stays disabled so this
 * task is the only owner of those sensors). One pinned task does sampling,
 * fusion, calibration and NVS persistence; the UI reads snapshots. */
#include "pfd_sensors.h"

#include <math.h>
#include <string.h>
#include "bmi270.h"
#include "bmm150.h"
#include "driver/i2c_master.h"
#include "esp_board_periph.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "pfd_fusion.h"

static const char *TAG = "pfd_sens";

#define I2C_BUS_NAME "i2c_master"
#define I2C_TIMEOUT_MS 20
#define I2C_SPEED_HZ 400000
#define BMI_ADDR_A 0x69
#define BMI_ADDR_B 0x68
#define BMI_WRITE_MAX 32U
static const uint8_t BMM_ADDR[2] = {0x11, 0x12};

#define RATE_HZ 200
#define PERIOD_MS (1000 / RATE_HZ)
#define MAG_EVERY 4            /* alternate sensors -> each at 25 Hz */
#define ACC_LSB_8G (8.0f / 32768.0f)
#define GYR_LSB_1000 (1000.0f / 32768.0f)
#define D2R 0.017453292f
#define R2D 57.29578f

#define GYRO_CAL_SAMPLES 200   /* 1 s still window */
#define GYRO_CAL_MAX_SECONDS 6
#define GYRO_STILL_DPS 0.6f    /* max per-axis std while still */
#define ACC_STILL_G 0.02f

#define MAGCAL_MAX 640         /* per sensor (25 Hz * 25 s) */
#define MAGCAL_MIN_SECONDS 12.0f
#define MAGCAL_TARGET_SECONDS 20.0f
#define MAGCAL_MAX_SECONDS 30.0f
#define MAGCAL_GOOD_COVERAGE 20
#define MAGCAL_MIN_COVERAGE 14

#define LEVELCAL_SAMPLES 400   /* 2 s still window */
#define LEVELCAL_MAX_SECONDS 12.0f
#define LEVELCAL_MAX_TILT_DEG 15.0f

#define KP_ACC 1.0f
#define KI_ACC 0.02f
#define KP_MAG_PER_SAMPLE 4.0f /* 25 Hz samples -> ~0.6 s^-1 heading gain */
#define G_FILTER 0.25f

#define NVS_NS "pfd"
#define CAL_MAGIC 0x50464431u  /* "PFD1" */

typedef struct {
    uint32_t magic;
    pfd_magcal_t mag[2];
    int8_t source;
} mag_store_t;

#define MOUNT_MAGIC 0x50464433u /* "PFD3": v2.2 display mode, default upright */
typedef struct {
    uint32_t magic;
    uint8_t mode;
} mount_store_t;

#define ALIGN_MAGIC 0x50464141u /* "PFAA" */
typedef struct {
    uint32_t magic;
    pfd_quat_t align;   /* sensor -> board body, from 水平校准 */
    float tilt_deg;
} align_store_t;

typedef struct {
    uint32_t magic;
    float bias[3];
} bias_store_t;

typedef struct {
    i2c_master_dev_handle_t dev;
} bus_dev_t;

typedef struct {
    i2c_master_bus_handle_t bus;
    bool bus_ref;
    bus_dev_t bmi_io;
    struct bmi2_dev bmi;
    bus_dev_t bmm_io[2];
    struct bmm150_dev bmm[2];
    bool bmm_ok[2];

    pfd_ahrs_t ahrs;
    pfd_vec3_t gyro_bias;    /* rad/s */
    pfd_quat_t level;        /* mounting reference M (display -> body) */
    bool level_set;
    uint8_t mount_mode;
    pfd_quat_t align;        /* IMU (mapped) -> board body correction C */
    float align_deg;
    /* flat calibration window */
    int lc_n;
    double lc_g[3], lc_g2[3], lc_a[3], lc_an, lc_an2;
    int64_t lc_started;
    bool lc_moving;
    pfd_magcal_t magcal[2];
    int8_t mag_source;       /* -1, 0, 1, 2 */
    pfd_vec3_t last_acc;
    float g_filt, g_peak;
    float noise[2];
    uint32_t overflow[2];   /* saturated samples per sensor */

    /* gyro calibration window */
    int gc_n;
    double gc_sum[3], gc_sum2[3], gc_acc_sum, gc_acc_sum2;
    int64_t gc_started;

    /* mag calibration buffers (PSRAM) */
    pfd_vec3_t *mc_raw[2];
    pfd_quat_t *mc_q[2];
    int mc_n[2];
    int64_t mc_started;
    int mc_cov;

    /* rest-noise estimate per mag */
    pfd_vec3_t nz_prev[2];
    float nz_acc[2];
    int nz_n[2];

    QueueHandle_t cmds;
    portMUX_TYPE lock;
    pfd_sens_snapshot_t snap;
    int64_t boost_until;     /* fast heading acquisition window */
    uint32_t rate_count;
    int64_t rate_t0;
} sensors_t;

static sensors_t s_ctx = {.lock = portMUX_INITIALIZER_UNLOCKED, .mag_source = -1};

/* ---------- bus glue ---------- */

static int8_t bus_read(uint8_t reg, uint8_t *data, uint32_t len, void *ctx)
{
    bus_dev_t *d = ctx;
    return i2c_master_transmit_receive(d->dev, &reg, 1, data, len, I2C_TIMEOUT_MS) == ESP_OK ? 0 : -1;
}

static int8_t bus_write(uint8_t reg, const uint8_t *data, uint32_t len, void *ctx)
{
    bus_dev_t *d = ctx;
    /* BMI270 config upload uses read_write_len chunks; BMM150 writes 1 byte. */
    uint8_t buf[BMI_WRITE_MAX + 1];
    if (len > BMI_WRITE_MAX) return -1;
    buf[0] = reg;
    memcpy(buf + 1, data, len);
    return i2c_master_transmit(d->dev, buf, len + 1, I2C_TIMEOUT_MS) == ESP_OK ? 0 : -1;
}

static void bus_delay(uint32_t us, void *ctx)
{
    (void)ctx;
    if (us >= 2000) vTaskDelay(pdMS_TO_TICKS((us + 999) / 1000));
    else esp_rom_delay_us(us);
}

static esp_err_t add_dev(uint8_t addr, bus_dev_t *out)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = I2C_SPEED_HZ,
    };
    return i2c_master_bus_add_device(s_ctx.bus, &cfg, &out->dev);
}

/* IMU axes -> aircraft body. +Y_imu is screen-up (Vibe auto-rotate uses it)
 * and +Z_imu points out of the screen (face-up reads +1 g on Z), so +X_imu
 * is screen-right. Body: x forward = into screen, y right, z down. */
static pfd_vec3_t imu_to_body(float x, float y, float z)
{
    return (pfd_vec3_t){-z, x, -y};
}

static esp_err_t bmi_init(void)
{
    uint8_t addr = BMI_ADDR_A;
    if (i2c_master_probe(s_ctx.bus, addr, 50) != ESP_OK) addr = BMI_ADDR_B;
    esp_err_t err = add_dev(addr, &s_ctx.bmi_io);
    if (err != ESP_OK) return err;
    s_ctx.bmi = (struct bmi2_dev){
        .intf = BMI2_I2C_INTF, .intf_ptr = &s_ctx.bmi_io,
        .read = bus_read, .write = bus_write, .delay_us = bus_delay,
        .read_write_len = BMI_WRITE_MAX,
    };
    if (bmi270_init(&s_ctx.bmi) != BMI2_OK) return ESP_FAIL;
    struct bmi2_sens_config cfg[2] = {{.type = BMI2_ACCEL}, {.type = BMI2_GYRO}};
    if (bmi2_get_sensor_config(cfg, 2, &s_ctx.bmi) != BMI2_OK) return ESP_FAIL;
    cfg[0].cfg.acc.odr = BMI2_ACC_ODR_200HZ;
    cfg[0].cfg.acc.range = BMI2_ACC_RANGE_8G;
    cfg[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
    cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_1000;
    cfg[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    cfg[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    uint8_t list[2] = {BMI2_ACCEL, BMI2_GYRO};
    if (bmi2_set_sensor_config(cfg, 2, &s_ctx.bmi) != BMI2_OK ||
            bmi2_sensor_enable(list, 2, &s_ctx.bmi) != BMI2_OK) {
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "BMI270 @0x%02x: 200 Hz, +-8 g, +-1000 dps", addr);
    return ESP_OK;
}

static bool bmm_init(int i)
{
    if (i2c_master_probe(s_ctx.bus, BMM_ADDR[i], 50) != ESP_OK) {
        ESP_LOGW(TAG, "BMM150[%d] @0x%02x not found", i, BMM_ADDR[i]);
        return false;
    }
    if (add_dev(BMM_ADDR[i], &s_ctx.bmm_io[i]) != ESP_OK) return false;
    struct bmm150_dev *d = &s_ctx.bmm[i];
    *d = (struct bmm150_dev){
        .intf = BMM150_I2C_INTF, .intf_ptr = &s_ctx.bmm_io[i],
        .read = bus_read, .write = bus_write, .delay_us = bus_delay,
    };
    if (bmm150_init(d) != BMM150_OK) {
        ESP_LOGW(TAG, "BMM150[%d] init failed", i);
        return false;
    }
    struct bmm150_settings st = {0};
    st.pwr_mode = BMM150_POWERMODE_NORMAL;
    st.preset_mode = BMM150_PRESETMODE_REGULAR;
    if (bmm150_set_op_mode(&st, d) != BMM150_OK || bmm150_set_presetmode(&st, d) != BMM150_OK) {
        return false;
    }
    st.data_rate = BMM150_DATA_RATE_25HZ;
    if (bmm150_set_sensor_settings(BMM150_SEL_DATA_RATE, &st, d) != BMM150_OK) return false;
    ESP_LOGI(TAG, "BMM150[%d] @0x%02x: 25 Hz regular preset", i, BMM_ADDR[i]);
    return true;
}

/* ---------- persistence ---------- */

static void nvs_save(const char *key, const void *data, size_t len)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    esp_err_t err = nvs_set_blob(h, key, data, len);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) ESP_LOGW(TAG, "save %s: %s", key, esp_err_to_name(err));
}

static bool nvs_load(const char *key, void *data, size_t len)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return false;
    size_t got = len;
    esp_err_t err = nvs_get_blob(h, key, data, &got);
    nvs_close(h);
    return err == ESP_OK && got == len;
}

static void load_settings(void)
{
    mag_store_t m;
    if (nvs_load("mag", &m, sizeof(m)) && m.magic == CAL_MAGIC) {
        s_ctx.magcal[0] = m.mag[0];
        s_ctx.magcal[1] = m.mag[1];
        s_ctx.mag_source = m.source;
        ESP_LOGI(TAG, "mag cal loaded: source=%d resid=%.3f/%.3f", m.source,
                 m.mag[0].residual, m.mag[1].residual);
    }
    /* v1 "level" and v2.1 "mount" keys are ignored: v2.2 defaults to upright. */
    mount_store_t l;
    if (nvs_load("mount3", &l, sizeof(l)) && l.magic == MOUNT_MAGIC && l.mode <= PFD_MOUNT_UPRIGHT)
        s_ctx.mount_mode = l.mode;
    else
        s_ctx.mount_mode = PFD_MOUNT_UPRIGHT;
    s_ctx.level = pfd_mount_preset(s_ctx.mount_mode);
    align_store_t al;
    s_ctx.align = (pfd_quat_t){1, 0, 0, 0};
    if (nvs_load("align", &al, sizeof(al)) && al.magic == ALIGN_MAGIC && al.tilt_deg <= LEVELCAL_MAX_TILT_DEG) {
        s_ctx.align = pfd_quat_normalize(al.align);
        s_ctx.align_deg = al.tilt_deg;
        s_ctx.level_set = true;
    }
    ESP_LOGI(TAG, "display mode: %s; IMU alignment: %s (%.2f deg) q=(%.4f %.4f %.4f %.4f)",
             s_ctx.mount_mode == PFD_MOUNT_FLAT ? "flat" : "upright", s_ctx.level_set ? "calibrated" : "none",
             s_ctx.align_deg, s_ctx.align.w, s_ctx.align.x, s_ctx.align.y, s_ctx.align.z);
}

/* ---------- helpers ---------- */

static bool mag_used(int i)
{
    return s_ctx.bmm_ok[i] && (s_ctx.mag_source == i || s_ctx.mag_source == 2) && s_ctx.magcal[i].valid;
}

static void publish(pfd_sens_state_t state, float progress)
{
    pfd_quat_t qd = pfd_mount_apply(s_ctx.ahrs.q, s_ctx.level);
    float r, p, y;
    pfd_quat_to_euler(qd, &r, &p, &y);
    portENTER_CRITICAL(&s_ctx.lock);
    pfd_sens_snapshot_t *s = &s_ctx.snap;
    s->state = state;
    s->roll_deg = r * R2D;
    s->pitch_deg = p * R2D;
    s->heading_deg = y * R2D;
    s->g_now = s_ctx.g_filt;
    s->g_peak = s_ctx.g_peak;
    s->progress = progress;
    s->mag_calibrated = s_ctx.mag_source >= 0;
    s->mag_source = s_ctx.mag_source;
    s->mag_residual[0] = s_ctx.magcal[0].residual;
    s->mag_residual[1] = s_ctx.magcal[1].residual;
    s->mag_noise_ut[0] = s_ctx.noise[0];
    s->mag_noise_ut[1] = s_ctx.noise[1];
    s->mag_ok[0] = s_ctx.bmm_ok[0];
    s->mag_ok[1] = s_ctx.bmm_ok[1];
    s->mag_overflow[0] = s_ctx.overflow[0];
    s->mag_overflow[1] = s_ctx.overflow[1];
    s->level_set = s_ctx.level_set;
    s->mount_mode = s_ctx.mount_mode;
    s->align_deg = s_ctx.align_deg;
    if (state == PFD_SENS_LEVELCAL) {
        float left = (float)(LEVELCAL_SAMPLES - s_ctx.lc_n) / RATE_HZ;
        s->levelcal_remaining_s = left > 0 ? left : 0;
        s->levelcal_moving = s_ctx.lc_moving;
    }
    portEXIT_CRITICAL(&s_ctx.lock);
}

static void set_magcal_result(pfd_magcal_result_t r, int coverage, float secs)
{
    portENTER_CRITICAL(&s_ctx.lock);
    s_ctx.snap.magcal_result = r;
    s_ctx.snap.magcal_result_seq++;
    s_ctx.snap.magcal_coverage = coverage;
    s_ctx.snap.magcal_seconds = secs;
    portEXIT_CRITICAL(&s_ctx.lock);
}

static void track_noise(int i, pfd_vec3_t raw, bool still)
{
    /* RMS of sample-to-sample differences / sqrt(2) while the board rests. */
    if (still && s_ctx.nz_n[i] > 0) {
        float dx = raw.x - s_ctx.nz_prev[i].x, dy = raw.y - s_ctx.nz_prev[i].y, dz = raw.z - s_ctx.nz_prev[i].z;
        float d2 = (dx * dx + dy * dy + dz * dz) / 6.0f;
        s_ctx.nz_acc[i] = s_ctx.nz_acc[i] * 0.98f + d2 * 0.02f;
        s_ctx.noise[i] = sqrtf(s_ctx.nz_acc[i]);
    }
    s_ctx.nz_prev[i] = raw;
    s_ctx.nz_n[i]++;
}

/* ---------- magnetometer calibration ---------- */

static void magcal_begin(void)
{
    for (int i = 0; i < 2; ++i) {
        if (!s_ctx.mc_raw[i]) {
            s_ctx.mc_raw[i] = heap_caps_malloc(sizeof(pfd_vec3_t) * MAGCAL_MAX, MALLOC_CAP_SPIRAM);
            s_ctx.mc_q[i] = heap_caps_malloc(sizeof(pfd_quat_t) * MAGCAL_MAX, MALLOC_CAP_SPIRAM);
        }
        s_ctx.mc_n[i] = 0;
    }
    s_ctx.mc_started = esp_timer_get_time();
    s_ctx.mc_cov = 0;
    ESP_LOGI(TAG, "mag calibration started");
}

static void magcal_finish(float secs)
{
    pfd_magcal_t cal[2] = {0};
    bool ok[2] = {false, false};
    int cov[2] = {0, 0};
    for (int i = 0; i < 2; ++i) {
        if (!s_ctx.bmm_ok[i] || s_ctx.mc_n[i] < 60) continue;
        cov[i] = pfd_magcal_coverage(s_ctx.mc_raw[i], s_ctx.mc_n[i]);
        if (cov[i] < MAGCAL_MIN_COVERAGE) continue;
        ok[i] = pfd_magcal_fit(s_ctx.mc_raw[i], s_ctx.mc_n[i], &cal[i]) &&
                pfd_magcal_align(&cal[i], s_ctx.mc_raw[i], s_ctx.mc_q[i], s_ctx.mc_n[i]);
        ESP_LOGI(TAG, "magcal[%d]: n=%d cov=%d ok=%d ellipsoid=%d center=(%.1f,%.1f,%.1f) field=%.1fuT "
                 "resid=%.3f rot=%d align=%.3f dip=%.1f", i, s_ctx.mc_n[i], cov[i], ok[i], cal[i].ellipsoid,
                 cal[i].center[0], cal[i].center[1], cal[i].center[2], cal[i].field, cal[i].residual,
                 cal[i].rotation, cal[i].align_score, cal[i].dip_deg);
        /* A wrong axis guess or a disturbed sensor leaves a large spread. */
        if (ok[i] && cal[i].align_score > 0.20f) ok[i] = false;
    }
    int best_cov = cov[0] > cov[1] ? cov[0] : cov[1];
    if (!ok[0] && !ok[1]) {
        set_magcal_result(best_cov < MAGCAL_MIN_COVERAGE ? PFD_MAGCAL_POOR_COVERAGE : PFD_MAGCAL_FIT_FAILED,
                          best_cov, secs);
        ESP_LOGW(TAG, "mag calibration failed (coverage %d)", best_cov);
        return;
    }
    int8_t source;
    if (ok[0] && ok[1]) {
        float q0 = cal[0].align_score + cal[0].residual, q1 = cal[1].align_score + cal[1].residual;
        /* Average both only when they agree in quality; else take the cleaner. */
        if (q0 < 0.10f && q1 < 0.10f && q0 < q1 * 1.6f && q1 < q0 * 1.6f) source = 2;
        else source = q0 <= q1 ? 0 : 1;
    } else {
        source = ok[0] ? 0 : 1;
    }
    for (int i = 0; i < 2; ++i) {
        cal[i].valid = ok[i];
        s_ctx.magcal[i] = cal[i];
    }
    s_ctx.mag_source = source;
    mag_store_t m = {.magic = CAL_MAGIC, .mag = {cal[0], cal[1]}, .source = source};
    nvs_save("mag", &m, sizeof(m));
    s_ctx.boost_until = esp_timer_get_time() + 3000000;
    set_magcal_result(PFD_MAGCAL_OK, best_cov, secs);
    ESP_LOGI(TAG, "mag calibration ok: source=%d", source);
}

/* ---------- flat calibration (水平校准) ---------- */

static void levelcal_reset(void)
{
    s_ctx.lc_n = 0;
    memset(s_ctx.lc_g, 0, sizeof(s_ctx.lc_g));
    memset(s_ctx.lc_g2, 0, sizeof(s_ctx.lc_g2));
    memset(s_ctx.lc_a, 0, sizeof(s_ctx.lc_a));
    s_ctx.lc_an = s_ctx.lc_an2 = 0;
}

static void set_levelcal_result(pfd_levelcal_result_t r, float tilt)
{
    portENTER_CRITICAL(&s_ctx.lock);
    s_ctx.snap.levelcal_result = r;
    s_ctx.snap.levelcal_result_seq++;
    s_ctx.snap.levelcal_tilt_deg = tilt;
    portEXIT_CRITICAL(&s_ctx.lock);
}

/* Raw (mapped, uncorrected) gyro/accel while the board lies on its back.
 * Returns true when the calibration finished (any result). */
static bool levelcal_step(pfd_vec3_t g, pfd_vec3_t a)
{
    const double gv[3] = {g.x * R2D, g.y * R2D, g.z * R2D}, av[3] = {a.x, a.y, a.z};
    double an = sqrt(av[0] * av[0] + av[1] * av[1] + av[2] * av[2]);
    for (int k = 0; k < 3; ++k) {
        s_ctx.lc_g[k] += gv[k];
        s_ctx.lc_g2[k] += gv[k] * gv[k];
        s_ctx.lc_a[k] += av[k];
    }
    s_ctx.lc_an += an;
    s_ctx.lc_an2 += an * an;
    s_ctx.lc_n++;
    float secs = (esp_timer_get_time() - s_ctx.lc_started) / 1e6f;
    /* check stillness every 0.25 s so a bump restarts the window quickly */
    if (s_ctx.lc_n % 50 == 0 || s_ctx.lc_n >= LEVELCAL_SAMPLES) {
        double n = s_ctx.lc_n, worst = 0;
        for (int k = 0; k < 3; ++k) {
            double m = s_ctx.lc_g[k] / n, var = s_ctx.lc_g2[k] / n - m * m;
            double sd = sqrt(var > 0 ? var : 0);
            if (sd > worst) worst = sd;
        }
        double am = s_ctx.lc_an / n, asd = sqrt(fmax(0, s_ctx.lc_an2 / n - am * am));
        if (worst > GYRO_STILL_DPS || asd > ACC_STILL_G || fabs(am - 1.0) > 0.1) {
            levelcal_reset();
            s_ctx.lc_moving = true;
            if (secs > LEVELCAL_MAX_SECONDS) {
                set_levelcal_result(PFD_LEVELCAL_MOVING, 0);
                ESP_LOGW(TAG, "flat calibration: board kept moving");
                return true;
            }
            return false;
        }
    }
    if (s_ctx.lc_n < LEVELCAL_SAMPLES) return false;

    double n = s_ctx.lc_n;
    pfd_vec3_t am = {(float)(s_ctx.lc_a[0] / n), (float)(s_ctx.lc_a[1] / n), (float)(s_ctx.lc_a[2] / n)};
    float tilt;
    pfd_quat_t C = pfd_flatcal_solve(am, &tilt);
    if (tilt > LEVELCAL_MAX_TILT_DEG) {
        set_levelcal_result(PFD_LEVELCAL_NOT_FLAT, tilt);
        ESP_LOGW(TAG, "flat calibration rejected: %.1f deg from flat (acc %.3f %.3f %.3f)", tilt, am.x, am.y, am.z);
        return true;
    }
    /* keep the fused attitude continuous: same physical sensor attitude,
     * re-expressed with the new correction */
    pfd_quat_t q_sensor = pfd_quat_mul(s_ctx.ahrs.q, s_ctx.align);
    s_ctx.ahrs.q = pfd_quat_normalize(pfd_quat_mul(q_sensor, pfd_quat_conj(C)));
    s_ctx.align = C;
    s_ctx.align_deg = tilt;
    s_ctx.level_set = true;
    s_ctx.gyro_bias = (pfd_vec3_t){(float)(s_ctx.lc_g[0] / n) * D2R, (float)(s_ctx.lc_g[1] / n) * D2R,
                                   (float)(s_ctx.lc_g[2] / n) * D2R};
    s_ctx.ahrs.integral = (pfd_vec3_t){0, 0, 0};
    align_store_t al = {.magic = ALIGN_MAGIC, .align = C, .tilt_deg = tilt};
    nvs_save("align", &al, sizeof(al));
    bias_store_t b = {CAL_MAGIC, {s_ctx.gyro_bias.x, s_ctx.gyro_bias.y, s_ctx.gyro_bias.z}};
    nvs_save("gbias", &b, sizeof(b));
    /* express the correction as small angles about the board axes */
    float rr, pp, yy;
    pfd_quat_to_euler(C, &rr, &pp, &yy);
    ESP_LOGI(TAG, "flat calibration OK: acc mean %.4f %.4f %.4f g, IMU tilt %.2f deg "
             "(about screen-vertical %.2f, screen-horizontal %.2f), gyro bias %.3f %.3f %.3f dps, %.1f s",
             am.x, am.y, am.z, tilt, (yy > (float)M_PI ? yy - 2 * (float)M_PI : yy) * R2D, pp * R2D,
             s_ctx.gyro_bias.x * R2D, s_ctx.gyro_bias.y * R2D, s_ctx.gyro_bias.z * R2D, secs);
    set_levelcal_result(PFD_LEVELCAL_OK, tilt);
    return true;
}

/* ---------- commands ---------- */

static void handle_command(pfd_sens_cmd_t cmd, pfd_sens_state_t *state)
{
    switch (cmd) {
    case PFD_CMD_LEVEL:
        if (*state == PFD_SENS_RUN) {
            levelcal_reset();
            s_ctx.lc_started = esp_timer_get_time();
            s_ctx.lc_moving = false;
            *state = PFD_SENS_LEVELCAL;
            ESP_LOGI(TAG, "flat calibration started");
        }
        break;
    case PFD_CMD_LEVEL_CANCEL:
        if (*state == PFD_SENS_LEVELCAL) {
            *state = PFD_SENS_RUN;
            set_levelcal_result(PFD_LEVELCAL_CANCELLED, 0);
        }
        break;
    case PFD_CMD_MOUNT_FLAT:
    case PFD_CMD_MOUNT_UPRIGHT: {
        uint8_t mode = cmd == PFD_CMD_MOUNT_FLAT ? PFD_MOUNT_FLAT : PFD_MOUNT_UPRIGHT;
        portENTER_CRITICAL(&s_ctx.lock);
        s_ctx.level = pfd_mount_preset(mode);
        s_ctx.mount_mode = mode;
        portEXIT_CRITICAL(&s_ctx.lock);
        mount_store_t l = {.magic = MOUNT_MAGIC, .mode = mode};
        nvs_save("mount3", &l, sizeof(l));
        ESP_LOGI(TAG, "display mode: %s", mode == PFD_MOUNT_FLAT ? "flat" : "upright");
        break;
    }
    case PFD_CMD_MAGCAL_START:
        if (*state == PFD_SENS_RUN && (s_ctx.bmm_ok[0] || s_ctx.bmm_ok[1])) {
            magcal_begin();
            if (s_ctx.mc_raw[0] && s_ctx.mc_q[0] && s_ctx.mc_raw[1] && s_ctx.mc_q[1]) *state = PFD_SENS_MAGCAL;
        } else if (!s_ctx.bmm_ok[0] && !s_ctx.bmm_ok[1]) {
            set_magcal_result(PFD_MAGCAL_FIT_FAILED, 0, 0);
        }
        break;
    case PFD_CMD_MAGCAL_CANCEL:
        if (*state == PFD_SENS_MAGCAL) {
            *state = PFD_SENS_RUN;
            set_magcal_result(PFD_MAGCAL_CANCELLED, s_ctx.mc_cov, 0);
        }
        break;
    case PFD_CMD_PEAK_RESET:
        s_ctx.g_peak = s_ctx.g_filt;
        break;
    }
}

/* ---------- main loop ---------- */

static bool read_imu(pfd_vec3_t *gyro_rad, pfd_vec3_t *acc_g)
{
    struct bmi2_sens_data d = {0};
    if (bmi2_get_sensor_data(&d, &s_ctx.bmi) != BMI2_OK) return false;
    *acc_g = imu_to_body(d.acc.x * ACC_LSB_8G, d.acc.y * ACC_LSB_8G, d.acc.z * ACC_LSB_8G);
    pfd_vec3_t g = imu_to_body(d.gyr.x * GYR_LSB_1000, d.gyr.y * GYR_LSB_1000, d.gyr.z * GYR_LSB_1000);
    *gyro_rad = (pfd_vec3_t){g.x * D2R, g.y * D2R, g.z * D2R};
    return true;
}

static bool read_mag(int i, pfd_vec3_t *raw)
{
    struct bmm150_mag_data m;
    if (bmm150_read_mag_data(&m, &s_ctx.bmm[i]) != BMM150_OK) return false;
    /* The Bosch float compensation reports an overflowed axis as exactly 0.
     * The docking magnets can saturate an axis; drop such samples entirely. */
    if (m.x == 0.0f || m.y == 0.0f || m.z == 0.0f) {
        s_ctx.overflow[i]++;
        return false;
    }
    *raw = (pfd_vec3_t){m.x, m.y, m.z};
    return true;
}

static void gyro_cal_step(pfd_vec3_t g, pfd_vec3_t a, pfd_sens_state_t *state)
{
    const double v[3] = {g.x * R2D, g.y * R2D, g.z * R2D};
    double an = sqrt((double)a.x * a.x + (double)a.y * a.y + (double)a.z * a.z);
    for (int k = 0; k < 3; ++k) {
        s_ctx.gc_sum[k] += v[k];
        s_ctx.gc_sum2[k] += v[k] * v[k];
    }
    s_ctx.gc_acc_sum += an;
    s_ctx.gc_acc_sum2 += an * an;
    s_ctx.gc_n++;
    float elapsed = (esp_timer_get_time() - s_ctx.gc_started) / 1e6f;
    publish(PFD_SENS_GYRO_CAL, (float)s_ctx.gc_n / GYRO_CAL_SAMPLES);
    if (s_ctx.gc_n < GYRO_CAL_SAMPLES) return;

    double n = s_ctx.gc_n, worst = 0;
    double mean[3];
    for (int k = 0; k < 3; ++k) {
        mean[k] = s_ctx.gc_sum[k] / n;
        double var = s_ctx.gc_sum2[k] / n - mean[k] * mean[k];
        double sd = sqrt(var > 0 ? var : 0);
        if (sd > worst) worst = sd;
    }
    double am = s_ctx.gc_acc_sum / n;
    double asd = sqrt(fmax(0, s_ctx.gc_acc_sum2 / n - am * am));
    bool still = worst < GYRO_STILL_DPS && asd < ACC_STILL_G;
    if (!still && elapsed < GYRO_CAL_MAX_SECONDS) {
        memset(s_ctx.gc_sum, 0, sizeof(s_ctx.gc_sum));
        memset(s_ctx.gc_sum2, 0, sizeof(s_ctx.gc_sum2));
        s_ctx.gc_acc_sum = s_ctx.gc_acc_sum2 = 0;
        s_ctx.gc_n = 0;
        return;
    }
    bias_store_t b;
    if (still) {
        s_ctx.gyro_bias = (pfd_vec3_t){(float)mean[0] * D2R, (float)mean[1] * D2R, (float)mean[2] * D2R};
        b = (bias_store_t){CAL_MAGIC, {s_ctx.gyro_bias.x, s_ctx.gyro_bias.y, s_ctx.gyro_bias.z}};
        nvs_save("gbias", &b, sizeof(b));
    } else if (nvs_load("gbias", &b, sizeof(b)) && b.magic == CAL_MAGIC) {
        s_ctx.gyro_bias = (pfd_vec3_t){b.bias[0], b.bias[1], b.bias[2]};
    }
    ESP_LOGI(TAG, "gyro bias %s: %.3f %.3f %.3f dps (std %.3f dps, |a| %.3f g sd %.4f)",
             still ? "measured" : "from NVS (board was moving)",
             s_ctx.gyro_bias.x * R2D, s_ctx.gyro_bias.y * R2D, s_ctx.gyro_bias.z * R2D, worst, am, asd);
    /* Orientation sanity: upright = (0,0,-1), face-up on a table = (-1,0,0). */
    ESP_LOGI(TAG, "accel body frame at rest: %+.3f %+.3f %+.3f g", a.x, a.y, a.z);
    *state = PFD_SENS_RUN;
    s_ctx.boost_until = esp_timer_get_time() + 3000000;
}

static void sensor_task(void *arg)
{
    (void)arg;
    pfd_sens_state_t state = PFD_SENS_GYRO_CAL;
    pfd_ahrs_init(&s_ctx.ahrs, KP_ACC, KI_ACC, KP_MAG_PER_SAMPLE);
    s_ctx.gc_started = esp_timer_get_time();
    s_ctx.rate_t0 = s_ctx.gc_started;
    TickType_t wake = xTaskGetTickCount();
    int64_t last_us = esp_timer_get_time();
    uint32_t tick = 0;
    int raw_logs = 0;
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_MS));
        int64_t now = esp_timer_get_time();
        float dt = (now - last_us) / 1e6f;
        last_us = now;
        if (dt <= 0 || dt > 0.05f) dt = 1.0f / RATE_HZ;

        pfd_sens_cmd_t cmd;
        while (xQueueReceive(s_ctx.cmds, &cmd, 0) == pdTRUE) handle_command(cmd, &state);

        pfd_vec3_t g, a;
        if (!read_imu(&g, &a)) continue;
        s_ctx.last_acc = a;
        float an = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
        s_ctx.g_filt += (an - s_ctx.g_filt) * G_FILTER;
        if (state == PFD_SENS_RUN && s_ctx.g_filt > s_ctx.g_peak) s_ctx.g_peak = s_ctx.g_filt;
        bool still = fabsf(g.x) + fabsf(g.y) + fabsf(g.z) < 0.05f && fabsf(an - 1.0f) < 0.03f;

        /* Magnetometer: one sensor every MAG_EVERY ticks, alternating. */
        const pfd_vec3_t *mag_in = NULL;
        pfd_vec3_t mag_body;
        if (++tick % MAG_EVERY == 0) {
            int i = (tick / MAG_EVERY) & 1;
            pfd_vec3_t raw;
            if (s_ctx.bmm_ok[i] && read_mag(i, &raw)) {
                track_noise(i, raw, still);
                if (raw_logs < 6) {
                    ESP_LOGI(TAG, "raw mag[%d] %.1f %.1f %.1f uT", i, raw.x, raw.y, raw.z);
                    raw_logs++;
                }
                if (state == PFD_SENS_MAGCAL && s_ctx.mc_n[i] < MAGCAL_MAX) {
                    s_ctx.mc_raw[i][s_ctx.mc_n[i]] = raw;
                    /* attitude of the mapped sensor frame (alignment removed) */
                    s_ctx.mc_q[i][s_ctx.mc_n[i]] = pfd_quat_mul(s_ctx.ahrs.q, s_ctx.align);
                    s_ctx.mc_n[i]++;
                } else if ((state == PFD_SENS_RUN || state == PFD_SENS_LEVELCAL) && mag_used(i)) {
                    mag_body = pfd_quat_rotate(s_ctx.align, pfd_magcal_to_body(&s_ctx.magcal[i], raw));
                    mag_in = &mag_body;
                }
            }
        }

        if (state == PFD_SENS_GYRO_CAL) {
            gyro_cal_step(g, a, &state);
            if (state == PFD_SENS_RUN) {
                /* Start level from gravity; heading converges from mag. */
                pfd_ahrs_reset(&s_ctx.ahrs, pfd_quat_rotate(s_ctx.align, a), NULL);
                s_ctx.g_peak = s_ctx.g_filt;
            }
            continue;
        }

        bool lc_done = state == PFD_SENS_LEVELCAL && levelcal_step(g, a);
        if (lc_done) state = PFD_SENS_RUN;
        /* bias in the mapped sensor frame, then rotate into the board body */
        pfd_vec3_t gc = pfd_quat_rotate(s_ctx.align, (pfd_vec3_t){g.x - s_ctx.gyro_bias.x, g.y - s_ctx.gyro_bias.y,
                                                                  g.z - s_ctx.gyro_bias.z});
        a = pfd_quat_rotate(s_ctx.align, a);
        float kp_mag = KP_MAG_PER_SAMPLE * (s_ctx.mag_source == 2 ? 0.5f : 1.0f);
        if (now < s_ctx.boost_until) kp_mag *= 6.0f;
        s_ctx.ahrs.kp_mag = kp_mag;
        pfd_ahrs_update(&s_ctx.ahrs, gc, a, mag_in, dt);

        float progress = 0;
        if (state == PFD_SENS_LEVELCAL) progress = (float)s_ctx.lc_n / LEVELCAL_SAMPLES;
        if (state == PFD_SENS_MAGCAL) {
            float secs = (now - s_ctx.mc_started) / 1e6f;
            if (tick % 50 == 0) {
                int c0 = s_ctx.bmm_ok[0] ? pfd_magcal_coverage(s_ctx.mc_raw[0], s_ctx.mc_n[0]) : 0;
                int c1 = s_ctx.bmm_ok[1] ? pfd_magcal_coverage(s_ctx.mc_raw[1], s_ctx.mc_n[1]) : 0;
                s_ctx.mc_cov = c0 > c1 ? c0 : c1;
                portENTER_CRITICAL(&s_ctx.lock);
                s_ctx.snap.magcal_coverage = s_ctx.mc_cov;
                s_ctx.snap.magcal_seconds = secs;
                portEXIT_CRITICAL(&s_ctx.lock);
            }
            float pc = (float)s_ctx.mc_cov / MAGCAL_GOOD_COVERAGE;
            float pt = secs / MAGCAL_TARGET_SECONDS;
            progress = fminf(1.0f, fminf(pc, 1.0f) * 0.6f + fminf(pt, 1.0f) * 0.4f);
            bool done = (secs >= MAGCAL_TARGET_SECONDS && s_ctx.mc_cov >= MAGCAL_GOOD_COVERAGE) ||
                        (secs >= MAGCAL_MIN_SECONDS && s_ctx.mc_cov >= 24) || secs >= MAGCAL_MAX_SECONDS ||
                        s_ctx.mc_n[0] >= MAGCAL_MAX || s_ctx.mc_n[1] >= MAGCAL_MAX;
            if (done) {
                magcal_finish(secs);
                state = PFD_SENS_RUN;
            }
        }
        publish(state, progress);

        s_ctx.rate_count++;
        if (now - s_ctx.rate_t0 >= 1000000) {
            float hz = s_ctx.rate_count * 1e6f / (float)(now - s_ctx.rate_t0);
            portENTER_CRITICAL(&s_ctx.lock);
            s_ctx.snap.imu_rate_hz = hz;
            portEXIT_CRITICAL(&s_ctx.lock);
            s_ctx.rate_count = 0;
            s_ctx.rate_t0 = now;
        }
    }
}

int pfd_sensors_start(void)
{
    if (s_ctx.cmds) return 0;
    s_ctx.cmds = xQueueCreate(8, sizeof(pfd_sens_cmd_t));
    if (!s_ctx.cmds) return -1;
    s_ctx.level = pfd_mount_preset(PFD_MOUNT_UPRIGHT);
    s_ctx.mount_mode = PFD_MOUNT_UPRIGHT;
    s_ctx.align = (pfd_quat_t){1, 0, 0, 0};
    esp_err_t err = esp_board_periph_ref_handle(I2C_BUS_NAME, (void **)&s_ctx.bus);
    if (err != ESP_OK || !s_ctx.bus) {
        ESP_LOGE(TAG, "I2C bus unavailable: %s", esp_err_to_name(err));
        s_ctx.snap.state = PFD_SENS_ERROR;
        return 0;
    }
    s_ctx.bus_ref = true;
    if (bmi_init() != ESP_OK) {
        ESP_LOGE(TAG, "BMI270 init failed");
        s_ctx.snap.state = PFD_SENS_ERROR;
        return 0;
    }
    s_ctx.snap.imu_ok = true;
    for (int i = 0; i < 2; ++i) s_ctx.bmm_ok[i] = bmm_init(i);
    load_settings();
    /* Pinned to core 1 above the render loop so sampling never jitters. */
    if (xTaskCreatePinnedToCore(sensor_task, "pfd_sens", 6144, NULL, 12, NULL, 1) != pdPASS) {
        s_ctx.snap.state = PFD_SENS_ERROR;
        return -1;
    }
    return 0;
}

void pfd_sensors_read(pfd_sens_snapshot_t *out)
{
    portENTER_CRITICAL(&s_ctx.lock);
    *out = s_ctx.snap;
    portEXIT_CRITICAL(&s_ctx.lock);
}

void pfd_sensors_command(pfd_sens_cmd_t cmd)
{
    if (s_ctx.cmds) (void)xQueueSend(s_ctx.cmds, &cmd, 0);
}
