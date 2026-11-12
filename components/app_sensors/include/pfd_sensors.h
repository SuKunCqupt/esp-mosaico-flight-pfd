// SPDX-License-Identifier: MIT
#pragma once
/* Attitude sensor service. On the device one task owns BMI270 + both
 * BMM150s on the Board Manager I2C bus and runs fusion at 200 Hz; the host
 * build animates a synthetic attitude. All functions are thread-safe:
 * snapshots are copied under a spinlock, commands are queued. */
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PFD_SENS_STARTING = 0,
    PFD_SENS_GYRO_CAL,   /* hold still, progress 0..1 */
    PFD_SENS_RUN,
    PFD_SENS_MAGCAL,     /* rotate board, progress 0..1 */
    PFD_SENS_LEVELCAL,   /* 水平校准: lying flat on its back, hold still */
    PFD_SENS_ERROR,
} pfd_sens_state_t;

typedef enum {
    PFD_MAGCAL_NONE = 0,
    PFD_MAGCAL_OK,
    PFD_MAGCAL_CANCELLED,
    PFD_MAGCAL_POOR_COVERAGE,
    PFD_MAGCAL_FIT_FAILED,
} pfd_magcal_result_t;

typedef enum {
    PFD_LEVELCAL_NONE = 0,
    PFD_LEVELCAL_OK,
    PFD_LEVELCAL_NOT_FLAT,   /* > 15 deg from lying flat on its back */
    PFD_LEVELCAL_MOVING,     /* never still long enough */
    PFD_LEVELCAL_CANCELLED,
} pfd_levelcal_result_t;

typedef struct {
    pfd_sens_state_t state;
    float roll_deg;      /* + = right wing down; display frame (level offset applied) */
    float pitch_deg;     /* + = nose up */
    float heading_deg;   /* 0..360, magnetic if mag_calibrated else gyro-relative */
    float g_now;         /* |specific force| in g, lightly filtered */
    float g_peak;
    float imu_rate_hz;   /* measured fusion update rate */
    float progress;      /* gyro-cal or mag-cal progress 0..1 */
    int magcal_coverage; /* 0..26 direction bins */
    float magcal_seconds;
    pfd_magcal_result_t magcal_result;
    uint32_t magcal_result_seq; /* increments per finished calibration */
    bool mag_calibrated;
    int8_t mag_source;   /* -1 none, 0/1 single sensor, 2 both */
    float mag_residual[2];
    float mag_noise_ut[2]; /* RMS jitter of each sensor at rest */
    bool imu_ok;
    bool mag_ok[2];
    uint32_t mag_overflow[2]; /* samples dropped for a saturated axis */
    bool level_set;          /* flat calibration (IMU alignment) in use */
    uint8_t mount_mode;      /* display: 0 flat, 1 upright (default) */
    /* 水平校准 (flat calibration) */
    float levelcal_remaining_s;
    bool levelcal_moving;    /* window restarted because the board moved */
    pfd_levelcal_result_t levelcal_result;
    uint32_t levelcal_result_seq;
    float levelcal_tilt_deg; /* table/board tilt seen by the last attempt */
    float align_deg;         /* applied IMU misalignment correction */
} pfd_sens_snapshot_t;

typedef enum {
    PFD_CMD_LEVEL = 1,        /* start flat calibration (board on its back, level table) */
    PFD_CMD_LEVEL_CANCEL,
    PFD_CMD_MOUNT_FLAT,       /* display mode: lying on its back, nose = top edge */
    PFD_CMD_MOUNT_UPRIGHT,    /* display mode: screen facing the pilot (default) */
    PFD_CMD_MAGCAL_START,
    PFD_CMD_MAGCAL_CANCEL,
    PFD_CMD_PEAK_RESET,
} pfd_sens_cmd_t;

/* Start the service once; returns 0 on success (device: also on partial
 * hardware failure, reported through snapshot flags). */
int pfd_sensors_start(void);
void pfd_sensors_read(pfd_sens_snapshot_t *out);
void pfd_sensors_command(pfd_sens_cmd_t cmd);

#ifdef __cplusplus
}
#endif
