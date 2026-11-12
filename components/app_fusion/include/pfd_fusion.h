// SPDX-License-Identifier: MIT
#pragma once
/* Attitude math for the flight display. Pure C, no hardware or RTOS
 * dependencies, so the same code runs in host tests and on the device.
 * Frames: body = aircraft NED (x forward/into screen, y right, z down);
 * nav = local NED (x north, y east, z down). Quaternions rotate body->nav.
 * Units: gyro rad/s, accel g (specific force; level at rest = (0,0,-1)),
 * magnetometer any consistent unit (uT). Not thread-safe: one owner. */
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { float w, x, y, z; } pfd_quat_t;
typedef struct { float x, y, z; } pfd_vec3_t;

typedef struct {
    pfd_quat_t q;
    pfd_vec3_t integral;   /* rad/s, slow gyro-bias correction */
    float kp_acc;          /* accel proportional gain (1/s) */
    float ki_acc;          /* accel integral gain (1/s^2) */
    float kp_mag;          /* yaw-only magnetometer gain (1/s) */
    float acc_weight;      /* last applied accel trust 0..1 (diagnostic) */
    bool initialized;
} pfd_ahrs_t;

void pfd_ahrs_init(pfd_ahrs_t *f, float kp_acc, float ki_acc, float kp_mag);
/* Set the attitude directly from gravity (and heading from mag if given). */
void pfd_ahrs_reset(pfd_ahrs_t *f, pfd_vec3_t acc, const pfd_vec3_t *mag);
/* One Mahony step. mag may be NULL (gyro/accel only). Magnetometer error is
 * projected onto the vertical so it can only correct heading. */
void pfd_ahrs_update(pfd_ahrs_t *f, pfd_vec3_t gyro, pfd_vec3_t acc,
                     const pfd_vec3_t *mag, float dt);

pfd_quat_t pfd_quat_mul(pfd_quat_t a, pfd_quat_t b);
pfd_quat_t pfd_quat_conj(pfd_quat_t q);
pfd_quat_t pfd_quat_normalize(pfd_quat_t q);
pfd_vec3_t pfd_quat_rotate(pfd_quat_t q, pfd_vec3_t v);      /* body->nav */
pfd_vec3_t pfd_quat_rotate_inv(pfd_quat_t q, pfd_vec3_t v);  /* nav->body */
pfd_quat_t pfd_quat_from_axis_angle(pfd_vec3_t axis, float angle);
/* ZYX Euler, radians: roll (-pi..pi), pitch (-pi/2..pi/2), yaw (0..2pi). */
void pfd_quat_to_euler(pfd_quat_t q, float *roll, float *pitch, float *yaw);
pfd_quat_t pfd_quat_from_euler(float roll, float pitch, float yaw);

/* Level offset: rotation Q (b' -> b) that makes the current down vector the
 * new body z axis with minimal rotation. Apply as q_display = q * Q. */
pfd_quat_t pfd_level_offset(pfd_quat_t q_current);

/* Mounting reference: a full rotation M (display frame -> sensor body frame).
 * Display attitude = q * M, i.e. the fused body->nav attitude re-expressed in
 * the display axes. Because M is a fixed body-side rotation (not a nav-side
 * "reference^-1 * current"), heading stays absolute and tilt-compensated, and
 * the magnetometer calibration (sensor/body frame) stays valid.
 *  FLAT:    lying on its back, screen up; nose = top edge, right = right edge,
 *           down = into the screen.  M = rotation about body y by +90 deg.
 *  UPRIGHT: screen facing the pilot; nose = into the screen.  M = identity.
 *  CUSTOM:  from pfd_mount_from_pose(): nearest preset, then the minimal
 *           tilt that makes the current pose read pitch 0 / roll 0. */
typedef enum { PFD_MOUNT_FLAT = 0, PFD_MOUNT_UPRIGHT = 1, PFD_MOUNT_CUSTOM = 2 } pfd_mount_mode_t;
pfd_quat_t pfd_mount_preset(pfd_mount_mode_t mode);
pfd_quat_t pfd_mount_from_pose(pfd_quat_t q_body);
pfd_quat_t pfd_mount_apply(pfd_quat_t q_body, pfd_quat_t mount);

/* Flat calibration (水平校准): the board lies on its back on a level table, so
 * the true specific force in the board body frame is exactly (-1, 0, 0)
 * (x = into the screen points down). Given the mean accelerometer reading in
 * the mapped sensor frame, return the sensor->body alignment C (minimal
 * rotation, v_body = rotate(C, v_sensor)) and the measured tilt in degrees.
 * Observable: misalignment about the screen's horizontal and vertical axes
 * (these are what make the upright horizon read wrong in pitch). A rotation
 * about the screen normal is not observable from this pose. */
pfd_quat_t pfd_flatcal_solve(pfd_vec3_t acc_mean, float *tilt_deg);

/* ---- Magnetometer calibration ---- */
enum { PFD_MAG_ROTATIONS = 24 };

typedef struct {
    float center[3];   /* hard-iron offset, sensor units */
    float W[9];        /* soft-iron matrix, row-major; |W (m - c)| ~= field */
    float field;       /* fitted field magnitude (sensor units) */
    float residual;    /* relative RMS deviation from the sphere (0.02 = 2%) */
    int8_t rotation;   /* index into the 24 sensor->body axis rotations */
    uint8_t ellipsoid; /* 1 = full ellipsoid fit, 0 = sphere fallback */
    float align_score; /* RMS spread of nav-frame field after alignment */
    float dip_deg;     /* inclination of the fitted field, + = down */
    bool valid;
} pfd_magcal_t;

/* Fit hard/soft-iron from raw samples. Returns false if ill-conditioned. */
bool pfd_magcal_fit(const pfd_vec3_t *raw, int n, pfd_magcal_t *out);
/* W (raw - center), still in sensor axes. */
pfd_vec3_t pfd_magcal_correct(const pfd_magcal_t *cal, pfd_vec3_t raw);
/* Correct and rotate into the body frame using cal->rotation. */
pfd_vec3_t pfd_magcal_to_body(const pfd_magcal_t *cal, pfd_vec3_t raw);
/* Choose the sensor->body rotation that keeps the nav-frame field most
 * constant given body->nav attitudes q[i] (6-DOF, recorded with the raw
 * samples). Writes rotation, align_score and dip_deg into cal. */
bool pfd_magcal_align(pfd_magcal_t *cal, const pfd_vec3_t *raw,
                      const pfd_quat_t *q, int n);
pfd_vec3_t pfd_axis_rotation_apply(int index, pfd_vec3_t v);
/* Coverage of sample directions around a provisional center, 0..26 bins. */
int pfd_magcal_coverage(const pfd_vec3_t *raw, int n);

#ifdef __cplusplus
}
#endif
