// SPDX-License-Identifier: MIT
/* Mahony complementary filter on quaternions. Gyro integration carries all
 * motion with no added delay; accelerometer and magnetometer only bleed out
 * drift, so the horizon never lags the hand. Accel trust is reduced while
 * the board is being shaken (|a| far from 1 g). */
#include "pfd_fusion.h"
#include <math.h>

#define ACC_TRUST_START 0.06f  /* g deviation where trust begins to fall */
#define ACC_TRUST_SPAN 0.30f   /* g deviation span to zero trust */
#define INTEGRAL_LIMIT 0.05f   /* rad/s, clamp on learned bias */

static pfd_vec3_t v3(float x, float y, float z) { return (pfd_vec3_t){x, y, z}; }
static pfd_vec3_t vcross(pfd_vec3_t a, pfd_vec3_t b)
{
    return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
static float vdot(pfd_vec3_t a, pfd_vec3_t b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static float vnorm(pfd_vec3_t a) { return sqrtf(vdot(a, a)); }
static pfd_vec3_t vscale(pfd_vec3_t a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static pfd_vec3_t vadd(pfd_vec3_t a, pfd_vec3_t b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

pfd_quat_t pfd_quat_mul(pfd_quat_t a, pfd_quat_t b)
{
    return (pfd_quat_t){
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

pfd_quat_t pfd_quat_conj(pfd_quat_t q) { return (pfd_quat_t){q.w, -q.x, -q.y, -q.z}; }

pfd_quat_t pfd_quat_normalize(pfd_quat_t q)
{
    float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (n < 1e-9f) return (pfd_quat_t){1, 0, 0, 0};
    float s = 1.0f / n;
    if (q.w < 0) s = -s; /* keep a canonical hemisphere */
    return (pfd_quat_t){q.w * s, q.x * s, q.y * s, q.z * s};
}

pfd_vec3_t pfd_quat_rotate(pfd_quat_t q, pfd_vec3_t v)
{
    pfd_vec3_t u = v3(q.x, q.y, q.z);
    pfd_vec3_t t = vscale(vcross(u, v), 2.0f);
    return vadd(vadd(v, vscale(t, q.w)), vcross(u, t));
}

pfd_vec3_t pfd_quat_rotate_inv(pfd_quat_t q, pfd_vec3_t v)
{
    return pfd_quat_rotate(pfd_quat_conj(q), v);
}

pfd_quat_t pfd_quat_from_axis_angle(pfd_vec3_t axis, float angle)
{
    float n = vnorm(axis);
    if (n < 1e-9f) return (pfd_quat_t){1, 0, 0, 0};
    float s = sinf(angle * 0.5f) / n;
    return (pfd_quat_t){cosf(angle * 0.5f), axis.x * s, axis.y * s, axis.z * s};
}

void pfd_quat_to_euler(pfd_quat_t q, float *roll, float *pitch, float *yaw)
{
    float sp = 2.0f * (q.w * q.y - q.z * q.x);
    if (roll) *roll = atan2f(2.0f * (q.w * q.x + q.y * q.z), 1.0f - 2.0f * (q.x * q.x + q.y * q.y));
    if (pitch) *pitch = asinf(clampf(sp, -1.0f, 1.0f));
    if (yaw) {
        float y = atan2f(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z));
        if (y < 0) y += 2.0f * (float)M_PI;
        *yaw = y;
    }
}

pfd_quat_t pfd_quat_from_euler(float roll, float pitch, float yaw)
{
    pfd_quat_t qz = pfd_quat_from_axis_angle(v3(0, 0, 1), yaw);
    pfd_quat_t qy = pfd_quat_from_axis_angle(v3(0, 1, 0), pitch);
    pfd_quat_t qx = pfd_quat_from_axis_angle(v3(1, 0, 0), roll);
    return pfd_quat_normalize(pfd_quat_mul(pfd_quat_mul(qz, qy), qx));
}

static pfd_quat_t quat_from_rows(pfd_vec3_t r0, pfd_vec3_t r1, pfd_vec3_t r2)
{
    /* Rotation matrix rows r0..r2 (nav axes expressed in body). */
    float m00 = r0.x, m01 = r0.y, m02 = r0.z;
    float m10 = r1.x, m11 = r1.y, m12 = r1.z;
    float m20 = r2.x, m21 = r2.y, m22 = r2.z;
    float tr = m00 + m11 + m22;
    pfd_quat_t q;
    if (tr > 0) {
        float s = sqrtf(tr + 1.0f) * 2.0f;
        q = (pfd_quat_t){0.25f * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s};
    } else if (m00 > m11 && m00 > m22) {
        float s = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
        q = (pfd_quat_t){(m21 - m12) / s, 0.25f * s, (m01 + m10) / s, (m02 + m20) / s};
    } else if (m11 > m22) {
        float s = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
        q = (pfd_quat_t){(m02 - m20) / s, (m01 + m10) / s, 0.25f * s, (m12 + m21) / s};
    } else {
        float s = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
        q = (pfd_quat_t){(m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25f * s};
    }
    return pfd_quat_normalize(q);
}

void pfd_ahrs_init(pfd_ahrs_t *f, float kp_acc, float ki_acc, float kp_mag)
{
    *f = (pfd_ahrs_t){
        .q = {1, 0, 0, 0}, .kp_acc = kp_acc, .ki_acc = ki_acc, .kp_mag = kp_mag,
    };
}

void pfd_ahrs_reset(pfd_ahrs_t *f, pfd_vec3_t acc, const pfd_vec3_t *mag)
{
    float an = vnorm(acc);
    if (an < 1e-3f) return;
    pfd_vec3_t down = vscale(acc, -1.0f / an);
    pfd_vec3_t east = v3(0, 0, 0);
    if (mag) east = vcross(down, *mag);
    if (vnorm(east) < 1e-3f) {
        /* No usable mag: keep the body x axis as "north". */
        pfd_vec3_t fwd = v3(1, 0, 0);
        if (fabsf(vdot(fwd, down)) > 0.95f) fwd = v3(0, 0, -1);
        pfd_vec3_t north = vadd(fwd, vscale(down, -vdot(fwd, down)));
        north = vscale(north, 1.0f / vnorm(north));
        east = vcross(down, north);
    }
    east = vscale(east, 1.0f / vnorm(east));
    pfd_vec3_t north = vcross(east, down);
    f->q = quat_from_rows(north, east, down);
    f->integral = v3(0, 0, 0);
    f->initialized = true;
}

void pfd_ahrs_update(pfd_ahrs_t *f, pfd_vec3_t gyro, pfd_vec3_t acc,
                     const pfd_vec3_t *mag, float dt)
{
    if (!f->initialized) {
        pfd_ahrs_reset(f, acc, mag);
        return;
    }
    pfd_quat_t q = f->q;
    pfd_vec3_t down_b = pfd_quat_rotate_inv(q, v3(0, 0, 1));
    pfd_vec3_t corr = v3(0, 0, 0);

    float an = vnorm(acc);
    float trust = 0.0f;
    if (an > 0.2f) {
        trust = 1.0f - (fabsf(an - 1.0f) - ACC_TRUST_START) / ACC_TRUST_SPAN;
        trust = clampf(trust, 0.0f, 1.0f);
    }
    f->acc_weight = trust;
    if (trust > 0) {
        pfd_vec3_t a = vscale(acc, 1.0f / an);
        pfd_vec3_t up_est = vscale(down_b, -1.0f);
        pfd_vec3_t e = vcross(a, up_est);
        corr = vadd(corr, vscale(e, f->kp_acc * trust));
        if (f->ki_acc > 0) {
            f->integral = vadd(f->integral, vscale(e, f->ki_acc * trust * dt));
            f->integral.x = clampf(f->integral.x, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            f->integral.y = clampf(f->integral.y, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
            f->integral.z = clampf(f->integral.z, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        }
    }
    if (mag && f->kp_mag > 0) {
        float mn = vnorm(*mag);
        if (mn > 1e-6f) {
            pfd_vec3_t m = vscale(*mag, 1.0f / mn);
            pfd_vec3_t h = pfd_quat_rotate(q, m);
            pfd_vec3_t b = v3(sqrtf(h.x * h.x + h.y * h.y), 0, h.z);
            pfd_vec3_t w = pfd_quat_rotate_inv(q, b);
            pfd_vec3_t e = vcross(m, w);
            /* Heading only: drop components that would tilt the horizon. */
            e = vscale(down_b, vdot(e, down_b));
            corr = vadd(corr, vscale(e, f->kp_mag));
        }
    }
    pfd_vec3_t w = vadd(vadd(gyro, corr), f->integral);
    pfd_quat_t dq = {0, w.x, w.y, w.z};
    pfd_quat_t qd = pfd_quat_mul(q, dq);
    q.w += 0.5f * qd.w * dt;
    q.x += 0.5f * qd.x * dt;
    q.y += 0.5f * qd.y * dt;
    q.z += 0.5f * qd.z * dt;
    f->q = pfd_quat_normalize(q);
}

pfd_quat_t pfd_level_offset(pfd_quat_t q_current)
{
    pfd_vec3_t d = pfd_quat_rotate_inv(q_current, v3(0, 0, 1));
    pfd_vec3_t ez = v3(0, 0, 1);
    pfd_vec3_t axis = vcross(ez, d);
    float c = clampf(vdot(ez, d), -1.0f, 1.0f);
    if (vnorm(axis) < 1e-6f) {
        if (c > 0) return (pfd_quat_t){1, 0, 0, 0};
        return pfd_quat_from_axis_angle(v3(1, 0, 0), (float)M_PI);
    }
    return pfd_quat_normalize(pfd_quat_from_axis_angle(axis, acosf(c)));
}

pfd_quat_t pfd_mount_preset(pfd_mount_mode_t mode)
{
    if (mode == PFD_MOUNT_FLAT) return pfd_quat_from_axis_angle(v3(0, 1, 0), (float)M_PI_2);
    return (pfd_quat_t){1, 0, 0, 0};
}

pfd_quat_t pfd_mount_apply(pfd_quat_t q_body, pfd_quat_t mount)
{
    return pfd_quat_normalize(pfd_quat_mul(q_body, mount));
}

pfd_quat_t pfd_mount_from_pose(pfd_quat_t q_body)
{
    /* Pick the preset whose "down" (display z) is closest to gravity so the
     * nose keeps its meaning (top edge when lying flat, into the screen when
     * upright), then remove the remaining tilt with the minimal rotation:
     * no rotation about the vertical, so heading is preserved. */
    pfd_vec3_t d = pfd_quat_rotate_inv(q_body, v3(0, 0, 1));
    pfd_quat_t p = pfd_mount_preset(d.x > d.z ? PFD_MOUNT_FLAT : PFD_MOUNT_UPRIGHT);
    pfd_quat_t fix = pfd_level_offset(pfd_quat_mul(q_body, p));
    return pfd_quat_normalize(pfd_quat_mul(p, fix));
}

pfd_quat_t pfd_flatcal_solve(pfd_vec3_t acc_mean, float *tilt_deg)
{
    float n = vnorm(acc_mean);
    if (n < 1e-3f) {
        if (tilt_deg) *tilt_deg = 180.0f;
        return (pfd_quat_t){1, 0, 0, 0};
    }
    pfd_vec3_t u = vscale(acc_mean, 1.0f / n), v = v3(-1, 0, 0);
    float c = clampf(vdot(u, v), -1.0f, 1.0f);
    if (tilt_deg) *tilt_deg = acosf(c) * 57.29578f;
    pfd_vec3_t axis = vcross(u, v);
    if (vnorm(axis) < 1e-7f) return (pfd_quat_t){1, 0, 0, 0};
    return pfd_quat_normalize(pfd_quat_from_axis_angle(axis, acosf(c)));
}
