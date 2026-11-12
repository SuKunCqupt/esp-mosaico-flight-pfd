// SPDX-License-Identifier: MIT
/* Host tests for the attitude filter and magnetometer calibration.
 * Build: cc -O2 -I../../components/app_fusion/include test_fusion.c \
 *        ../../components/app_fusion/fusion.c ../../components/app_fusion/magcal.c -lm */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "pfd_fusion.h"

#define D2R 0.017453292f
#define R2D 57.29578f
static int s_fail;
#define CHECK(cond, ...) do { if (!(cond)) { printf("FAIL %s:%d ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); s_fail++; } } while (0)

static float frand(void) { return (float)rand() / RAND_MAX * 2.0f - 1.0f; }
static float gauss(void) { float s = 0; for (int i = 0; i < 6; ++i) s += frand(); return s / 1.414f; }
static float wrap180(float d) { while (d > 180) d -= 360; while (d < -180) d += 360; return d; }

static const pfd_vec3_t EARTH = {33.0f, 0.0f, 38.0f}; /* uT, NED, ~49 deg dip */

static pfd_vec3_t specific_force(pfd_quat_t q) { pfd_vec3_t up = {0, 0, -1}; return pfd_quat_rotate_inv(q, up); }

static void test_euler_conventions(void)
{
    pfd_quat_t q = pfd_quat_from_euler(30 * D2R, 0, 0);
    /* Right wing (body +y) must point down (nav +z) when rolled right. */
    pfd_vec3_t right = pfd_quat_rotate(q, (pfd_vec3_t){0, 1, 0});
    CHECK(right.z > 0.4f, "roll right should lower right wing (z=%f)", right.z);
    pfd_quat_t p = pfd_quat_from_euler(0, 20 * D2R, 0);
    pfd_vec3_t nose = pfd_quat_rotate(p, (pfd_vec3_t){1, 0, 0});
    CHECK(nose.z < -0.3f, "pitch up should raise nose (z=%f)", nose.z);
    float r, pt, y;
    pfd_quat_to_euler(pfd_quat_from_euler(-40 * D2R, 15 * D2R, 250 * D2R), &r, &pt, &y);
    CHECK(fabsf(r * R2D + 40) < 0.01f && fabsf(pt * R2D - 15) < 0.01f && fabsf(y * R2D - 250) < 0.01f,
          "euler roundtrip %f %f %f", r * R2D, pt * R2D, y * R2D);
}

/* Simulate a motion profile at rate hz; returns max attitude error (deg). */
static float run_profile(float hz, float seconds, bool use_mag, float bias_dps, float *heading_err, float *lag_ms)
{
    pfd_ahrs_t f;
    /* mag arrives every 8th step (25 Hz), gain is per mag sample */
    pfd_ahrs_init(&f, 1.0f, 0.02f, 4.0f);
    pfd_quat_t truth = pfd_quat_from_euler(0, 0, 40 * D2R);
    float dt = 1.0f / hz, worst = 0, worst_h = 0;
    pfd_vec3_t bias = {bias_dps * D2R, -bias_dps * 0.7f * D2R, bias_dps * 0.4f * D2R};
    int n = (int)(seconds * hz);
    for (int i = 0; i < n; ++i) {
        float t = i * dt;
        /* Rich hand-like motion: roll +-50, pitch +-25, yaw sweeps. */
        pfd_vec3_t w = {
            50 * D2R * 2.0f * cosf(2.0f * t),
            25 * D2R * 1.3f * cosf(1.3f * t + 1.0f),
            30 * D2R * 0.5f * cosf(0.5f * t),
        };
        if (t < 2.0f) w = (pfd_vec3_t){0, 0, 0}; /* settle */
        pfd_quat_t dq = pfd_quat_from_axis_angle(w, sqrtf(w.x * w.x + w.y * w.y + w.z * w.z) * dt);
        truth = pfd_quat_normalize(pfd_quat_mul(truth, dq));
        pfd_vec3_t g = {w.x + bias.x + gauss() * 0.002f, w.y + bias.y + gauss() * 0.002f, w.z + bias.z + gauss() * 0.002f};
        pfd_vec3_t a = specific_force(truth);
        a.x += gauss() * 0.01f; a.y += gauss() * 0.01f; a.z += gauss() * 0.01f;
        pfd_vec3_t m = pfd_quat_rotate_inv(truth, EARTH);
        m.x += gauss() * 0.4f; m.y += gauss() * 0.4f; m.z += gauss() * 0.4f;
        if (i == 0) {
            /* Start 20 deg off to exercise convergence. */
            pfd_ahrs_reset(&f, a, use_mag ? &m : NULL);
            f.q = pfd_quat_normalize(pfd_quat_mul(f.q, pfd_quat_from_euler(15 * D2R, -10 * D2R, 0)));
        }
        pfd_ahrs_update(&f, g, a, use_mag && (i % 8 == 0) ? &m : NULL, dt);
        if (t > 4.0f) {
            float r1, p1, y1, r2, p2, y2;
            pfd_quat_to_euler(truth, &r1, &p1, &y1);
            pfd_quat_to_euler(f.q, &r2, &p2, &y2);
            float e = fmaxf(fabsf(wrap180((r1 - r2) * R2D)), fabsf((p1 - p2) * R2D));
            if (e > worst) worst = e;
            float he = fabsf(wrap180((y1 - y2) * R2D));
            if (t > 30.0f && he > worst_h) worst_h = he;
        }
    }
    if (heading_err) *heading_err = worst_h;
    (void)lag_ms;
    return worst;
}

static void test_filter(void)
{
    float he;
    float e = run_profile(200, 60, true, 0.0f, &he, NULL);
    printf("filter 200Hz: max roll/pitch err %.2f deg, heading err %.2f deg\n", e, he);
    CHECK(e < 2.0f, "attitude error too big %f", e);
    CHECK(he < 3.0f, "heading error too big %f", he);
    e = run_profile(200, 120, true, 0.3f, &he, NULL);
    printf("filter 200Hz w/ 0.3dps residual bias: err %.2f deg, heading %.2f deg\n", e, he);
    CHECK(e < 3.0f, "attitude error with bias %f", e);
    CHECK(he < 5.0f, "heading error with bias %f", he);
}

static void test_static_level(void)
{
    pfd_ahrs_t f;
    pfd_ahrs_init(&f, 1.0f, 0.02f, 0.6f);
    pfd_vec3_t a = {0, 0, -1};
    for (int i = 0; i < 400; ++i) pfd_ahrs_update(&f, (pfd_vec3_t){0, 0, 0}, a, NULL, 0.005f);
    float r, p;
    pfd_quat_to_euler(f.q, &r, &p, NULL);
    CHECK(fabsf(r) < 0.001f && fabsf(p) < 0.001f, "static level %f %f", r, p);
    /* Board tilted 20 deg right wing down: gravity has +y component. */
    pfd_ahrs_t g;
    pfd_ahrs_init(&g, 1.0f, 0.0f, 0.0f);
    pfd_vec3_t a2 = specific_force(pfd_quat_from_euler(20 * D2R, 0, 0));
    pfd_ahrs_update(&g, (pfd_vec3_t){0, 0, 0}, (pfd_vec3_t){0, 0, -1}, NULL, 0.005f);
    for (int i = 0; i < 4000; ++i) pfd_ahrs_update(&g, (pfd_vec3_t){0, 0, 0}, a2, NULL, 0.005f);
    pfd_quat_to_euler(g.q, &r, &p, NULL);
    CHECK(fabsf(r * R2D - 20) < 0.2f, "accel convergence roll %f", r * R2D);
}

static void test_level_offset(void)
{
    pfd_quat_t q = pfd_quat_from_euler(12 * D2R, -7 * D2R, 100 * D2R);
    pfd_quat_t off = pfd_level_offset(q);
    float r, p, y;
    pfd_quat_to_euler(pfd_quat_mul(q, off), &r, &p, &y);
    CHECK(fabsf(r) < 1e-3f && fabsf(p) < 1e-3f, "level offset %f %f", r * R2D, p * R2D);
    /* Flat on a table: pitch 90 (nose up). Offset must remove it. */
    pfd_quat_t flat = pfd_quat_from_euler(0, 89.9f * D2R, 30 * D2R);
    pfd_quat_to_euler(pfd_quat_mul(flat, pfd_level_offset(flat)), &r, &p, &y);
    CHECK(fabsf(r) < 1e-2f && fabsf(p) < 1e-2f, "flat offset %f %f", r * R2D, p * R2D);
    /* After offset, rolling the device must still read as roll. */
    pfd_quat_t rolled = pfd_quat_mul(q, pfd_quat_from_axis_angle((pfd_vec3_t){0, 0, 1}, 0));
    (void)rolled;
}

static void test_magcal(void)
{
    /* Unknown sensor mounting: rotation index 13, hard iron, soft iron. */
    const int true_rot = 13;
    const float hard[3] = {180.0f, -95.0f, 410.0f};
    const float soft[9] = {1.10f, 0.05f, 0.02f, 0.05f, 0.93f, -0.03f, 0.02f, -0.03f, 1.02f};
    enum { N = 500 };
    static pfd_vec3_t raw[N];
    static pfd_quat_t q[N];
    srand(7);
    for (int i = 0; i < N; ++i) {
        /* Random orientations like a user waving the board. */
        q[i] = pfd_quat_from_euler(frand() * 3.1f, frand() * 1.4f, frand() * 3.1f);
        pfd_vec3_t mb = pfd_quat_rotate_inv(q[i], EARTH);
        /* body -> sensor: inverse of axis rotation (find by brute force). */
        pfd_vec3_t ms = mb;
        for (int k = 0; k < 1; ++k) {
            /* inverse of a signed permutation = apply rotation that maps back */
            pfd_vec3_t ex = pfd_axis_rotation_apply(true_rot, (pfd_vec3_t){1, 0, 0});
            pfd_vec3_t ey = pfd_axis_rotation_apply(true_rot, (pfd_vec3_t){0, 1, 0});
            pfd_vec3_t ez = pfd_axis_rotation_apply(true_rot, (pfd_vec3_t){0, 0, 1});
            ms = (pfd_vec3_t){ex.x * mb.x + ex.y * mb.y + ex.z * mb.z,
                              ey.x * mb.x + ey.y * mb.y + ey.z * mb.z,
                              ez.x * mb.x + ez.y * mb.y + ez.z * mb.z};
        }
        float d[3] = {ms.x, ms.y, ms.z};
        raw[i] = (pfd_vec3_t){
            soft[0] * d[0] + soft[1] * d[1] + soft[2] * d[2] + hard[0] + gauss() * 0.5f,
            soft[3] * d[0] + soft[4] * d[1] + soft[5] * d[2] + hard[1] + gauss() * 0.5f,
            soft[6] * d[0] + soft[7] * d[1] + soft[8] * d[2] + hard[2] + gauss() * 0.5f,
        };
        /* attitude estimate noise (6-DOF filter during calibration) */
        q[i] = pfd_quat_normalize(pfd_quat_mul(q[i], pfd_quat_from_euler(gauss() * 0.01f, gauss() * 0.01f, gauss() * 0.02f)));
    }
    int cov = pfd_magcal_coverage(raw, N);
    pfd_magcal_t cal;
    bool ok = pfd_magcal_fit(raw, N, &cal);
    CHECK(ok, "fit failed");
    ok = pfd_magcal_align(&cal, raw, q, N);
    printf("magcal: coverage %d/26 ellipsoid=%d center %.1f %.1f %.1f field %.1f resid %.3f rot %d (true %d) score %.3f dip %.1f\n",
           cov, cal.ellipsoid, cal.center[0], cal.center[1], cal.center[2], cal.field, cal.residual, cal.rotation,
           true_rot, cal.align_score, cal.dip_deg);
    CHECK(ok && cal.rotation == true_rot, "alignment picked %d", cal.rotation);
    CHECK(fabsf(cal.center[0] - hard[0]) < 3 && fabsf(cal.center[2] - hard[2]) < 3, "hard iron off");
    CHECK(cal.residual < 0.02f, "residual %f", cal.residual);
    /* Heading accuracy at several tilted poses. */
    float worst = 0;
    for (int i = 0; i < 50; ++i) {
        pfd_quat_t qt = pfd_quat_from_euler(frand() * 0.6f, frand() * 0.5f, (i * 7.2f) * D2R);
        pfd_vec3_t mb = pfd_quat_rotate_inv(qt, EARTH);
        pfd_vec3_t ex = pfd_axis_rotation_apply(true_rot, (pfd_vec3_t){1, 0, 0});
        pfd_vec3_t ey = pfd_axis_rotation_apply(true_rot, (pfd_vec3_t){0, 1, 0});
        pfd_vec3_t ez = pfd_axis_rotation_apply(true_rot, (pfd_vec3_t){0, 0, 1});
        float d[3] = {ex.x * mb.x + ex.y * mb.y + ex.z * mb.z, ey.x * mb.x + ey.y * mb.y + ey.z * mb.z,
                      ez.x * mb.x + ez.y * mb.y + ez.z * mb.z};
        pfd_vec3_t r = {soft[0] * d[0] + soft[1] * d[1] + soft[2] * d[2] + hard[0],
                        soft[3] * d[0] + soft[4] * d[1] + soft[5] * d[2] + hard[1],
                        soft[6] * d[0] + soft[7] * d[1] + soft[8] * d[2] + hard[2]};
        pfd_vec3_t m = pfd_magcal_to_body(&cal, r);
        pfd_ahrs_t f;
        pfd_ahrs_init(&f, 1, 0, 0.5f);
        pfd_ahrs_reset(&f, specific_force(qt), &m);
        float y1, y2;
        pfd_quat_to_euler(qt, NULL, NULL, &y1);
        pfd_quat_to_euler(f.q, NULL, NULL, &y2);
        float e = fabsf(wrap180((y1 - y2) * R2D));
        if (e > worst) worst = e;
    }
    /* True heading is relative to magnetic north of EARTH (atan2(-3,33)). */
    printf("magcal: worst heading error after cal (incl. %.1f deg declination of test field) %.2f\n",
           atan2f(EARTH.y, EARTH.x) * R2D, worst);
    CHECK(worst < 7.0f, "heading err %f", worst);

    /* Poor coverage (only yaw spin, level) must not fake a good fit. */
    for (int i = 0; i < N; ++i) {
        pfd_quat_t qt = pfd_quat_from_euler(0, 0, i * 0.05f);
        pfd_vec3_t mb = pfd_quat_rotate_inv(qt, EARTH);
        raw[i] = (pfd_vec3_t){mb.x + hard[0], mb.y + hard[1], mb.z + hard[2] + gauss() * 0.3f};
    }
    int cov2 = pfd_magcal_coverage(raw, N);
    printf("magcal: yaw-only coverage %d/26\n", cov2);
    CHECK(cov2 < 14, "yaw-only coverage should be low (%d)", cov2);
}


/* ---- v2.1 mounting reference ---- */
static pfd_quat_t rnav(float ax, float ay, float az, float deg)
{
    return pfd_quat_from_axis_angle((pfd_vec3_t){ax, ay, az}, deg * D2R);
}
/* Physical pose -> what the filter would converge to from accel + mag. */
static pfd_quat_t settle(pfd_quat_t q_true)
{
    pfd_ahrs_t f;
    pfd_ahrs_init(&f, 1.0f, 0.02f, 2.0f);
    pfd_vec3_t a = specific_force(q_true), m = pfd_quat_rotate_inv(q_true, EARTH);
    pfd_ahrs_reset(&f, a, &m);
    for (int i = 0; i < 400; ++i) pfd_ahrs_update(&f, (pfd_vec3_t){0, 0, 0}, a, &m, 0.005f);
    return f.q;
}
static void disp(pfd_quat_t qb, pfd_quat_t mount, float *r, float *p, float *h)
{
    pfd_quat_to_euler(pfd_mount_apply(qb, mount), r, p, h);
    *r *= R2D; *p *= R2D; *h *= R2D;
}

static void test_mount(void)
{
    /* Board lying on its back: body x (into screen) = down, top edge (-z) =
     * north, right edge (+y) = east. */
    const pfd_quat_t q0 = rnav(0, 1, 0, -90);
    const pfd_quat_t FLAT = pfd_mount_preset(PFD_MOUNT_FLAT), UP = pfd_mount_preset(PFD_MOUNT_UPRIGHT);
    float r, p, h, worst = 0;
    for (int hd = 0; hd < 360; hd += 30) {
        for (int k = 0; k < 5; ++k) {
            static const float tilt[5][2] = {{0, 0}, {10, 0}, {-10, 0}, {0, 15}, {0, -15}}; /* {nose up, right down} */
            float a = tilt[k][0], b = tilt[k][1];
            /* tilt about the board's own right / nose axes, then point the top edge at hd */
            pfd_quat_t q = pfd_quat_mul(rnav(0, 0, 1, (float)hd), pfd_quat_mul(rnav(0, 1, 0, a), pfd_quat_mul(rnav(1, 0, 0, b), q0)));
            disp(settle(q), FLAT, &r, &p, &h);
            float e = fmaxf(fmaxf(fabsf(p - a), fabsf(r - b)), fabsf(wrap180(h - hd)));
            if (e > worst) worst = e;
            CHECK(e < 0.5f, "flat hd=%d tilt(%g,%g): pitch=%.2f roll=%.2f hdg=%.2f", hd, a, b, p, r, h);
        }
    }
    printf("mount flat: worst error over 12 headings x 5 tilts = %.3f deg\n", worst);
    /* v1 problem: same flat pose through the upright reference sits at the pitch -90 singularity */
    disp(settle(q0), UP, &r, &p, &h);
    CHECK(p < -85.0f, "flat pose, upright mount should read pitch -90 (got %.1f)", p);
    /* Sign sanity in plain words: top edge raised -> pitch +, right edge lowered -> roll + */
    disp(settle(pfd_quat_mul(rnav(0, 1, 0, 7), q0)), FLAT, &r, &p, &h);
    CHECK(p > 6.5f && fabsf(r) < 0.3f, "top edge up 7: pitch %.2f roll %.2f", p, r);
    disp(settle(pfd_quat_mul(rnav(1, 0, 0, 7), q0)), FLAT, &r, &p, &h);
    CHECK(r > 6.5f && fabsf(p) < 0.3f, "right edge down 7: roll %.2f pitch %.2f", r, p);
    /* Upright preset unchanged: into-screen = nose */
    disp(settle(pfd_quat_mul(rnav(0, 0, 1, 120), rnav(0, 1, 0, 12))), UP, &r, &p, &h);
    CHECK(fabsf(p - 12) < 0.5f && fabsf(r) < 0.5f && fabsf(wrap180(h - 120)) < 0.5f, "upright: p=%.2f r=%.2f h=%.2f", p, r, h);
    /* Heading invariant to the mounting choice: flat top-edge-to-75 deg and
     * upright screen-facing-75 deg both read 75 */
    float h_flat, h_up;
    disp(settle(pfd_quat_mul(rnav(0, 0, 1, 75), q0)), FLAT, &r, &p, &h_flat);
    disp(settle(rnav(0, 0, 1, 75)), UP, &r, &p, &h_up);
    CHECK(fabsf(wrap180(h_flat - 75)) < 0.5f && fabsf(wrap180(h_up - 75)) < 0.5f, "heading flat %.2f upright %.2f", h_flat, h_up);

    /* 水平校准 on a slightly uneven desk (4 deg nose up, 3 deg left down, top edge at 40) */
    pfd_quat_t qd = pfd_quat_mul(rnav(0, 0, 1, 40), pfd_quat_mul(rnav(0, 1, 0, 4), pfd_quat_mul(rnav(1, 0, 0, -3), q0)));
    pfd_quat_t qb = settle(qd);
    pfd_quat_t M = pfd_mount_from_pose(qb);
    disp(qb, M, &r, &p, &h);
    CHECK(fabsf(p) < 0.05f && fabsf(r) < 0.05f && fabsf(wrap180(h - 40)) < 0.5f, "custom level: p=%.3f r=%.3f h=%.2f", p, r, h);
    /* then raise the top edge 10 deg about the board's right axis */
    float s = sinf(40 * D2R), cs = cosf(40 * D2R);
    pfd_quat_t q2 = pfd_quat_mul(rnav(-s, cs, 0, 10), qd);
    disp(settle(q2), M, &r, &p, &h);
    CHECK(fabsf(p - 10) < 0.7f && fabsf(r) < 0.7f && fabsf(wrap180(h - 40)) < 0.7f, "custom + nose up 10: p=%.2f r=%.2f h=%.2f", p, r, h);
    pfd_quat_t q3 = pfd_quat_mul(rnav(cs, s, 0, 10), qd); /* right edge down 10 */
    disp(settle(q3), M, &r, &p, &h);
    CHECK(fabsf(r - 10) < 0.7f && fabsf(p) < 0.7f && fabsf(wrap180(h - 40)) < 0.7f, "custom + right down 10: p=%.2f r=%.2f h=%.2f", p, r, h);
    /* level calibration done while upright keeps into-screen as the nose */
    pfd_quat_t qu = settle(pfd_quat_mul(rnav(0, 0, 1, 200), rnav(0, 1, 0, 8)));
    disp(qu, pfd_mount_from_pose(qu), &r, &p, &h);
    CHECK(fabsf(p) < 0.05f && fabsf(r) < 0.05f && fabsf(wrap180(h - 200)) < 0.5f, "upright custom: p=%.3f r=%.3f h=%.2f", p, r, h);
    /* no gimbal trouble anywhere near flat: sweep pitch -80..80 in the flat mount */
    for (int a = -80; a <= 80; a += 20) {
        disp(settle(pfd_quat_mul(rnav(0, 1, 0, (float)a), q0)), FLAT, &r, &p, &h);
        CHECK(fabsf(p - a) < 0.6f && fabsf(r) < 0.6f && fabsf(wrap180(h)) < 1.0f, "sweep %d: p=%.2f r=%.2f h=%.2f", a, p, r, h);
    }
}

/* ---- v2.2 flat calibration of IMU misalignment ---- */
/* Sensor axes rotated by E relative to the board: v_body = E v_sensor. */
static pfd_quat_t settle_meas(pfd_quat_t q_true, pfd_quat_t E, pfd_quat_t C)
{
    pfd_ahrs_t f;
    pfd_ahrs_init(&f, 1.0f, 0.02f, 2.0f);
    pfd_vec3_t a_s = pfd_quat_rotate_inv(E, specific_force(q_true));
    pfd_vec3_t m_s = pfd_quat_rotate_inv(E, pfd_quat_rotate_inv(q_true, EARTH));
    pfd_vec3_t a = pfd_quat_rotate(C, a_s), m = pfd_quat_rotate(C, m_s);
    pfd_ahrs_reset(&f, a, &m);
    for (int i = 0; i < 400; ++i) pfd_ahrs_update(&f, (pfd_vec3_t){0, 0, 0}, a, &m, 0.005f);
    return f.q;
}

static float upright_worst(pfd_quat_t E, pfd_quat_t C, float *worst_roll, float *worst_pitch)
{
    const pfd_quat_t UP = pfd_mount_preset(PFD_MOUNT_UPRIGHT);
    float worst = 0;
    *worst_roll = *worst_pitch = 0;
    static const float poses[][3] = {{0, 0, 0}, {0, 0, 90}, {20, 0, 200}, {-30, 5, 45}, {0, 15, 300},
                                     {0, -20, 120}, {45, -10, 10}, {-10, 30, 270}, {5, 60, 180}, {-60, -45, 330}};
    for (unsigned k = 0; k < sizeof(poses) / sizeof(poses[0]); ++k) {
        float rr = poses[k][0], pp = poses[k][1], hh = poses[k][2];
        pfd_quat_t q = pfd_quat_from_euler(rr * D2R, pp * D2R, hh * D2R);
        float r, p, h;
        disp(settle_meas(q, E, C), UP, &r, &p, &h);
        float er = fabsf(wrap180(r - rr)), ep = fabsf(p - pp), eh = fabsf(wrap180(h - hh));
        if (er > *worst_roll) *worst_roll = er;
        if (ep > *worst_pitch) *worst_pitch = ep;
        float e = fmaxf(fmaxf(er, ep), eh);
        if (e > worst) worst = e;
    }
    return worst;
}

static void test_flatcal(void)
{
    const pfd_quat_t I = {1, 0, 0, 0};
    const pfd_quat_t q_flat = rnav(0, 1, 0, -90); /* on its back, top edge north */
    /* IMU tilted 3 deg about the screen-horizontal axis and 2 deg about the screen-vertical axis */
    pfd_quat_t E = pfd_quat_mul(rnav(0, 1, 0, 3), rnav(0, 0, 1, 2));
    float wr, wp, worst;
    worst = upright_worst(E, I, &wr, &wp);
    printf("flatcal: uncorrected 3/2 deg IMU: worst upright error %.2f deg (roll %.2f pitch %.2f)\n", worst, wr, wp);
    CHECK(wp > 2.0f, "test must be sensitive (uncorrected pitch err %.2f)", wp);

    /* 2 s on the table: 400 noisy accel samples in the sensor frame */
    srand(7);
    pfd_vec3_t f_s = pfd_quat_rotate_inv(E, specific_force(q_flat)), sum = {0, 0, 0};
    for (int i = 0; i < 400; ++i) {
        sum.x += f_s.x + 0.003f * gauss();
        sum.y += f_s.y + 0.003f * gauss();
        sum.z += f_s.z + 0.003f * gauss();
    }
    pfd_vec3_t mean = {sum.x / 400, sum.y / 400, sum.z / 400};
    float tilt;
    pfd_quat_t C = pfd_flatcal_solve(mean, &tilt);
    CHECK(fabsf(tilt - 3.6f) < 0.2f, "measured tilt %.2f (expect ~3.6)", tilt);
    worst = upright_worst(E, C, &wr, &wp);
    printf("flatcal: corrected: worst upright roll/pitch/heading error %.3f deg (roll %.3f pitch %.3f), tilt %.2f\n",
           worst, wr, wp, tilt);
    CHECK(worst < 0.25f, "corrected upright error %.3f", worst);
    /* flat display mode sees the table as level after calibration */
    float r, p, h;
    disp(settle_meas(q_flat, E, C), pfd_mount_preset(PFD_MOUNT_FLAT), &r, &p, &h);
    CHECK(fabsf(r) < 0.1f && fabsf(p) < 0.1f, "flat after cal: r=%.2f p=%.2f", r, p);
    /* heading keeps its absolute (magnetic) reference: top edge north -> 0 */
    CHECK(fabsf(wrap180(h)) < 0.3f, "flat heading %.2f", h);
    /* rotation about the screen normal is not observable from the flat pose */
    pfd_quat_t En = rnav(1, 0, 0, 2);
    pfd_vec3_t fn = pfd_quat_rotate_inv(En, specific_force(q_flat));
    pfd_quat_t Cn = pfd_flatcal_solve(fn, &tilt);
    upright_worst(En, Cn, &wr, &wp);
    printf("flatcal: 2 deg about screen normal (unobservable flat): tilt %.2f, upright roll err %.2f, pitch err %.2f\n",
           tilt, wr, wp);
    CHECK(wp < 0.3f, "screen-normal misalignment must not leak into pitch (%.2f)", wp);
    /* not flat -> reject */
    pfd_vec3_t f20 = specific_force(pfd_quat_mul(rnav(0, 1, 0, 20), q_flat));
    pfd_flatcal_solve(f20, &tilt);
    CHECK(tilt > 15.0f && fabsf(tilt - 20) < 0.1f, "20 deg off flat measured %.2f", tilt);
    pfd_vec3_t fup = specific_force(I);
    pfd_flatcal_solve(fup, &tilt);
    CHECK(tilt > 85.0f, "upright board measured %.2f from flat", tilt);
}

int main(void)
{
    test_euler_conventions();
    test_static_level();
    test_level_offset();
    test_mount();
    test_flatcal();
    test_filter();
    test_magcal();
    printf(s_fail ? "%d FAILURES\n" : "ALL PASS\n", s_fail);
    return s_fail ? 1 : 0;
}
