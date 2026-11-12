// SPDX-License-Identifier: MIT
/* Magnetometer hard/soft-iron calibration and automatic sensor->body axis
 * alignment. The board has edge magnets for module docking, so offsets can
 * be large; they move with the board and are removed as hard iron. */
#include "pfd_fusion.h"
#include <math.h>
#include <string.h>

#define FIT_MIN_SAMPLES 40

static int8_t s_rot[PFD_MAG_ROTATIONS][6]; /* perm[3], sign[3] */
static bool s_rot_ready;

static void rotations_init(void)
{
    static const int perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
    static const int parity[6] = {1, -1, -1, 1, 1, -1};
    int n = 0;
    for (int p = 0; p < 6; ++p) {
        for (int s = 0; s < 8; ++s) {
            int sx = (s & 1) ? -1 : 1, sy = (s & 2) ? -1 : 1, sz = (s & 4) ? -1 : 1;
            if (parity[p] * sx * sy * sz != 1) continue;
            s_rot[n][0] = (int8_t)perms[p][0];
            s_rot[n][1] = (int8_t)perms[p][1];
            s_rot[n][2] = (int8_t)perms[p][2];
            s_rot[n][3] = (int8_t)sx;
            s_rot[n][4] = (int8_t)sy;
            s_rot[n][5] = (int8_t)sz;
            ++n;
        }
    }
    s_rot_ready = true;
}

pfd_vec3_t pfd_axis_rotation_apply(int index, pfd_vec3_t v)
{
    if (!s_rot_ready) rotations_init();
    if (index < 0 || index >= PFD_MAG_ROTATIONS) return v;
    const float in[3] = {v.x, v.y, v.z};
    const int8_t *r = s_rot[index];
    return (pfd_vec3_t){r[3] * in[r[0]], r[4] * in[r[1]], r[5] * in[r[2]]};
}

/* Solve A x = b (n <= 10) in place with partial pivoting. */
static bool solve(double *A, double *b, int n)
{
    for (int c = 0; c < n; ++c) {
        int piv = c;
        for (int r = c + 1; r < n; ++r)
            if (fabs(A[r * n + c]) > fabs(A[piv * n + c])) piv = r;
        if (fabs(A[piv * n + c]) < 1e-12) return false;
        if (piv != c) {
            for (int k = 0; k < n; ++k) {
                double t = A[c * n + k]; A[c * n + k] = A[piv * n + k]; A[piv * n + k] = t;
            }
            double t = b[c]; b[c] = b[piv]; b[piv] = t;
        }
        for (int r = 0; r < n; ++r) {
            if (r == c) continue;
            double f = A[r * n + c] / A[c * n + c];
            if (f == 0) continue;
            for (int k = c; k < n; ++k) A[r * n + k] -= f * A[c * n + k];
            b[r] -= f * b[c];
        }
    }
    for (int c = 0; c < n; ++c) b[c] /= A[c * n + c];
    return true;
}

/* Jacobi eigen-decomposition of a symmetric 3x3: M = V diag(e) V^T. */
static void eigen3(const double M[9], double V[9], double e[3])
{
    double a[9];
    memcpy(a, M, sizeof(a));
    for (int i = 0; i < 9; ++i) V[i] = (i % 4 == 0) ? 1.0 : 0.0;
    for (int sweep = 0; sweep < 50; ++sweep) {
        double off = fabs(a[1]) + fabs(a[2]) + fabs(a[5]);
        if (off < 1e-15) break;
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                double apq = a[p * 3 + q];
                if (fabs(apq) < 1e-18) continue;
                double theta = (a[q * 3 + q] - a[p * 3 + p]) / (2.0 * apq);
                double t = (theta >= 0 ? 1.0 : -1.0) / (fabs(theta) + sqrt(theta * theta + 1.0));
                double c = 1.0 / sqrt(t * t + 1.0), s = t * c;
                for (int k = 0; k < 3; ++k) {
                    double akp = a[k * 3 + p], akq = a[k * 3 + q];
                    a[k * 3 + p] = c * akp - s * akq;
                    a[k * 3 + q] = s * akp + c * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    double apk = a[p * 3 + k], aqk = a[q * 3 + k];
                    a[p * 3 + k] = c * apk - s * aqk;
                    a[q * 3 + k] = s * apk + c * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    double vkp = V[k * 3 + p], vkq = V[k * 3 + q];
                    V[k * 3 + p] = c * vkp - s * vkq;
                    V[k * 3 + q] = s * vkp + c * vkq;
                }
            }
        }
    }
    e[0] = a[0]; e[1] = a[4]; e[2] = a[8];
}

static void finish_stats(pfd_magcal_t *cal, const pfd_vec3_t *raw, int n)
{
    double sum = 0, sum2 = 0;
    for (int i = 0; i < n; ++i) {
        pfd_vec3_t m = pfd_magcal_correct(cal, raw[i]);
        double r = sqrt((double)m.x * m.x + (double)m.y * m.y + (double)m.z * m.z);
        sum += r;
        sum2 += r * r;
    }
    double mean = sum / n;
    double var = sum2 / n - mean * mean;
    cal->residual = mean > 0 ? (float)(sqrt(var > 0 ? var : 0) / mean) : 1.0f;
}

static bool sphere_fit(const pfd_vec3_t *raw, int n, const double mu[3], double sc, pfd_magcal_t *out)
{
    double A[16] = {0}, b[4] = {0};
    for (int i = 0; i < n; ++i) {
        double x = (raw[i].x - mu[0]) / sc, y = (raw[i].y - mu[1]) / sc, z = (raw[i].z - mu[2]) / sc;
        double row[4] = {2 * x, 2 * y, 2 * z, 1};
        double rhs = x * x + y * y + z * z;
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) A[r * 4 + c] += row[r] * row[c];
            b[r] += row[r] * rhs;
        }
    }
    if (!solve(A, b, 4)) return false;
    double r2 = b[3] + b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
    if (r2 <= 0) return false;
    double r = sqrt(r2) * sc;
    for (int k = 0; k < 3; ++k) out->center[k] = (float)(mu[k] + b[k] * sc);
    memset(out->W, 0, sizeof(out->W));
    out->W[0] = out->W[4] = out->W[8] = 1.0f;
    out->field = (float)r;
    out->ellipsoid = 0;
    return true;
}

bool pfd_magcal_fit(const pfd_vec3_t *raw, int n, pfd_magcal_t *out)
{
    memset(out, 0, sizeof(*out));
    out->rotation = 0;
    if (n < FIT_MIN_SAMPLES) return false;
    double mu[3] = {0}, sc = 0;
    for (int i = 0; i < n; ++i) {
        mu[0] += raw[i].x; mu[1] += raw[i].y; mu[2] += raw[i].z;
    }
    for (int k = 0; k < 3; ++k) mu[k] /= n;
    for (int i = 0; i < n; ++i) {
        double dx = raw[i].x - mu[0], dy = raw[i].y - mu[1], dz = raw[i].z - mu[2];
        sc += dx * dx + dy * dy + dz * dz;
    }
    sc = sqrt(sc / n);
    if (sc < 1e-6) return false;

    /* x^T A x + 2 v^T x = 1 on normalized samples. */
    double N[81] = {0}, rhs[9] = {0};
    for (int i = 0; i < n; ++i) {
        double x = (raw[i].x - mu[0]) / sc, y = (raw[i].y - mu[1]) / sc, z = (raw[i].z - mu[2]) / sc;
        double row[9] = {x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, 2 * x, 2 * y, 2 * z};
        for (int r = 0; r < 9; ++r) {
            for (int c = 0; c < 9; ++c) N[r * 9 + c] += row[r] * row[c];
            rhs[r] += row[r];
        }
    }
    bool ok = solve(N, rhs, 9);
    if (ok) {
        double A[9] = {rhs[0], rhs[3], rhs[4], rhs[3], rhs[1], rhs[5], rhs[4], rhs[5], rhs[2]};
        double v[3] = {rhs[6], rhs[7], rhs[8]};
        double Ac[9], c[3] = {-v[0], -v[1], -v[2]};
        memcpy(Ac, A, sizeof(Ac));
        ok = solve(Ac, c, 3);
        if (ok) {
            double cAc = 0;
            for (int r = 0; r < 3; ++r)
                for (int k = 0; k < 3; ++k) cAc += c[r] * A[r * 3 + k] * c[k];
            double s = 1.0 + cAc;
            double M[9], V[9], e[3];
            for (int k = 0; k < 9; ++k) M[k] = A[k] / s;
            eigen3(M, V, e);
            double emin = fmin(e[0], fmin(e[1], e[2])), emax = fmax(e[0], fmax(e[1], e[2]));
            /* Reject degenerate shells (poor coverage) or extreme soft iron. */
            if (s <= 0 || emin <= 0 || emax / emin > 4.0) {
                ok = false;
            } else {
                double gm = pow(e[0] * e[1] * e[2], 1.0 / 6.0); /* 1/radius (normalized) */
                double radius = sc / gm;
                double W[9];
                for (int r = 0; r < 3; ++r) {
                    for (int k = 0; k < 3; ++k) {
                        double acc = 0;
                        for (int j = 0; j < 3; ++j) acc += V[r * 3 + j] * sqrt(e[j]) * V[k * 3 + j];
                        W[r * 3 + k] = acc / gm; /* unit-radius sphere scaled back to field */
                    }
                }
                for (int k = 0; k < 3; ++k) out->center[k] = (float)(mu[k] + c[k] * sc);
                for (int k = 0; k < 9; ++k) out->W[k] = (float)W[k];
                out->field = (float)radius;
                out->ellipsoid = 1;
            }
        }
    }
    if (!ok && !sphere_fit(raw, n, mu, sc, out)) return false;
    finish_stats(out, raw, n);
    out->valid = out->residual < 0.25f;
    return out->valid;
}

pfd_vec3_t pfd_magcal_correct(const pfd_magcal_t *cal, pfd_vec3_t raw)
{
    float d[3] = {raw.x - cal->center[0], raw.y - cal->center[1], raw.z - cal->center[2]};
    const float *W = cal->W;
    return (pfd_vec3_t){
        W[0] * d[0] + W[1] * d[1] + W[2] * d[2],
        W[3] * d[0] + W[4] * d[1] + W[5] * d[2],
        W[6] * d[0] + W[7] * d[1] + W[8] * d[2],
    };
}

pfd_vec3_t pfd_magcal_to_body(const pfd_magcal_t *cal, pfd_vec3_t raw)
{
    return pfd_axis_rotation_apply(cal->rotation, pfd_magcal_correct(cal, raw));
}

bool pfd_magcal_align(pfd_magcal_t *cal, const pfd_vec3_t *raw, const pfd_quat_t *q, int n)
{
    if (n < FIT_MIN_SAMPLES || !cal->valid) return false;
    float best = 1e30f;
    int best_i = -1;
    pfd_vec3_t best_mean = {0, 0, 0};
    for (int r = 0; r < PFD_MAG_ROTATIONS; ++r) {
        double sx = 0, sy = 0, sz = 0, s2 = 0;
        for (int i = 0; i < n; ++i) {
            pfd_vec3_t m = pfd_axis_rotation_apply(r, pfd_magcal_correct(cal, raw[i]));
            pfd_vec3_t w = pfd_quat_rotate(q[i], m);
            sx += w.x; sy += w.y; sz += w.z;
            s2 += (double)w.x * w.x + (double)w.y * w.y + (double)w.z * w.z;
        }
        sx /= n; sy /= n; sz /= n;
        double var = s2 / n - (sx * sx + sy * sy + sz * sz);
        float score = (float)(sqrt(var > 0 ? var : 0) / (cal->field > 0 ? cal->field : 1));
        if (score < best) {
            best = score;
            best_i = r;
            best_mean = (pfd_vec3_t){(float)sx, (float)sy, (float)sz};
        }
    }
    if (best_i < 0) return false;
    cal->rotation = (int8_t)best_i;
    cal->align_score = best;
    float mn = sqrtf(best_mean.x * best_mean.x + best_mean.y * best_mean.y + best_mean.z * best_mean.z);
    cal->dip_deg = mn > 0 ? asinf(best_mean.z / mn) * 57.29578f : 0;
    return true;
}

int pfd_magcal_coverage(const pfd_vec3_t *raw, int n)
{
    if (n <= 0) return 0;
    float lo[3] = {raw[0].x, raw[0].y, raw[0].z}, hi[3] = {raw[0].x, raw[0].y, raw[0].z};
    for (int i = 1; i < n; ++i) {
        const float v[3] = {raw[i].x, raw[i].y, raw[i].z};
        for (int k = 0; k < 3; ++k) {
            if (v[k] < lo[k]) lo[k] = v[k];
            if (v[k] > hi[k]) hi[k] = v[k];
        }
    }
    float c[3] = {(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2};
    uint32_t bins = 0;
    for (int i = 0; i < n; ++i) {
        float d[3] = {raw[i].x - c[0], raw[i].y - c[1], raw[i].z - c[2]};
        float l = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        if (l < 1e-6f) continue;
        int code = 0;
        for (int k = 0; k < 3; ++k) {
            float u = d[k] / l;
            int t = u > 0.38f ? 2 : u < -0.38f ? 0 : 1;
            code = code * 3 + t;
        }
        if (code != 13) bins |= 1u << code;
    }
    int count = 0;
    for (int i = 0; i < 27; ++i) count += (bins >> i) & 1u;
    return count;
}
