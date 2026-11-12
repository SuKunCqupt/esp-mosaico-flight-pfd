// SPDX-License-Identifier: MIT
#include "include/pfd_gps.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define R_EARTH 6371008.8
#define DEG2RAD 0.017453292519943295

/* Find "key" followed by ':' and return a pointer to the value (or NULL). */
static const char *find_value(const char *b, size_t len, const char *key)
{
    size_t kl = strlen(key);
    for (size_t i = 0; i + kl + 2 < len; ++i) {
        if (b[i] != '"' || memcmp(b + i + 1, key, kl) != 0 || b[i + 1 + kl] != '"') continue;
        size_t j = i + kl + 2;
        while (j < len && (b[j] == ' ' || b[j] == '\t' || b[j] == '\r' || b[j] == '\n')) ++j;
        if (j >= len || b[j] != ':') continue;
        ++j;
        while (j < len && (b[j] == ' ' || b[j] == '\t' || b[j] == '\r' || b[j] == '\n')) ++j;
        return j < len ? b + j : NULL;
    }
    return NULL;
}

bool pfd_json_number(const char *b, size_t len, const char *key, double *v, bool *is_null)
{
    const char *p = find_value(b, len, key);
    if (is_null) *is_null = false;
    if (!p) return false;
    size_t rem = len - (size_t)(p - b);
    if (rem >= 4 && memcmp(p, "null", 4) == 0) {
        if (is_null) *is_null = true;
        return true;
    }
    char tmp[40];
    size_t n = 0;
    while (n < rem && n < sizeof(tmp) - 1 && ((p[n] && strchr("+-.eE", p[n])) || (p[n] >= '0' && p[n] <= '9'))) {
        tmp[n] = p[n];
        ++n;
    }
    tmp[n] = 0;
    if (!n) {
        if (is_null) *is_null = true;
        return true;
    }
    char *end = NULL;
    double d = strtod(tmp, &end);
    if (end == tmp || !isfinite(d)) {
        if (is_null) *is_null = true;
        return true;
    }
    *v = d;
    return true;
}

bool pfd_json_bool(const char *b, size_t len, const char *key, bool *v)
{
    const char *p = find_value(b, len, key);
    if (!p) return false;
    size_t rem = len - (size_t)(p - b);
    if (rem >= 4 && memcmp(p, "true", 4) == 0) { *v = true; return true; }
    if (rem >= 5 && memcmp(p, "false", 5) == 0) { *v = false; return true; }
    return false;
}

static size_t put_utf8(char *o, size_t cap, size_t n, unsigned cp)
{
    char t[4];
    size_t k;
    if (cp < 0x80) { t[0] = (char)cp; k = 1; }
    else if (cp < 0x800) { t[0] = (char)(0xC0 | (cp >> 6)); t[1] = (char)(0x80 | (cp & 0x3F)); k = 2; }
    else { t[0] = (char)(0xE0 | (cp >> 12)); t[1] = (char)(0x80 | ((cp >> 6) & 0x3F)); t[2] = (char)(0x80 | (cp & 0x3F)); k = 3; }
    if (n + k >= cap) return n;
    memcpy(o + n, t, k);
    return n + k;
}

bool pfd_json_string(const char *b, size_t len, const char *key, char *out, size_t cap)
{
    const char *p = find_value(b, len, key);
    if (!p || *p != '"' || !cap) return false;
    const char *e = b + len;
    size_t n = 0;
    for (++p; p < e && *p != '"'; ++p) {
        char c = *p;
        if (c == '\\' && p + 1 < e) {
            c = *++p;
            switch (c) {
            case 'n': c = '\n'; break;
            case 't': c = '\t'; break;
            case 'r': c = '\r'; break;
            case 'b': c = '\b'; break;
            case 'f': c = '\f'; break;
            case 'u':
                if (p + 4 < e) {
                    char hex[5] = {p[1], p[2], p[3], p[4], 0};
                    n = put_utf8(out, cap, n, (unsigned)strtoul(hex, NULL, 16));
                    p += 4;
                }
                continue;
            default: break; /* \" \\ \/ */
            }
        }
        if (n + 1 < cap) out[n++] = c;
    }
    out[n] = 0;
    return p < e;
}

double pfd_geo_distance_m(double lat1, double lon1, double lat2, double lon2)
{
    double p1 = lat1 * DEG2RAD, p2 = lat2 * DEG2RAD;
    double dp = p2 - p1, dl = (lon2 - lon1) * DEG2RAD;
    double a = sin(dp / 2) * sin(dp / 2) + cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
    return 2 * R_EARTH * atan2(sqrt(a), sqrt(1 - a));
}

double pfd_geo_bearing_deg(double lat1, double lon1, double lat2, double lon2)
{
    double p1 = lat1 * DEG2RAD, p2 = lat2 * DEG2RAD, dl = (lon2 - lon1) * DEG2RAD;
    double y = sin(dl) * cos(p2), x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
    double b = atan2(y, x) / DEG2RAD;
    return b < 0 ? b + 360 : b;
}

bool pfd_gps_parse(const char *b, size_t len, pfd_gps_msg_t *m)
{
    memset(m, 0, sizeof(*m));
    bool nul;
    double lat = 0, lon = 0, v = 0;
    m->acc = m->aacc = -1;
    if (pfd_json_number(b, len, "lat", &lat, &nul) && !nul && pfd_json_number(b, len, "lon", &lon, &nul) && !nul &&
        fabs(lat) <= 90 && fabs(lon) <= 180) {
        m->has_pos = true;
        m->lat = lat;
        m->lon = lon;
    }
    if (pfd_json_number(b, len, "alt", &v, &nul) && !nul && fabs(v) < 20000) { m->alt_valid = true; m->alt = v; }
    /* iOS uses -1 for "unknown" speed / course, Chrome uses null / NaN */
    if (pfd_json_number(b, len, "spd", &v, &nul) && !nul && v >= 0 && v < 400) { m->spd_valid = true; m->spd = v; }
    if (pfd_json_number(b, len, "hdg", &v, &nul) && !nul && v >= 0 && v <= 360) { m->hdg_valid = true; m->hdg = v; }
    if (pfd_json_number(b, len, "acc", &v, &nul) && !nul && v >= 0) m->acc = v;
    if (pfd_json_number(b, len, "aacc", &v, &nul) && !nul && v >= 0) m->aacc = v;
    if (pfd_json_number(b, len, "t", &v, &nul) && !nul) m->t_ms = v;
    if (pfd_json_number(b, len, "age", &v, &nul) && !nul && v >= 0) m->age_ms = v;
    /* a body with none of the fields is not ours */
    return m->has_pos || find_value(b, len, "lat") != NULL || find_value(b, len, "hb") != NULL;
}

void pfd_gps_apply(pfd_gps_track_t *t, const pfd_gps_msg_t *m, double now_s)
{
    t->packets++;
    t->last_rx_s = now_s;
    if (!m->has_pos) return;
    bool new_fix = !t->has_fix || m->t_ms != t->fix.t_ms;
    t->has_fix = true;
    t->fix = *m;
    t->fix_rx_s = now_s;
    if (!new_fix) return;
    if (t->prev_valid) {
        double dt = (m->t_ms - t->prev_t_ms) / 1000.0;
        if (dt > 0.3 && dt < 15.0) {
            double d = pfd_geo_distance_m(t->prev_lat, t->prev_lon, m->lat, m->lon);
            double v = d / dt;
            /* ignore jumps that are within the reported position noise */
            double noise = m->acc > 0 ? m->acc : 10.0;
            if (d < 0.25 * noise && v < 1.0) v = 0;
            if (v < 400) {
                t->dspd = t->dspd_valid ? 0.5 * t->dspd + 0.5 * v : v;
                t->dspd_valid = true;
            }
            if (d > 3.0 && d > 0.5 * noise) {
                t->dtrk = pfd_geo_bearing_deg(t->prev_lat, t->prev_lon, m->lat, m->lon);
                t->dtrk_valid = true;
            }
        } else if (dt >= 15.0) {
            t->dspd_valid = t->dtrk_valid = false;
        }
    }
    t->prev_valid = true;
    t->prev_lat = m->lat;
    t->prev_lon = m->lon;
    t->prev_t_ms = m->t_ms;
}

void pfd_gps_fill(const pfd_gps_track_t *t, double now_s, pfd_link_snapshot_t *o)
{
    o->packets = t->packets;
    o->since_s = t->packets ? (float)(now_s - t->last_rx_s) : 0;
    o->state = !t->packets ? PFD_LINK_NONE : o->since_s <= PFD_LINK_TIMEOUT_S ? PFD_LINK_ACTIVE : PFD_LINK_LOST;
    o->has_fix = t->has_fix;
    if (!t->has_fix) return;
    const pfd_gps_msg_t *f = &t->fix;
    o->lat = f->lat;
    o->lon = f->lon;
    o->alt_valid = f->alt_valid;
    o->alt_m = (float)f->alt;
    o->acc_m = (float)f->acc;
    o->alt_acc_m = (float)f->aacc;
    o->fix_age_s = (float)(f->age_ms / 1000.0 + (now_s - t->fix_rx_s));
    if (f->spd_valid) {
        o->speed_valid = true;
        o->speed_mps = (float)f->spd;
        o->speed_derived = false;
    } else if (t->dspd_valid) {
        o->speed_valid = true;
        o->speed_mps = (float)t->dspd;
        o->speed_derived = true;
    }
    if (f->hdg_valid && (!f->spd_valid || f->spd > 0.5)) {
        o->track_valid = true;
        o->track_deg = (float)f->hdg;
    } else if (t->dtrk_valid) {
        o->track_valid = true;
        o->track_deg = (float)t->dtrk;
    }
    /* stale fix (phone stopped producing positions) → don't claim live data */
    if (o->fix_age_s > 10.0f) {
        o->speed_valid = false;
        o->track_valid = false;
    }
}
