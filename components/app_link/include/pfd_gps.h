// SPDX-License-Identifier: MIT
#pragma once
/* Platform-independent part of the phone link: message parsing and fix
 * bookkeeping (derived speed / track when the browser reports none). */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "pfd_link.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool has_pos;
    double lat, lon;
    bool alt_valid; double alt;
    bool spd_valid; double spd;      /* m/s */
    bool hdg_valid; double hdg;      /* deg */
    double acc, aacc;                /* < 0 unknown */
    double t_ms;                     /* phone fix timestamp */
    double age_ms;                   /* phone-side age when sent */
} pfd_gps_msg_t;

typedef struct {
    uint32_t packets;
    double last_rx_s;
    bool has_fix;
    pfd_gps_msg_t fix;
    double fix_rx_s;
    /* derived from successive positions */
    bool prev_valid;
    double prev_lat, prev_lon, prev_t_ms;
    bool dspd_valid; double dspd;
    bool dtrk_valid; double dtrk;
} pfd_gps_track_t;

/* Parse {"lat":..,"lon":..,"alt":..|null,"spd":..,"hdg":..,"acc":..,"aacc":..,"t":..,"age":..}. */
bool pfd_gps_parse(const char *body, size_t len, pfd_gps_msg_t *m);
void pfd_gps_apply(pfd_gps_track_t *t, const pfd_gps_msg_t *m, double now_s);
void pfd_gps_fill(const pfd_gps_track_t *t, double now_s, pfd_link_snapshot_t *out);

/* JSON helpers (flat objects only). Return false if the key is absent. */
bool pfd_json_number(const char *body, size_t len, const char *key, double *v, bool *is_null);
bool pfd_json_string(const char *body, size_t len, const char *key, char *out, size_t cap);
bool pfd_json_bool(const char *body, size_t len, const char *key, bool *v);

double pfd_geo_distance_m(double lat1, double lon1, double lat2, double lon2);
double pfd_geo_bearing_deg(double lat1, double lon1, double lat2, double lon2);

#ifdef __cplusplus
}
#endif
