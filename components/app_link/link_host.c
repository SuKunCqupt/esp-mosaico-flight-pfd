// SPDX-License-Identifier: MIT
/* Host simulator stand-in for the phone link. PFD_HOST_GPS=1 feeds a
 * synthetic phone (1 Hz fixes, speed null every other fix to exercise the
 * derived-speed path); PFD_HOST_GPS=lost stops after 5 s. */
#include "include/pfd_gps.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern double pfd_host_now_s(void);
static pfd_gps_track_t s_track;
static int s_mode; /* 0 none, 1 live, 2 lost after 5 s */
static double s_next;
static int s_n;

int pfd_link_start(void)
{
    const char *e = getenv("PFD_HOST_GPS");
    s_mode = !e ? 0 : strcmp(e, "lost") == 0 ? 2 : 1;
    return 0;
}

void pfd_link_read(pfd_link_snapshot_t *out)
{
    double now = pfd_host_now_s();
    memset(out, 0, sizeof(*out));
    out->wifi_up = out->server_up = true;
    snprintf(out->ap_ssid, sizeof(out->ap_ssid), "飞行表-1A2B");
    snprintf(out->ap_pass, sizeof(out->ap_pass), "12345678");
    snprintf(out->ap_url, sizeof(out->ap_url), "https://192.168.4.1");
    out->ap_clients = s_mode ? 1 : 0;
    while (s_mode && now >= s_next && !(s_mode == 2 && now > 5.0)) {
        char body[256];
        double t = s_next;
        double lat = 31.2304 + 0.00030 * t, lon = 121.4737 + 0.00020 * t;
        if (s_n % 2)
            snprintf(body, sizeof(body), "{\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f,\"spd\":null,\"hdg\":null,\"acc\":6,"
                     "\"aacc\":9,\"t\":%.0f,\"age\":120}", lat, lon, 35.0 + t, t * 1000);
        else
            snprintf(body, sizeof(body), "{\"lat\":%.7f,\"lon\":%.7f,\"alt\":%.1f,\"spd\":%.2f,\"hdg\":30,\"acc\":5,"
                     "\"aacc\":8,\"t\":%.0f,\"age\":80}", lat, lon, 35.0 + t, 39.5, t * 1000);
        pfd_gps_msg_t m;
        if (pfd_gps_parse(body, strlen(body), &m)) pfd_gps_apply(&s_track, &m, s_next);
        s_n++;
        s_next += 1.0;
    }
    pfd_gps_fill(&s_track, now, out);
}
