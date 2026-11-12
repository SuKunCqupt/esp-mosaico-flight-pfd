// SPDX-License-Identifier: MIT
/* Phone-link parsing and derived speed / track. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pfd_gps.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL %s:%d ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

int main(void)
{
    pfd_gps_msg_t m;
    const char *a = "{\"hb\":1,\"t\":1700000000123,\"age\":40,\"lat\":31.2304612,\"lon\":121.4737021,\"alt\":null,"
                    "\"spd\":null,\"hdg\":null,\"acc\":4.8,\"aacc\":null}";
    CHECK(pfd_gps_parse(a, strlen(a), &m), "parse a");
    CHECK(m.has_pos && fabs(m.lat - 31.2304612) < 1e-9 && fabs(m.lon - 121.4737021) < 1e-9, "pos");
    CHECK(!m.alt_valid && !m.spd_valid && !m.hdg_valid && m.acc == 4.8 && m.aacc < 0, "nulls");
    const char *ios = "{\"lat\":1,\"lon\":2,\"alt\":12.5,\"spd\":-1,\"hdg\":-1,\"acc\":5,\"aacc\":3,\"t\":1000,\"age\":0}";
    CHECK(pfd_gps_parse(ios, strlen(ios), &m) && m.alt_valid && m.alt == 12.5 && !m.spd_valid && !m.hdg_valid, "ios -1");
    const char *hb = "{\"hb\":1}";
    CHECK(pfd_gps_parse(hb, strlen(hb), &m) && !m.has_pos, "heartbeat");
    const char *junk = "{\"foo\":1}";
    CHECK(!pfd_gps_parse(junk, strlen(junk), &m), "junk rejected");
    char s[40];
    const char *w = "{\"ssid\":\"\xe5\xae\xb6\\\"WiFi\\u00e9\",\"pass\":\"12345678\",\"forget\":false}";
    bool f = true;
    CHECK(pfd_json_string(w, strlen(w), "ssid", s, sizeof(s)) && strcmp(s, "\xe5\xae\xb6\"WiFi\xc3\xa9") == 0, "ssid '%s'", s);
    CHECK(pfd_json_bool(w, strlen(w), "forget", &f) && !f, "bool");

    /* moving north at 20 m/s, 1 Hz, speed null → derived */
    pfd_gps_track_t t;
    memset(&t, 0, sizeof(t));
    pfd_link_snapshot_t o;
    for (int i = 0; i < 6; ++i) {
        char b[200];
        double lat = 31.0 + i * 20.0 / 111195.0;
        snprintf(b, sizeof(b), "{\"lat\":%.9f,\"lon\":121.0,\"alt\":50,\"spd\":null,\"hdg\":null,\"acc\":5,\"t\":%d,\"age\":100}",
                 lat, 1000 * i);
        CHECK(pfd_gps_parse(b, strlen(b), &m), "p%d", i);
        pfd_gps_apply(&t, &m, 10.0 + i);
        /* heartbeat repeat of the same fix must not disturb the estimate */
        pfd_gps_apply(&t, &m, 10.5 + i);
    }
    memset(&o, 0, sizeof(o));
    pfd_gps_fill(&t, 15.6, &o);
    CHECK(o.state == PFD_LINK_ACTIVE && o.packets == 12, "state %d packets %lu", o.state, (unsigned long)o.packets);
    CHECK(o.speed_valid && o.speed_derived && fabsf(o.speed_mps - 20.0f) < 0.3f, "derived %.2f", (double)o.speed_mps);
    CHECK(o.track_valid && (o.track_deg < 1.0f || o.track_deg > 359.0f), "track %.1f", (double)o.track_deg);
    CHECK(o.alt_valid && o.alt_m == 50.0f && o.acc_m == 5.0f, "alt");
    memset(&o, 0, sizeof(o));
    pfd_gps_fill(&t, 15.5 + 3.2, &o);
    CHECK(o.state == PFD_LINK_LOST, "lost after 3 s (%d)", o.state);
    memset(&o, 0, sizeof(o));
    pfd_gps_fill(&t, 15.5 + 11, &o);
    CHECK(!o.speed_valid, "stale fix drops speed");
    /* reported speed wins */
    const char *r = "{\"lat\":31.001,\"lon\":121.0,\"alt\":51,\"spd\":12.5,\"hdg\":90,\"acc\":5,\"t\":7000,\"age\":0}";
    pfd_gps_parse(r, strlen(r), &m);
    pfd_gps_apply(&t, &m, 17.0);
    memset(&o, 0, sizeof(o));
    pfd_gps_fill(&t, 17.1, &o);
    CHECK(o.speed_valid && !o.speed_derived && o.speed_mps == 12.5f && o.track_deg == 90.0f, "reported");
    printf(fails ? "%d FAILURES\n" : "link: ALL PASS\n", fails);
    return fails != 0;
}
