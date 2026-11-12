// SPDX-License-Identifier: MIT
#pragma once
/* Primary flight display composition. Pure drawing: takes a view model,
 * writes one full 480x480 frame. No state besides what is passed in. */
#include <stdbool.h>
#include "pfd_gfx.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { PFD_W = 480, PFD_H = 480 };

typedef enum {
    PFD_OVERLAY_NONE = 0,
    PFD_OVERLAY_BOOT,     /* gyro calibration: hold still */
    PFD_OVERLAY_MENU,
    PFD_OVERLAY_MAGCAL,
    PFD_OVERLAY_LEVELCAL, /* 水平校准: lying flat on its back */
    PFD_OVERLAY_ERROR,
    PFD_OVERLAY_PHONE,    /* phone GPS setup: hotspot QR, then page QR */
} pfd_overlay_t;

typedef enum { PFD_TONE_GREY = 0, PFD_TONE_AMBER, PFD_TONE_GREEN, PFD_TONE_RED } pfd_tone_t;

enum {
    PFD_MENU_LEVEL = 0,
    PFD_MENU_MOUNT,
    PFD_MENU_MAGCAL,
    PFD_MENU_PHONE,
    PFD_MENU_PEAK,
    PFD_MENU_BRIGHT,
    PFD_MENU_SIM,
    PFD_MENU_CLOSE,
    PFD_MENU_COUNT,
};

typedef struct {
    float roll_deg, pitch_deg, heading_deg;
    bool heading_calibrated;
    float g_now, g_peak;
    bool sim;               /* simulated speed/altitude (badge) */
    /* tapes: the runtime picks phone GPS > simulation > none */
    bool spd_live, alt_live;
    float speed_kmh, alt_m, vs_mps;
    const char *spd_status, *alt_status;
    pfd_tone_t spd_tone, alt_tone;
    /* phone GPS block (top-left) and track marker */
    int gps_link;           /* 0 never, 1 live, 2 lost */
    bool gps_fix;
    double lat, lon;
    float gps_acc_m;
    bool track_valid;
    float track_deg;
    /* phone setup overlay */
    int phone_page;         /* 0 hotspot, 1 web page */
    bool wifi_up, server_up;
    const char *ap_ssid, *ap_pass, *ap_url, *sta_url;
    int ap_clients;
    unsigned long gps_packets;
    int bright_pct;
    pfd_overlay_t overlay;
    float progress;         /* boot / magcal */
    int magcal_coverage;
    float magcal_seconds;
    int menu_pressed;       /* highlighted menu row or -1 */
    const char *toast;      /* short message or NULL */
    const char *info;       /* menu footer line */
    int mount_mode;         /* display: 0 flat, 1 upright */
    bool level_set;         /* IMU alignment calibrated */
    float levelcal_remaining_s;
    bool levelcal_moving;
    const char *error;
} pfd_view_t;

/* Draw everything into g (g->bright must be set). */
void pfd_view_draw(pfd_gfx_t *g, const pfd_view_t *v);

/* Hit tests in screen pixels. */
bool pfd_view_hit_menu_button(int x, int y);
int pfd_view_hit_menu_item(int x, int y);       /* -1 if none */
bool pfd_view_hit_magcal_cancel(int x, int y);
/* Phone overlay buttons: 0 none, 1 left, 2 right. */
int pfd_view_hit_phone(int x, int y);

#ifdef __cplusplus
}
#endif
