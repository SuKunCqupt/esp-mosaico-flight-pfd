// SPDX-License-Identifier: MIT
#pragma once
/* Phone GPS link: the board runs a Wi-Fi hotspot plus an HTTPS page; the
 * phone's browser posts navigator.geolocation fixes to it. The render loop
 * only ever calls pfd_link_read() (lock + memcpy). Network work runs on core 1
 * at low priority so the IMU task and the frame loop are not disturbed. */
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PFD_LINK_TIMEOUT_S 3.0f

typedef enum {
    PFD_LINK_NONE = 0,   /* no phone data yet */
    PFD_LINK_ACTIVE,     /* packet within PFD_LINK_TIMEOUT_S */
    PFD_LINK_LOST,       /* had data, now silent */
} pfd_link_state_t;

typedef enum { PFD_STA_OFF = 0, PFD_STA_CONNECTING, PFD_STA_CONNECTED, PFD_STA_FAILED } pfd_sta_state_t;

typedef struct {
    /* network */
    bool wifi_up, server_up;
    char ap_ssid[40];
    char ap_pass[16];
    char ap_url[32];
    int ap_clients;
    pfd_sta_state_t sta;
    char sta_ssid[33];
    char sta_url[32];
    /* phone data */
    pfd_link_state_t state;
    float since_s;          /* since last packet (valid if packets > 0) */
    uint32_t packets;
    bool has_fix;
    double lat, lon;
    bool alt_valid;
    float alt_m;
    bool speed_valid, speed_derived;
    float speed_mps;
    bool track_valid;
    float track_deg;
    float acc_m;            /* horizontal accuracy, < 0 unknown */
    float alt_acc_m;        /* < 0 unknown */
    float fix_age_s;        /* phone-side age of the fix at receive time + since */
} pfd_link_snapshot_t;

/* Start Wi-Fi + servers in the background. Returns 0 if the task started. */
int pfd_link_start(void);
void pfd_link_read(pfd_link_snapshot_t *out);

#ifdef __cplusplus
}
#endif
