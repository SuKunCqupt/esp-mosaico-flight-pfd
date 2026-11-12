// SPDX-License-Identifier: MIT
/* Host stand-in for the sensor service: animates a gentle flight so the
 * simulator exercises every instrument. PFD_HOST_POSE="roll,pitch,heading"
 * freezes a pose for screenshots. Single-threaded (Host loop). */
#include "include/pfd_sensors.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static pfd_sens_snapshot_t s_snap;
static unsigned s_reads;
static float s_peak = 1.0f;

int pfd_sensors_start(void)
{
    s_snap.state = PFD_SENS_GYRO_CAL;
    s_snap.imu_ok = true;
    s_snap.mag_ok[0] = s_snap.mag_ok[1] = true;
    s_snap.mag_calibrated = true;
    s_snap.mag_source = 0;
    s_snap.imu_rate_hz = 200;
    s_snap.mount_mode = 1; /* upright */
    return 0;
}

void pfd_sensors_read(pfd_sens_snapshot_t *out)
{
    float t = (s_reads++) / 60.0f;
    const char *pose = getenv("PFD_HOST_POSE");
    if (s_snap.state == PFD_SENS_GYRO_CAL) {
        s_snap.progress = t / 1.0f;
        if (t >= 1.0f) s_snap.state = PFD_SENS_RUN;
    } else if (s_snap.state == PFD_SENS_LEVELCAL) {
        s_snap.progress += 1.0f / 120.0f;
        s_snap.levelcal_remaining_s = 2.0f * (1.0f - s_snap.progress);
        if (s_snap.progress >= 1.0f) {
            s_snap.state = PFD_SENS_RUN;
            s_snap.levelcal_result = getenv("PFD_HOST_NOTFLAT") ? PFD_LEVELCAL_NOT_FLAT : PFD_LEVELCAL_OK;
            s_snap.levelcal_tilt_deg = getenv("PFD_HOST_NOTFLAT") ? 32.0f : 2.1f;
            s_snap.levelcal_result_seq++;
            if (s_snap.levelcal_result == PFD_LEVELCAL_OK) {
                s_snap.level_set = true;
                s_snap.align_deg = 2.1f;
            }
        }
    } else if (s_snap.state == PFD_SENS_MAGCAL) {
        s_snap.magcal_seconds += 1.0f / 60.0f;
        s_snap.progress = s_snap.magcal_seconds / 6.0f;
        s_snap.magcal_coverage = (int)(s_snap.progress * 24);
        if (s_snap.progress >= 1.0f) {
            s_snap.state = PFD_SENS_RUN;
            s_snap.magcal_result = PFD_MAGCAL_OK;
            s_snap.magcal_result_seq++;
        }
    }
    float r = 30.0f * sinf(t * 0.6f), p = 8.0f * sinf(t * 0.37f), h = fmodf(40.0f + t * 12.0f, 360.0f);
    if (pose) sscanf(pose, "%f,%f,%f", &r, &p, &h);
    s_snap.roll_deg = r;
    s_snap.pitch_deg = p;
    s_snap.heading_deg = h;
    s_snap.g_now = 1.0f / cosf(r * 0.0174533f);
    if (s_snap.g_now > s_peak) s_peak = s_snap.g_now;
    s_snap.g_peak = s_peak;
    *out = s_snap;
}

void pfd_sensors_command(pfd_sens_cmd_t cmd)
{
    if (cmd == PFD_CMD_MAGCAL_START && s_snap.state == PFD_SENS_RUN) {
        s_snap.state = PFD_SENS_MAGCAL;
        s_snap.magcal_seconds = 0;
    } else if (cmd == PFD_CMD_MAGCAL_CANCEL && s_snap.state == PFD_SENS_MAGCAL) {
        s_snap.state = PFD_SENS_RUN;
        s_snap.magcal_result = PFD_MAGCAL_CANCELLED;
        s_snap.magcal_result_seq++;
    } else if (cmd == PFD_CMD_PEAK_RESET) {
        s_peak = 1.0f;
    } else if (cmd == PFD_CMD_LEVEL && s_snap.state == PFD_SENS_RUN) {
        s_snap.state = PFD_SENS_LEVELCAL;
        s_snap.progress = 0;
        s_snap.levelcal_remaining_s = 2.0f;
    } else if (cmd == PFD_CMD_LEVEL_CANCEL && s_snap.state == PFD_SENS_LEVELCAL) {
        s_snap.state = PFD_SENS_RUN;
        s_snap.levelcal_result = PFD_LEVELCAL_CANCELLED;
        s_snap.levelcal_result_seq++;
    } else if (cmd == PFD_CMD_MOUNT_FLAT || cmd == PFD_CMD_MOUNT_UPRIGHT) {
        s_snap.mount_mode = cmd == PFD_CMD_MOUNT_FLAT ? 0 : 1;
    }
}
