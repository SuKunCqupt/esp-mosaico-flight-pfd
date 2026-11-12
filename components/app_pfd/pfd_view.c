// SPDX-License-Identifier: MIT
/* Cockpit-style primary flight display: rotating horizon with pitch ladder
 * and roll scale, speed/altitude tapes, heading box, compass arc, G meter,
 * plus the menu / calibration overlays. All coordinates are screen pixels
 * (480x480, y down). */
#include "include/pfd_view.h"
#include "qrcodegen.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define D2R 0.017453292f
#define CX 240.0f
#define CY 240.0f
#define PX_PER_DEG 5.0f
#define LADDER_CLIP_R 132.0f
#define ROLL_R 170.0f

/* tapes */
#define TAPE_W 80
#define TAPE_TOP 108
#define TAPE_BOT 372
#define SPD_X 6
#define ALT_X (PFD_W - 6 - TAPE_W)
#define SPD_PX_PER_KMH 4.0f
#define ALT_PX_PER_M 1.0f

/* compass arc */
#define CMP_CY 690.0f
#define CMP_RO 262.0f
#define CMP_RI 216.0f
#define CMP_SCALE 1.6f        /* heading degrees per arc degree */

/* menu */
#define MENU_X 50
#define MENU_Y 22
#define MENU_W 380
#define MENU_H 436
#define ITEM_X 70
#define ITEM_W 340
#define ITEM_Y0 74
#define ITEM_H 38
#define ITEM_GAP 4
#define MENU_BTN_X 394
#define MENU_BTN_Y 8
#define MENU_BTN_W 78
#define MENU_BTN_H 40
#define CANCEL_X 170
#define CANCEL_Y 372
#define CANCEL_W 140
#define CANCEL_H 46
/* phone overlay buttons */
#define PH_BTN_Y 404
#define PH_BTN_H 44
#define PH_BTN_W 160
#define PH_BTN_L 60
#define PH_BTN_R 260

enum {
    COL_SKY = 0x2C78D2, COL_GROUND = 0x8A5A2C, COL_WHITE = 0xFFFFFF, COL_BLACK = 0x000000,
    COL_YELLOW = 0xFFD21F, COL_AMBER = 0xFFA51A, COL_GREEN = 0x3BE07A, COL_GREY = 0x8C8C8C,
    COL_DARKGREY = 0x505050, COL_PANEL = 0x15191F, COL_BTN = 0x26313D, COL_BTN_HI = 0x3D6EA8,
    COL_MAGENTA = 0xFF5CF0, COL_CYAN = 0x45D8FF, COL_RED = 0xFF4A3D,
};

static float wrap360(float d)
{
    d = fmodf(d, 360.0f);
    return d < 0 ? d + 360.0f : d;
}

/* ---------------- attitude ---------------- */

static void draw_attitude(pfd_gfx_t *g, const pfd_view_t *v)
{
    const float phi = v->roll_deg * D2R;
    const float ux = cosf(phi), uy = -sinf(phi);   /* along horizon */
    const float nx = -sinf(phi), ny = -cosf(phi);  /* toward sky */
    float pitch = v->pitch_deg;
    const float p0x = CX - nx * pitch * PX_PER_DEG, p0y = CY - ny * pitch * PX_PER_DEG;
    const uint16_t white = pfd_rgb(g, COL_WHITE);

    pfd_gfx_horizon(g, p0x, p0y, nx, ny, pfd_rgb(g, COL_SKY), pfd_rgb(g, COL_GROUND));
    pfd_gfx_line(g, p0x - ux * 800, p0y - uy * 800, p0x + ux * 800, p0y + uy * 800, 2.5f, white);

    /* pitch ladder, clipped to a circle around the aircraft symbol */
    g->clip_cx = CX;
    g->clip_cy = CY;
    g->clip_r = LADDER_CLIP_R;
    for (int p = -90; p <= 90; p += 5) {
        if (p == 0) continue;
        float off = (p - pitch) * PX_PER_DEG;
        if (fabsf(off) > LADDER_CLIP_R + 20) continue;
        float cx = CX + nx * off, cy = CY + ny * off;
        bool major = (p % 10) == 0;
        float half = major ? 52.0f : 22.0f;
        if (major && p < 0) {
            /* below the horizon: dashed, like most PFDs */
            for (int s = -1; s <= 1; s += 2) {
                for (int k = 0; k < 3; ++k) {
                    float a = s * (half - k * 18.0f), b = s * (half - k * 18.0f - 10.0f);
                    pfd_gfx_line(g, cx + ux * a, cy + uy * a, cx + ux * b, cy + uy * b, 2.5f, white);
                }
            }
        } else {
            pfd_gfx_line(g, cx - ux * half, cy - uy * half, cx + ux * half, cy + uy * half, 2.5f, white);
        }
        if (major) {
            char t[8];
            snprintf(t, sizeof(t), "%d", p < 0 ? -p : p);
            float lx = half + 18.0f;
            pfd_text_rot(g, PFD_FONT_S, cx - ux * lx, cy - uy * lx, -phi, t, white);
            pfd_text_rot(g, PFD_FONT_S, cx + ux * lx, cy + uy * lx, -phi, t, white);
        }
    }
    g->clip_r = 0;

    /* roll scale: fixed ticks, moving sky pointer */
    static const int ticks[] = {-60, -45, -30, -20, -10, 10, 20, 30, 45, 60};
    for (unsigned i = 0; i < sizeof(ticks) / sizeof(ticks[0]); ++i) {
        float a = ticks[i] * D2R, s = sinf(a), c = -cosf(a);
        float len = (ticks[i] % 30 == 0) ? 20.0f : 11.0f;
        pfd_gfx_line(g, CX + s * ROLL_R, CY + c * ROLL_R, CX + s * (ROLL_R + len), CY + c * (ROLL_R + len),
                     ticks[i] % 30 == 0 ? 3.0f : 2.5f, white);
    }
    for (int a = -60; a < 60; a += 3) {
        float a0 = a * D2R, a1 = (a + 3) * D2R;
        pfd_gfx_line(g, CX + sinf(a0) * ROLL_R, CY - cosf(a0) * ROLL_R, CX + sinf(a1) * ROLL_R,
                     CY - cosf(a1) * ROLL_R, 2.0f, white);
    }
    /* fixed zero index (points down at the arc) */
    float zt[6] = {CX, CY - ROLL_R - 2, CX - 10, CY - ROLL_R - 18, CX + 10, CY - ROLL_R - 18};
    pfd_gfx_fill_poly(g, zt, 3, white);
    /* sky pointer: rotates with the horizon */
    float tip = ROLL_R - 3, base = ROLL_R - 20, hw = 10;
    float tri[6] = {
        CX + nx * tip, CY + ny * tip,
        CX + nx * base + ux * hw, CY + ny * base + uy * hw,
        CX + nx * base - ux * hw, CY + ny * base - uy * hw,
    };
    bool overbank = fabsf(v->roll_deg) > 45.0f;
    pfd_gfx_fill_poly(g, tri, 3, pfd_rgb(g, overbank ? COL_AMBER : COL_YELLOW));

    /* fixed aircraft symbol */
    const uint16_t blk = pfd_rgb(g, COL_BLACK), yel = pfd_rgb(g, COL_YELLOW);
    for (int side = -1; side <= 1; side += 2) {
        int x_out = (int)CX + side * 112, x_in = (int)CX + side * 42;
        int xl = side < 0 ? x_out : x_in, w = 70;
        pfd_gfx_fill_rect(g, xl - 2, (int)CY - 6, w + 4, 12, blk);
        int sx = side < 0 ? x_in - 10 : x_in;
        pfd_gfx_fill_rect(g, sx - 2, (int)CY - 6, 14, 26, blk);
        pfd_gfx_fill_rect(g, xl, (int)CY - 4, w, 8, yel);
        pfd_gfx_fill_rect(g, sx, (int)CY - 4, 10, 22, yel);
    }
    pfd_gfx_fill_rect(g, (int)CX - 7, (int)CY - 7, 14, 14, blk);
    pfd_gfx_fill_rect(g, (int)CX - 5, (int)CY - 5, 10, 10, yel);
}

/* ---------------- tapes ---------------- */

static uint32_t tone_rgb(pfd_tone_t t)
{
    switch (t) {
    case PFD_TONE_AMBER: return COL_AMBER;
    case PFD_TONE_GREEN: return COL_GREEN;
    case PFD_TONE_RED: return COL_RED;
    default: return COL_GREY;
    }
}

static void draw_tape(pfd_gfx_t *g, int x, bool right, bool live, float value, float px_per_unit,
                      int tick_step, int label_step, const char *title, const char *unit, const char *status,
                      pfd_tone_t tone)
{
    const uint16_t fg = pfd_rgb(g, live ? COL_WHITE : COL_GREY);
    const int mid = (int)CY;
    pfd_gfx_darken_rect(g, x, TAPE_TOP, TAPE_W, TAPE_BOT - TAPE_TOP);
    pfd_gfx_darken_rect(g, x, TAPE_TOP, TAPE_W, TAPE_BOT - TAPE_TOP);
    pfd_gfx_fill_rect(g, right ? x : x + TAPE_W - 2, TAPE_TOP, 2, TAPE_BOT - TAPE_TOP, fg);

    float ref = live ? value : 0.0f;
    int lo = (int)floorf((ref - (mid - TAPE_TOP) / px_per_unit) / tick_step) * tick_step;
    int hi = (int)ceilf((ref + (TAPE_BOT - mid) / px_per_unit) / tick_step) * tick_step;
    for (int t = lo; t <= hi; t += tick_step) {
        if (t < 0 && !right) continue; /* no negative speed */
        float y = mid - (t - ref) * px_per_unit;
        if (y < TAPE_TOP + 4 || y > TAPE_BOT - 4) continue;
        bool lab = (t % label_step) == 0;
        int len = lab ? 14 : 8;
        int tx = right ? x + 2 : x + TAPE_W - 2 - len;
        pfd_gfx_fill_rect(g, tx, (int)y - 1, len, 2, fg);
        if (lab && live) {
            char s[12];
            snprintf(s, sizeof(s), "%d", t);
            int ty = (int)y - pfd_text_height(PFD_FONT_S) / 2;
            if (right) pfd_text(g, PFD_FONT_S, x + 20, ty, s, fg, PFD_ALIGN_LEFT);
            else pfd_text(g, PFD_FONT_S, x + TAPE_W - 20, ty, s, fg, PFD_ALIGN_RIGHT);
        }
    }
    /* readout box */
    const int bw = TAPE_W + 4, bh = 42, bx = x - 2, by = mid - bh / 2;
    pfd_gfx_fill_rect(g, bx, by, bw, bh, pfd_rgb(g, COL_BLACK));
    pfd_gfx_frame_rect(g, bx, by, bw, bh, 2, fg);
    float ax = right ? bx : bx + bw;
    float dir = right ? -1.0f : 1.0f;
    float arrow[6] = {ax, mid - 9.0f, ax + dir * 10.0f, (float)mid, ax, mid + 9.0f};
    pfd_gfx_fill_poly(g, arrow, 3, fg);
    char s[16];
    if (live) snprintf(s, sizeof(s), "%d", (int)lroundf(value));
    else snprintf(s, sizeof(s), "--");
    int ty = mid - pfd_text_height(PFD_FONT_L) / 2;
    pfd_text(g, PFD_FONT_L, bx + bw / 2, ty, s, live ? pfd_rgb(g, COL_GREEN) : fg, PFD_ALIGN_CENTER);

    /* captions */
    const uint16_t cap = pfd_rgb(g, COL_WHITE);
    pfd_gfx_darken_rect(g, x, TAPE_TOP - 26, TAPE_W, 24);
    char head[32];
    snprintf(head, sizeof(head), "%s %s", title, unit);
    pfd_text(g, PFD_FONT_S, x + TAPE_W / 2, TAPE_TOP - 24, head, cap, PFD_ALIGN_CENTER);
    pfd_gfx_darken_rect(g, x, TAPE_BOT + 2, TAPE_W, 24);
    if (status) pfd_text(g, PFD_FONT_S, x + TAPE_W / 2, TAPE_BOT + 4, status, pfd_rgb(g, tone_rgb(tone)), PFD_ALIGN_CENTER);
}

/* ---------------- heading ---------------- */

static const char *heading_label(int deg)
{
    switch (deg) {
    case 0: return "北";
    case 90: return "东";
    case 180: return "南";
    case 270: return "西";
    default: return NULL;
    }
}

static void draw_heading(pfd_gfx_t *g, const pfd_view_t *v)
{
    const float hdg = wrap360(v->heading_deg);
    const uint16_t fg = pfd_rgb(g, v->heading_calibrated ? COL_WHITE : COL_AMBER);

    /* bottom compass arc */
    pfd_gfx_darken_ring(g, CX, CMP_CY, CMP_RI, CMP_RO, (int)(CMP_CY - CMP_RO) - 1, PFD_H - 1);
    pfd_gfx_darken_ring(g, CX, CMP_CY, CMP_RI, CMP_RO, (int)(CMP_CY - CMP_RO) - 1, PFD_H - 1);
    int first = (int)floorf((hdg - 75.0f) / 5.0f) * 5;
    for (int t = first; t <= hdg + 75.0f; t += 5) {
        float a = (t - hdg) / CMP_SCALE * D2R;
        float s = sinf(a), c = -cosf(a);
        float px = CX + s * CMP_RO, py = CMP_CY + c * CMP_RO;
        if (py > PFD_H + 4 || px < -10 || px > PFD_W + 10) continue;
        int deg = ((t % 360) + 360) % 360;
        bool ten = deg % 10 == 0;
        float len = ten ? 13.0f : 7.0f;
        pfd_gfx_line(g, px, py, CX + s * (CMP_RO - len), CMP_CY + c * (CMP_RO - len), 2.0f, fg);
        if (deg % 30 == 0) {
            char buf[8];
            const char *lab = heading_label(deg);
            if (!lab) {
                snprintf(buf, sizeof(buf), "%d", deg / 10);
                lab = buf;
            }
            float r = CMP_RO - 30.0f;
            uint16_t lc = (deg == 0 && v->heading_calibrated) ? pfd_rgb(g, COL_RED) : fg;
            pfd_text_rot(g, PFD_FONT_M, CX + s * r, CMP_CY + c * r, a, lab, lc);
        }
    }
    /* GPS track (magenta diamond on the arc) */
    if (v->track_valid) {
        float d = v->track_deg - hdg;
        while (d > 180) d -= 360;
        while (d < -180) d += 360;
        if (fabsf(d) < 75.0f) {
            float a = d / CMP_SCALE * D2R, s = sinf(a), c = -cosf(a), r = CMP_RO - 4.0f;
            float px = CX + s * r, py = CMP_CY + c * r;
            float tx = cosf(a), ty = sinf(a);
            float dia[8] = {px + s * 9, py + c * 9, px + tx * 7, py + ty * 7, px - s * 9, py - c * 9, px - tx * 7, py - ty * 7};
            pfd_gfx_fill_poly(g, dia, 4, pfd_rgb(g, COL_MAGENTA));
        }
    }
    /* lubber mark */
    float lt[6] = {CX, CMP_CY - CMP_RO + 12, CX - 9, CMP_CY - CMP_RO - 6, CX + 9, CMP_CY - CMP_RO - 6};
    pfd_gfx_fill_poly(g, lt, 3, pfd_rgb(g, COL_YELLOW));

    /* top heading box */
    const int bw = 104, bh = 44, bx = (int)CX - bw / 2, by = 8;
    pfd_gfx_fill_rect(g, bx, by, bw, bh, pfd_rgb(g, COL_BLACK));
    pfd_gfx_frame_rect(g, bx, by, bw, bh, 2, fg);
    char s[16];
    int hd = (int)lroundf(hdg) % 360;
    snprintf(s, sizeof(s), "%03d°", hd);
    pfd_text(g, PFD_FONT_XL, (int)CX, by + (bh - pfd_text_height(PFD_FONT_XL)) / 2, s, fg, PFD_ALIGN_CENTER);
    pfd_text_outlined(g, PFD_FONT_S, bx - 8, by + 12, "航向", pfd_rgb(g, COL_WHITE), PFD_ALIGN_RIGHT);
    if (!v->heading_calibrated)
        pfd_text_outlined(g, PFD_FONT_S, (int)CX, by + bh + 2, "磁力计未校准", pfd_rgb(g, COL_AMBER),
                          PFD_ALIGN_CENTER);
}

/* ---------------- corner readouts ---------------- */

static void draw_corners(pfd_gfx_t *g, const pfd_view_t *v)
{
    char s[32];
    const uint16_t w = pfd_rgb(g, COL_WHITE);
    /* G meter, bottom-left */
    pfd_gfx_darken_rect(g, 8, 400, 118, 64);
    pfd_gfx_darken_rect(g, 8, 400, 118, 64);
    snprintf(s, sizeof(s), "G %.2f", (double)v->g_now);
    uint16_t gc = v->g_now > 2.5f ? pfd_rgb(g, COL_AMBER) : w;
    pfd_text(g, PFD_FONT_L, 16, 403, s, gc, PFD_ALIGN_LEFT);
    snprintf(s, sizeof(s), "峰值 %.2f", (double)v->g_peak);
    pfd_text(g, PFD_FONT_S, 16, 437, s, pfd_rgb(g, COL_CYAN), PFD_ALIGN_LEFT);

    /* attitude numbers, bottom-right */
    pfd_gfx_darken_rect(g, PFD_W - 126, 400, 118, 64);
    pfd_gfx_darken_rect(g, PFD_W - 126, 400, 118, 64);
    snprintf(s, sizeof(s), "俯仰 %+.1f°", (double)v->pitch_deg);
    pfd_text(g, PFD_FONT_S, PFD_W - 118, 408, s, w, PFD_ALIGN_LEFT);
    snprintf(s, sizeof(s), "横滚 %+.1f°", (double)v->roll_deg);
    pfd_text(g, PFD_FONT_S, PFD_W - 118, 434, s, w, PFD_ALIGN_LEFT);

    /* menu button, top-right */
    pfd_gfx_fill_rect(g, MENU_BTN_X, MENU_BTN_Y, MENU_BTN_W, MENU_BTN_H, pfd_rgb(g, COL_PANEL));
    pfd_gfx_frame_rect(g, MENU_BTN_X, MENU_BTN_Y, MENU_BTN_W, MENU_BTN_H, 2, pfd_rgb(g, COL_GREY));
    pfd_text(g, PFD_FONT_M, MENU_BTN_X + MENU_BTN_W / 2, MENU_BTN_Y + 8, "菜单", w, PFD_ALIGN_CENTER);

    /* phone GPS block, top-left (replaces the sim badge once a phone talked to us) */
    if (v->gps_link) {
        pfd_gfx_darken_rect(g, 6, 6, 134, 74);
        pfd_gfx_darken_rect(g, 6, 6, 134, 74);
        bool live = v->gps_link == 1;
        char l1[40];
        if (live && v->gps_fix && v->gps_acc_m >= 0)
            snprintf(l1, sizeof(l1), "手机 ±%dm", (int)lroundf(v->gps_acc_m));
        else
            snprintf(l1, sizeof(l1), "%s", live ? "手机已连接" : "信号丢失");
        pfd_gfx_fill_rect(g, 12, 15, 8, 8, pfd_rgb(g, live ? COL_GREEN : COL_RED));
        pfd_text(g, PFD_FONT_S, 25, 8, l1, pfd_rgb(g, live ? COL_WHITE : COL_RED), PFD_ALIGN_LEFT);
        const uint16_t cc = pfd_rgb(g, live ? COL_WHITE : COL_GREY);
        if (v->gps_fix) {
            snprintf(l1, sizeof(l1), "%.5f°%s", fabs(v->lat), v->lat >= 0 ? "N" : "S");
            pfd_text(g, PFD_FONT_S, 12, 31, l1, cc, PFD_ALIGN_LEFT);
            snprintf(l1, sizeof(l1), "%.5f°%s", fabs(v->lon), v->lon >= 0 ? "E" : "W");
            pfd_text(g, PFD_FONT_S, 12, 54, l1, cc, PFD_ALIGN_LEFT);
        } else {
            pfd_text(g, PFD_FONT_S, 12, 31, "等待定位…", cc, PFD_ALIGN_LEFT);
        }
    } else if (v->sim) {
        pfd_gfx_fill_rect(g, 8, 8, 78, 40, pfd_rgb(g, COL_PANEL));
        pfd_gfx_frame_rect(g, 8, 8, 78, 40, 2, pfd_rgb(g, COL_AMBER));
        pfd_text(g, PFD_FONT_M, 47, 16, "模拟", pfd_rgb(g, COL_AMBER), PFD_ALIGN_CENTER);
    }
}

/* ---------------- overlays ---------------- */

static void panel(pfd_gfx_t *g, int x, int y, int w, int h)
{
    pfd_gfx_fill_rect(g, x, y, w, h, pfd_rgb(g, COL_PANEL));
    pfd_gfx_frame_rect(g, x, y, w, h, 2, pfd_rgb(g, COL_DARKGREY));
}

static void progress_bar(pfd_gfx_t *g, int x, int y, int w, int h, float p, uint32_t col)
{
    p = p < 0 ? 0 : p > 1 ? 1 : p;
    pfd_gfx_fill_rect(g, x, y, w, h, pfd_rgb(g, 0x303840));
    pfd_gfx_fill_rect(g, x, y, (int)(w * p), h, pfd_rgb(g, col));
}

static void draw_menu(pfd_gfx_t *g, const pfd_view_t *v)
{
    pfd_gfx_dim_all(g);
    panel(g, MENU_X, MENU_Y, MENU_W, MENU_H);
    const uint16_t w = pfd_rgb(g, COL_WHITE);
    pfd_text(g, PFD_FONT_L, PFD_W / 2, MENU_Y + 10, "菜单", w, PFD_ALIGN_CENTER);
    char bright[24], sim[24], mount[48];
    snprintf(mount, sizeof(mount), "显示方向：%s", v->mount_mode == 0 ? "平放（屏幕朝上）" : "竖直（默认）");
    snprintf(bright, sizeof(bright), "亮度  %d%%", v->bright_pct);
    snprintf(sim, sizeof(sim), "模拟数据  %s", v->sim ? "开" : "关");
    const char *labels[PFD_MENU_COUNT] = {
        "水平校准（背面朝下平放）", mount, "磁力计校准", "手机定位（GPS）", "峰值G清零", bright, sim,
        "返回",
    };
    for (int i = 0; i < PFD_MENU_COUNT; ++i) {
        int y = ITEM_Y0 + i * (ITEM_H + ITEM_GAP);
        uint32_t bg = (i == v->menu_pressed) ? COL_BTN_HI : COL_BTN;
        pfd_gfx_fill_rect(g, ITEM_X, y, ITEM_W, ITEM_H, pfd_rgb(g, bg));
        pfd_text(g, PFD_FONT_M, ITEM_X + ITEM_W / 2, y + (ITEM_H - pfd_text_height(PFD_FONT_M)) / 2,
                 labels[i], w, PFD_ALIGN_CENTER);
    }
    if (v->info)
        pfd_text(g, PFD_FONT_S, PFD_W / 2, MENU_Y + MENU_H - 28, v->info, pfd_rgb(g, COL_GREY),
                 PFD_ALIGN_CENTER);
}

static void draw_magcal(pfd_gfx_t *g, const pfd_view_t *v)
{
    pfd_gfx_dim_all(g);
    panel(g, 40, 60, 400, 380);
    const uint16_t w = pfd_rgb(g, COL_WHITE);
    pfd_text(g, PFD_FONT_L, PFD_W / 2, 74, "磁力计校准", w, PFD_ALIGN_CENTER);
    pfd_text(g, PFD_FONT_M, PFD_W / 2, 124, "拿起板子，慢慢朝各个方向转动", w, PFD_ALIGN_CENTER);
    pfd_text(g, PFD_FONT_M, PFD_W / 2, 154, "翻过来、竖起来、侧过来都要转到", w, PFD_ALIGN_CENTER);
    pfd_text(g, PFD_FONT_S, PFD_W / 2, 188, "远离磁铁、手机、电脑和金属桌面", pfd_rgb(g, COL_AMBER), PFD_ALIGN_CENTER);
    progress_bar(g, 80, 236, 320, 22, v->progress, COL_GREEN);
    char s[48];
    snprintf(s, sizeof(s), "方向覆盖 %d/26   %d 秒", v->magcal_coverage, (int)v->magcal_seconds);
    pfd_text(g, PFD_FONT_M, PFD_W / 2, 276, s, w, PFD_ALIGN_CENTER);
    pfd_text(g, PFD_FONT_S, PFD_W / 2, 314, "约 20 秒，覆盖够了会自动完成", pfd_rgb(g, COL_GREY), PFD_ALIGN_CENTER);
    pfd_gfx_fill_rect(g, CANCEL_X, CANCEL_Y, CANCEL_W, CANCEL_H, pfd_rgb(g, COL_BTN));
    pfd_text(g, PFD_FONT_M, CANCEL_X + CANCEL_W / 2, CANCEL_Y + 12, "取消", w, PFD_ALIGN_CENTER);
}

static void draw_levelcal(pfd_gfx_t *g, const pfd_view_t *v)
{
    pfd_gfx_dim_all(g);
    panel(g, 40, 60, 400, 380);
    const uint16_t w = pfd_rgb(g, COL_WHITE);
    pfd_text(g, PFD_FONT_L, PFD_W / 2, 74, "水平校准", w, PFD_ALIGN_CENTER);
    pfd_text(g, PFD_FONT_M, PFD_W / 2, 124, "请将设备背面朝下平放在水平桌面上，", w, PFD_ALIGN_CENTER);
    pfd_text(g, PFD_FONT_M, PFD_W / 2, 154, "保持不动", w, PFD_ALIGN_CENTER);
    pfd_text(g, PFD_FONT_S, PFD_W / 2, 188, "屏幕朝上，桌面要水平", pfd_rgb(g, COL_GREY), PFD_ALIGN_CENTER);
    progress_bar(g, 80, 236, 320, 22, v->progress, COL_GREEN);
    char s[48];
    snprintf(s, sizeof(s), "还剩 %.1f 秒", v->levelcal_remaining_s);
    pfd_text(g, PFD_FONT_M, PFD_W / 2, 276, s, w, PFD_ALIGN_CENTER);
    if (v->levelcal_moving)
        pfd_text(g, PFD_FONT_M, PFD_W / 2, 314, "检测到晃动，请放稳", pfd_rgb(g, COL_AMBER), PFD_ALIGN_CENTER);
    pfd_gfx_fill_rect(g, CANCEL_X, CANCEL_Y, CANCEL_W, CANCEL_H, pfd_rgb(g, COL_BTN));
    pfd_text(g, PFD_FONT_M, CANCEL_X + CANCEL_W / 2, CANCEL_Y + 12, "取消", w, PFD_ALIGN_CENTER);
}

/* QR cache: re-encode only when the text changes. */
typedef struct {
    char text[96];
    bool ok;
    uint8_t qr[qrcodegen_BUFFER_LEN_FOR_VERSION(8)];
} qr_cache_t;
static qr_cache_t s_qr[2];

static const uint8_t *qr_get(int slot, const char *text)
{
    qr_cache_t *q = &s_qr[slot];
    if (strcmp(q->text, text) != 0 || !q->text[0]) {
        uint8_t tmp[qrcodegen_BUFFER_LEN_FOR_VERSION(8)];
        snprintf(q->text, sizeof(q->text), "%s", text);
        q->ok = qrcodegen_encodeText(text, tmp, q->qr, qrcodegen_Ecc_MEDIUM, 1, 8, qrcodegen_Mask_AUTO, true);
    }
    return q->ok ? q->qr : NULL;
}

/* Draw a QR centered at cx, top y, fitting max_px (incl. 2-module quiet zone). */
static void draw_qr(pfd_gfx_t *g, int slot, const char *text, int cx, int y, int max_px)
{
    const uint8_t *qr = qr_get(slot, text);
    if (!qr) return;
    int n = qrcodegen_getSize(qr), quiet = 2;
    int sc = max_px / (n + 2 * quiet);
    if (sc < 1) sc = 1;
    int full = (n + 2 * quiet) * sc, x0 = cx - full / 2;
    /* full contrast regardless of the brightness setting: easier to scan */
    pfd_gfx_fill_rect(g, x0, y, full, full, 0xFFFF);
    const uint16_t k = 0x0000;
    for (int r = 0; r < n; ++r) {
        int run = -1;
        for (int c = 0; c <= n; ++c) {
            bool on = c < n && qrcodegen_getModule(qr, c, r);
            if (on && run < 0) run = c;
            if (!on && run >= 0) {
                pfd_gfx_fill_rect(g, x0 + (quiet + run) * sc, y + (quiet + r) * sc, (c - run) * sc, sc, k);
                run = -1;
            }
        }
    }
}

static void button(pfd_gfx_t *g, int x, const char *label, bool primary)
{
    pfd_gfx_fill_rect(g, x, PH_BTN_Y, PH_BTN_W, PH_BTN_H, pfd_rgb(g, primary ? COL_BTN_HI : COL_BTN));
    pfd_text(g, PFD_FONT_M, x + PH_BTN_W / 2, PH_BTN_Y + (PH_BTN_H - pfd_text_height(PFD_FONT_M)) / 2, label,
             pfd_rgb(g, COL_WHITE), PFD_ALIGN_CENTER);
}

static void draw_phone(pfd_gfx_t *g, const pfd_view_t *v)
{
    pfd_gfx_dim_all(g);
    panel(g, 20, 14, 440, 452);
    const uint16_t w = pfd_rgb(g, COL_WHITE), grey = pfd_rgb(g, COL_GREY), amber = pfd_rgb(g, COL_AMBER);
    char s[128];
    if (!v->wifi_up || !v->ap_ssid || !v->ap_ssid[0]) {
        pfd_text(g, PFD_FONT_L, PFD_W / 2, 26, "手机定位", w, PFD_ALIGN_CENTER);
        pfd_text(g, PFD_FONT_M, PFD_W / 2, 200, "Wi-Fi 热点启动中…", w, PFD_ALIGN_CENTER);
        button(g, PH_BTN_R, "返回", false);
        return;
    }
    if (v->phone_page == 0) {
        pfd_text(g, PFD_FONT_L, PFD_W / 2, 24, "① 手机连接热点", w, PFD_ALIGN_CENTER);
        snprintf(s, sizeof(s), "WIFI:T:WPA;S:%s;P:%s;;", v->ap_ssid, v->ap_pass ? v->ap_pass : "");
        draw_qr(g, 0, s, PFD_W / 2, 64, 236);
        snprintf(s, sizeof(s), "热点  %s", v->ap_ssid);
        pfd_text(g, PFD_FONT_M, PFD_W / 2, 306, s, w, PFD_ALIGN_CENTER);
        snprintf(s, sizeof(s), "密码  %s", v->ap_pass ? v->ap_pass : "");
        pfd_text(g, PFD_FONT_L, PFD_W / 2, 334, s, pfd_rgb(g, COL_GREEN), PFD_ALIGN_CENTER);
        if (v->ap_clients > 0) snprintf(s, sizeof(s), "已有 %d 台手机连上热点，点「下一步」", v->ap_clients);
        else snprintf(s, sizeof(s), "用手机相机扫码，或在 Wi-Fi 列表里手动连接");
        pfd_text(g, PFD_FONT_S, PFD_W / 2, 372, s, v->ap_clients > 0 ? pfd_rgb(g, COL_GREEN) : grey, PFD_ALIGN_CENTER);
        button(g, PH_BTN_L, "返回", false);
        button(g, PH_BTN_R, "下一步", true);
        return;
    }
    pfd_text(g, PFD_FONT_L, PFD_W / 2, 24, "② 打开网页并开始定位", w, PFD_ALIGN_CENTER);
    const char *url = v->ap_url ? v->ap_url : "https://192.168.4.1";
    draw_qr(g, 1, url, PFD_W / 2, 62, 212);
    pfd_text(g, PFD_FONT_M, PFD_W / 2, 278, url, w, PFD_ALIGN_CENTER);
    pfd_text(g, PFD_FONT_S, PFD_W / 2, 304, "提示「不安全/不是私密连接」属正常：", amber, PFD_ALIGN_CENTER);
    pfd_text(g, PFD_FONT_S, PFD_W / 2, 324, "iPhone 点 显示详细信息→访问此网站", amber, PFD_ALIGN_CENTER);
    pfd_text(g, PFD_FONT_S, PFD_W / 2, 344, "安卓 点 高级→继续前往；再点 开始定位", amber, PFD_ALIGN_CENTER);
    uint16_t sc = grey;
    if (v->gps_link == 1) {
        if (v->gps_fix && v->gps_acc_m >= 0)
            snprintf(s, sizeof(s), "手机已连接 · 已收到 %lu 条 · 精度 ±%dm", v->gps_packets, (int)lroundf(v->gps_acc_m));
        else
            snprintf(s, sizeof(s), "手机已连接 · 等待定位…");
        sc = pfd_rgb(g, COL_GREEN);
    } else if (v->gps_link == 2) {
        snprintf(s, sizeof(s), "信号丢失：网页要保持在前台");
        sc = pfd_rgb(g, COL_RED);
    } else if (v->sta_url && v->sta_url[0]) {
        snprintf(s, sizeof(s), "同一 Wi-Fi 下也可打开 %s", v->sta_url);
    } else if (v->ap_clients > 0) {
        snprintf(s, sizeof(s), "热点已连接，等待网页…");
    } else {
        snprintf(s, sizeof(s), "还没有手机连上热点");
    }
    pfd_text(g, PFD_FONT_S, PFD_W / 2, 370, s, sc, PFD_ALIGN_CENTER);
    button(g, PH_BTN_L, "上一步", false);
    button(g, PH_BTN_R, v->gps_link == 1 ? "完成" : "返回", v->gps_link == 1);
}

static void draw_boot(pfd_gfx_t *g, const pfd_view_t *v)
{
    pfd_gfx_dim_all(g);
    panel(g, 70, 150, 340, 180);
    const uint16_t w = pfd_rgb(g, COL_WHITE);
    pfd_text(g, PFD_FONT_L, PFD_W / 2, 168, "陀螺仪校准中", w, PFD_ALIGN_CENTER);
    pfd_text(g, PFD_FONT_M, PFD_W / 2, 214, "请把板子放稳，保持静止", w, PFD_ALIGN_CENTER);
    progress_bar(g, 110, 262, 260, 18, v->progress, COL_CYAN);
    pfd_text(g, PFD_FONT_S, PFD_W / 2, 292, "一直在动的话会改用上次的数据", pfd_rgb(g, COL_GREY), PFD_ALIGN_CENTER);
}

static void draw_error(pfd_gfx_t *g, const pfd_view_t *v)
{
    panel(g, 50, 380, 380, 60);
    pfd_text(g, PFD_FONT_M, PFD_W / 2, 398, v->error ? v->error : "传感器初始化失败", pfd_rgb(g, COL_RED),
             PFD_ALIGN_CENTER);
}

static void draw_toast(pfd_gfx_t *g, const char *msg)
{
    int tw = pfd_text_width(PFD_FONT_M, msg) + 40;
    int x = (PFD_W - tw) / 2, y = 296;
    pfd_gfx_fill_rect(g, x, y, tw, 44, pfd_rgb(g, COL_PANEL));
    pfd_gfx_frame_rect(g, x, y, tw, 44, 2, pfd_rgb(g, COL_GREEN));
    pfd_text(g, PFD_FONT_M, PFD_W / 2, y + 10, msg, pfd_rgb(g, COL_WHITE), PFD_ALIGN_CENTER);
}

void pfd_view_draw(pfd_gfx_t *g, const pfd_view_t *v)
{
    g->clip_r = 0;
    draw_attitude(g, v);
    draw_tape(g, SPD_X, false, v->spd_live, v->speed_kmh, SPD_PX_PER_KMH, 5, 10, "速度", "km/h", v->spd_status,
              v->spd_tone);
    draw_tape(g, ALT_X, true, v->alt_live, v->alt_m, ALT_PX_PER_M, 10, 50, "高度", "m", v->alt_status, v->alt_tone);
    draw_heading(g, v);
    draw_corners(g, v);
    switch (v->overlay) {
    case PFD_OVERLAY_BOOT: draw_boot(g, v); break;
    case PFD_OVERLAY_MENU: draw_menu(g, v); break;
    case PFD_OVERLAY_MAGCAL: draw_magcal(g, v); break;
    case PFD_OVERLAY_LEVELCAL: draw_levelcal(g, v); break;
    case PFD_OVERLAY_ERROR: draw_error(g, v); break;
    case PFD_OVERLAY_PHONE: draw_phone(g, v); break;
    default: break;
    }
    if (v->toast) draw_toast(g, v->toast);
}

bool pfd_view_hit_menu_button(int x, int y)
{
    return x >= MENU_BTN_X - 12 && x < MENU_BTN_X + MENU_BTN_W + 8 && y >= 0 && y < MENU_BTN_Y + MENU_BTN_H + 12;
}

int pfd_view_hit_menu_item(int x, int y)
{
    if (x < ITEM_X || x >= ITEM_X + ITEM_W) return -1;
    for (int i = 0; i < PFD_MENU_COUNT; ++i) {
        int y0 = ITEM_Y0 + i * (ITEM_H + ITEM_GAP);
        if (y >= y0 - 2 && y < y0 + ITEM_H + 2) return i;
    }
    return -1;
}

bool pfd_view_hit_magcal_cancel(int x, int y)
{
    return x >= CANCEL_X - 10 && x < CANCEL_X + CANCEL_W + 10 && y >= CANCEL_Y - 10 && y < CANCEL_Y + CANCEL_H + 10;
}

int pfd_view_hit_phone(int x, int y)
{
    if (y < PH_BTN_Y - 10 || y >= PH_BTN_Y + PH_BTN_H + 10) return 0;
    if (x >= PH_BTN_L - 10 && x < PH_BTN_L + PH_BTN_W + 10) return 1;
    if (x >= PH_BTN_R - 10 && x < PH_BTN_R + PH_BTN_W + 10) return 2;
    return 0;
}
