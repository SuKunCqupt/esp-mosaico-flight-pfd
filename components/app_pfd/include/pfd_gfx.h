// SPDX-License-Identifier: MIT
#pragma once
/* Minimal RGB565 software rasterizer for the flight display. Writes straight
 * into the Engine framebuffer; no allocation. A frame context carries the
 * target and the global brightness applied to every color. Single owner
 * (the render task). */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t *fb;
    int w, h;
    size_t stride;
    int bright;          /* 0..256 */
    /* optional circular clip for polygons/text (r <= 0 disables) */
    float clip_cx, clip_cy, clip_r;
} pfd_gfx_t;

typedef enum { PFD_FONT_S = 0, PFD_FONT_M, PFD_FONT_L, PFD_FONT_XL, PFD_FONT_COUNT } pfd_font_id_t;
typedef enum { PFD_ALIGN_LEFT = 0, PFD_ALIGN_CENTER, PFD_ALIGN_RIGHT } pfd_align_t;

uint16_t pfd_rgb(const pfd_gfx_t *g, uint32_t rgb888);
void pfd_gfx_fill_rect(pfd_gfx_t *g, int x, int y, int w, int h, uint16_t c);
void pfd_gfx_darken_rect(pfd_gfx_t *g, int x, int y, int w, int h);
void pfd_gfx_frame_rect(pfd_gfx_t *g, int x, int y, int w, int h, int t, uint16_t c);
/* Convex polygon, honoring the circular clip if set. */
void pfd_gfx_fill_poly(pfd_gfx_t *g, const float *xy, int n, uint16_t c);
void pfd_gfx_line(pfd_gfx_t *g, float x0, float y0, float x1, float y1, float width, uint16_t c);
/* Horizon: pixels with (p - p0) . n > 0 get sky, else ground. */
void pfd_gfx_horizon(pfd_gfx_t *g, float p0x, float p0y, float nx, float ny, uint16_t sky, uint16_t ground);
/* Darken pixels between radii r_in..r_out of (cx,cy), rows y0..y1. */
void pfd_gfx_darken_ring(pfd_gfx_t *g, float cx, float cy, float r_in, float r_out, int y0, int y1);
void pfd_gfx_dim_all(pfd_gfx_t *g);

int pfd_text_width(pfd_font_id_t f, const char *s);
int pfd_text_height(pfd_font_id_t f);
void pfd_text(pfd_gfx_t *g, pfd_font_id_t f, int x, int y, const char *s, uint16_t c, pfd_align_t a);
void pfd_text_outlined(pfd_gfx_t *g, pfd_font_id_t f, int x, int y, const char *s, uint16_t c, pfd_align_t a);
/* Text centered at (cx,cy) rotated by angle (radians, screen CW positive). */
void pfd_text_rot(pfd_gfx_t *g, pfd_font_id_t f, float cx, float cy, float angle, const char *s, uint16_t c);

#ifdef __cplusplus
}
#endif
