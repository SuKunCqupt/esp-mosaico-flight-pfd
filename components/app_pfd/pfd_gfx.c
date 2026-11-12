// SPDX-License-Identifier: MIT
#include "include/pfd_gfx.h"
#include <math.h>
#include <string.h>
#include "private/pfd_font.h"

static inline int imin(int a, int b) { return a < b ? a : b; }
static inline int imax(int a, int b) { return a > b ? a : b; }

uint16_t pfd_rgb(const pfd_gfx_t *g, uint32_t rgb)
{
    unsigned b = (unsigned)g->bright;
    unsigned r = (((rgb >> 16) & 0xff) * b) >> 8;
    unsigned gg = (((rgb >> 8) & 0xff) * b) >> 8;
    unsigned bl = ((rgb & 0xff) * b) >> 8;
    return (uint16_t)(((r & 0xf8) << 8) | ((gg & 0xfc) << 3) | (bl >> 3));
}

static inline void fill_span(uint16_t *row, int x0, int x1, uint16_t c)
{
    /* [x0, x1) */
    if (x1 <= x0) return;
    uint16_t *p = row + x0, *e = row + x1;
    if (((uintptr_t)p & 2) && p < e) *p++ = c;
    uint32_t c2 = ((uint32_t)c << 16) | c;
    uint32_t *q = (uint32_t *)p;
    while ((uint16_t *)(q + 4) <= e) {
        q[0] = c2; q[1] = c2; q[2] = c2; q[3] = c2;
        q += 4;
    }
    while ((uint16_t *)(q + 1) <= e) *q++ = c2;
    p = (uint16_t *)q;
    if (p < e) *p = c;
}

static inline void darken_span(uint16_t *row, int x0, int x1)
{
    for (int x = x0; x < x1; ++x) row[x] = (row[x] >> 1) & 0x7BEF;
}

void pfd_gfx_fill_rect(pfd_gfx_t *g, int x, int y, int w, int h, uint16_t c)
{
    int x0 = imax(x, 0), x1 = imin(x + w, g->w), y0 = imax(y, 0), y1 = imin(y + h, g->h);
    for (int yy = y0; yy < y1; ++yy) fill_span(g->fb + (size_t)yy * g->stride, x0, x1, c);
}

void pfd_gfx_darken_rect(pfd_gfx_t *g, int x, int y, int w, int h)
{
    int x0 = imax(x, 0), x1 = imin(x + w, g->w), y0 = imax(y, 0), y1 = imin(y + h, g->h);
    for (int yy = y0; yy < y1; ++yy) darken_span(g->fb + (size_t)yy * g->stride, x0, x1);
}

void pfd_gfx_dim_all(pfd_gfx_t *g)
{
    for (int y = 0; y < g->h; ++y) {
        uint16_t *row = g->fb + (size_t)y * g->stride;
        for (int x = 0; x < g->w; ++x) row[x] = (row[x] >> 2) & 0x39E7;
    }
}

void pfd_gfx_frame_rect(pfd_gfx_t *g, int x, int y, int w, int h, int t, uint16_t c)
{
    pfd_gfx_fill_rect(g, x, y, w, t, c);
    pfd_gfx_fill_rect(g, x, y + h - t, w, t, c);
    pfd_gfx_fill_rect(g, x, y + t, t, h - 2 * t, c);
    pfd_gfx_fill_rect(g, x + w - t, y + t, t, h - 2 * t, c);
}

void pfd_gfx_fill_poly(pfd_gfx_t *g, const float *xy, int n, uint16_t c)
{
    if (n < 3) return;
    float miny = xy[1], maxy = xy[1];
    for (int i = 1; i < n; ++i) {
        miny = fminf(miny, xy[2 * i + 1]);
        maxy = fmaxf(maxy, xy[2 * i + 1]);
    }
    int y0 = imax((int)ceilf(miny - 0.5f), 0), y1 = imin((int)floorf(maxy - 0.5f), g->h - 1);
    for (int y = y0; y <= y1; ++y) {
        float yc = y + 0.5f, xl = 1e9f, xr = -1e9f;
        for (int i = 0; i < n; ++i) {
            float ax = xy[2 * i], ay = xy[2 * i + 1];
            float bx = xy[2 * ((i + 1) % n)], by = xy[2 * ((i + 1) % n) + 1];
            if ((ay <= yc && by > yc) || (by <= yc && ay > yc)) {
                float x = ax + (yc - ay) * (bx - ax) / (by - ay);
                xl = fminf(xl, x);
                xr = fmaxf(xr, x);
            }
        }
        if (xl > xr) continue;
        if (g->clip_r > 0) {
            float dy = yc - g->clip_cy;
            if (fabsf(dy) >= g->clip_r) continue;
            float half = sqrtf(g->clip_r * g->clip_r - dy * dy);
            xl = fmaxf(xl, g->clip_cx - half);
            xr = fminf(xr, g->clip_cx + half);
        }
        int xa = imax((int)ceilf(xl - 0.5f), 0), xb = imin((int)floorf(xr - 0.5f), g->w - 1);
        if (xa <= xb) fill_span(g->fb + (size_t)y * g->stride, xa, xb + 1, c);
    }
}

void pfd_gfx_line(pfd_gfx_t *g, float x0, float y0, float x1, float y1, float width, uint16_t c)
{
    float dx = x1 - x0, dy = y1 - y0, l = sqrtf(dx * dx + dy * dy);
    if (l < 1e-3f) return;
    float nx = -dy / l * width * 0.5f, ny = dx / l * width * 0.5f;
    float q[8] = {x0 + nx, y0 + ny, x1 + nx, y1 + ny, x1 - nx, y1 - ny, x0 - nx, y0 - ny};
    pfd_gfx_fill_poly(g, q, 4, c);
}

void pfd_gfx_horizon(pfd_gfx_t *g, float p0x, float p0y, float nx, float ny, uint16_t sky, uint16_t ground)
{
    for (int y = 0; y < g->h; ++y) {
        uint16_t *row = g->fb + (size_t)y * g->stride;
        float k = (y + 0.5f - p0y) * ny - p0x * nx;
        if (fabsf(nx) < 1e-6f) {
            fill_span(row, 0, g->w, k > 0 ? sky : ground);
            continue;
        }
        float xb = -k / nx - 0.5f; /* boundary in pixel index space */
        if (nx > 0) {
            int xs = (int)floorf(xb) + 1;
            xs = imax(0, imin(g->w, xs));
            fill_span(row, 0, xs, ground);
            fill_span(row, xs, g->w, sky);
        } else {
            int xe = (int)ceilf(xb);
            xe = imax(0, imin(g->w, xe));
            fill_span(row, 0, xe, sky);
            fill_span(row, xe, g->w, ground);
        }
    }
}

void pfd_gfx_darken_ring(pfd_gfx_t *g, float cx, float cy, float r_in, float r_out, int y0, int y1)
{
    y0 = imax(y0, 0);
    y1 = imin(y1, g->h - 1);
    for (int y = y0; y <= y1; ++y) {
        float dy = y + 0.5f - cy;
        if (fabsf(dy) >= r_out) continue;
        float ho = sqrtf(r_out * r_out - dy * dy);
        float hi = fabsf(dy) < r_in ? sqrtf(r_in * r_in - dy * dy) : 0;
        uint16_t *row = g->fb + (size_t)y * g->stride;
        int a0 = imax((int)(cx - ho), 0), a1 = imin((int)(cx - hi), g->w);
        int b0 = imax((int)(cx + hi), 0), b1 = imin((int)(cx + ho), g->w);
        if (hi == 0) {
            darken_span(row, a0, b1);
        } else {
            darken_span(row, a0, a1);
            darken_span(row, b0, b1);
        }
    }
}

/* ---------- text ---------- */

static uint32_t utf8_next(const char **ps)
{
    const unsigned char *s = (const unsigned char *)*ps;
    uint32_t c = s[0];
    int n = 1;
    if (c >= 0xF0 && s[1] && s[2] && s[3]) {
        c = ((c & 7u) << 18) | ((s[1] & 63u) << 12) | ((s[2] & 63u) << 6) | (s[3] & 63u);
        n = 4;
    } else if (c >= 0xE0 && s[1] && s[2]) {
        c = ((c & 15u) << 12) | ((s[1] & 63u) << 6) | (s[2] & 63u);
        n = 3;
    } else if (c >= 0xC0 && s[1]) {
        c = ((c & 31u) << 6) | (s[1] & 63u);
        n = 2;
    }
    *ps += n;
    return c;
}

static const pfd_glyph_t *find_glyph(const pfd_font_t *f, uint32_t cp)
{
    int lo = 0, hi = f->count - 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (f->glyphs[mid].cp == cp) return &f->glyphs[mid];
        if (f->glyphs[mid].cp < cp) lo = mid + 1;
        else hi = mid - 1;
    }
    return NULL;
}

int pfd_text_width(pfd_font_id_t fid, const char *s)
{
    const pfd_font_t *f = &pfd_fonts[fid];
    int w = 0;
    while (s && *s) {
        const pfd_glyph_t *gl = find_glyph(f, utf8_next(&s));
        w += gl ? gl->adv : f->size / 2;
    }
    return w;
}

int pfd_text_height(pfd_font_id_t fid) { return pfd_fonts[fid].line_h; }

static inline uint16_t blend565(uint16_t d, uint16_t s, unsigned a)
{
    a = (a + (a >> 7)) >> 3; /* 0..32 */
    uint32_t dd = (d | ((uint32_t)d << 16)) & 0x07E0F81F;
    uint32_t ss = (s | ((uint32_t)s << 16)) & 0x07E0F81F;
    uint32_t r = (dd + (((ss - dd) * a) >> 5)) & 0x07E0F81F;
    return (uint16_t)(r | (r >> 16));
}

static void blit_glyph(pfd_gfx_t *g, const pfd_font_t *f, const pfd_glyph_t *gl, int x, int y, uint16_t c)
{
    const uint8_t *src = f->px + gl->off;
    for (int j = 0; j < gl->h; ++j) {
        int yy = y + gl->y0 + j;
        if ((unsigned)yy >= (unsigned)g->h) continue;
        uint16_t *row = g->fb + (size_t)yy * g->stride;
        const uint8_t *sr = src + j * gl->w;
        for (int i = 0; i < gl->w; ++i) {
            unsigned a = sr[i];
            if (a < 8) continue;
            int xx = x + gl->x0 + i;
            if ((unsigned)xx >= (unsigned)g->w) continue;
            row[xx] = a > 247 ? c : blend565(row[xx], c, a);
        }
    }
}

void pfd_text(pfd_gfx_t *g, pfd_font_id_t fid, int x, int y, const char *s, uint16_t c, pfd_align_t a)
{
    const pfd_font_t *f = &pfd_fonts[fid];
    if (a == PFD_ALIGN_CENTER) x -= pfd_text_width(fid, s) / 2;
    else if (a == PFD_ALIGN_RIGHT) x -= pfd_text_width(fid, s);
    while (s && *s) {
        const pfd_glyph_t *gl = find_glyph(f, utf8_next(&s));
        if (!gl) {
            x += f->size / 2;
            continue;
        }
        if (gl->w) blit_glyph(g, f, gl, x, y, c);
        x += gl->adv;
    }
}

void pfd_text_outlined(pfd_gfx_t *g, pfd_font_id_t fid, int x, int y, const char *s, uint16_t c, pfd_align_t a)
{
    static const int8_t off[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    for (int i = 0; i < 4; ++i) pfd_text(g, fid, x + off[i][0], y + off[i][1], s, 0x0000, a);
    pfd_text(g, fid, x, y, s, c, a);
}

static void blit_glyph_rot(pfd_gfx_t *g, const pfd_font_t *f, const pfd_glyph_t *gl,
                           float cx, float cy, float cs, float sn, uint16_t c)
{
    /* (cx,cy): screen position of the glyph box center. */
    float hw = gl->w * 0.5f, hh = gl->h * 0.5f;
    float ex = fabsf(cs) * hw + fabsf(sn) * hh + 1, ey = fabsf(sn) * hw + fabsf(cs) * hh + 1;
    int x0 = imax((int)(cx - ex), 0), x1 = imin((int)(cx + ex) + 1, g->w);
    int y0 = imax((int)(cy - ey), 0), y1 = imin((int)(cy + ey) + 1, g->h);
    const uint8_t *src = f->px + gl->off;
    float r2 = g->clip_r * g->clip_r;
    for (int y = y0; y < y1; ++y) {
        float dy = y + 0.5f - cy;
        uint16_t *row = g->fb + (size_t)y * g->stride;
        for (int x = x0; x < x1; ++x) {
            float dx = x + 0.5f - cx;
            float u = dx * cs + dy * sn + hw, v = -dx * sn + dy * cs + hh;
            int iu = (int)floorf(u), iv = (int)floorf(v);
            if (iu < 0 || iv < 0 || iu >= gl->w || iv >= gl->h) continue;
            unsigned a = src[iv * gl->w + iu];
            if (a < 8) continue;
            if (g->clip_r > 0) {
                float qx = x + 0.5f - g->clip_cx, qy = y + 0.5f - g->clip_cy;
                if (qx * qx + qy * qy > r2) continue;
            }
            row[x] = a > 247 ? c : blend565(row[x], c, a);
        }
    }
}

void pfd_text_rot(pfd_gfx_t *g, pfd_font_id_t fid, float cx, float cy, float angle, const char *s, uint16_t c)
{
    const pfd_font_t *f = &pfd_fonts[fid];
    float cs = cosf(angle), sn = sinf(angle);
    float W = (float)pfd_text_width(fid, s), H = (float)f->line_h;
    float pen = 0;
    while (s && *s) {
        const pfd_glyph_t *gl = find_glyph(f, utf8_next(&s));
        if (!gl) {
            pen += f->size / 2;
            continue;
        }
        if (gl->w) {
            float u = pen + gl->x0 + gl->w * 0.5f - W * 0.5f;
            float v = gl->y0 + gl->h * 0.5f - H * 0.5f;
            blit_glyph_rot(g, f, gl, cx + u * cs - v * sn, cy + u * sn + v * cs, cs, sn, c);
        }
        pen += gl->adv;
    }
}
