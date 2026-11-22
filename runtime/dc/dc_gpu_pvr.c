/* dc_gpu_pvr.c - PS1 polygons and sprites on the Dreamcast PowerVR.
 *
 * Polygons and rectangles drawn into the frame being built (a drawing area
 * about the size of the display) go to the PowerVR instead of the software
 * rasterizer; anything drawn elsewhere in VRAM (render to texture) still goes
 * through software so it can be used as a texture later.
 *
 * - Everything is sent to the translucent list with presorting off, so the
 *   PowerVR draws primitives in submission order like the PS1 (painter's
 *   order, no depth test).
 * - 4-bit texture pages become paletted PowerVR textures (one twiddled page
 *   per VRAM page, CLUTs in the 64 palette banks). 8-bit and 15-bit pages
 *   are converted per (page, CLUT) to ARGB1555 in 16x16-texel cells as
 *   they are needed. Colour 0 is transparent. Everything is checked against VRAM
 *   write generations, so uploads and CLUT changes are seen.
 * - Texture windows are converted as small repeating textures.
 * - Semi-transparency: average = alpha 0.5, add = ONE, subtract is
 *   approximated by dst * (1 - src), B + F/4 = alpha 0.25 added.
 * - A frame that textures from itself (the heat haze over a burning village)
 *   is rendered to a texture first, then shown with those tiles on top.
 * - Light frames (menus, text) are also drawn in software and VRAM's copy is
 *   shown, which keeps mask bits, subtraction and frame feedback exact.
 * - A frame is closed when the game moves its display start (buffer flip).
 *
 * Not done yet: lines, the 2x colour range of texture modulation, exact
 * subtractive blending.
 */
#include "dc_gpu.h"
#include <kos.h>
#include <dc/pvr.h>
#include <string.h>
#include <stdio.h>

int g_dc_pvr_on;                    /* set once dc_gpu_pvr_init() ran */
static int s_scene;                 /* a scene and the TR list are open */
static int s_ox, s_oy;              /* VRAM origin of the frame being built */
static float s_sx = 2.0f, s_sy = 2.0f;
static uint32_t s_frame;
static int s_trace_on;
#ifdef DC_PVR_TRACE
static unsigned s_trace_left = 20;
#define TRACE(...) do { if (s_trace_on && s_trace_left) { char tb_[160]; snprintf(tb_, sizeof tb_, __VA_ARGS__); dbgio_write_str(tb_); } } while (0)
#else
#define TRACE(...) do {} while (0)
#endif

static uint32_t s_hdr_key = ~0u;
static unsigned s_prims, s_vblanks_open;
static int s_bg;
unsigned g_dc_pvr_stat[4];
/* Frames left to draw in software: set when the game textures from a frame
   buffer (feedback effects), which the PowerVR's frames never reach. */
static unsigned s_soft_frames;          /* prims drawn, prims declined, textures converted, scenes */

/* Render to texture, for frames that texture from themselves (the heat
   haze over a burning village redraws the whole frame as 280 distorted
   tiles). Such a frame is first rendered into s_rtt; at the first tile the
   scene is finished there and a new one shows s_rtt and the tiles over it. */
#define RTT_W 1024
#define RTT_H 512
static pvr_ptr_t s_rtt;
static int s_rtt_want;              /* the last frames read themselves */
static int s_rtt_scene;             /* the open scene renders into s_rtt */
static int s_rtt_split;             /* s_rtt holds this frame's start */
static int s_rtt_hit;               /* this frame read itself */
static unsigned s_rtt_idle;         /* frames since one read itself */

/* ---- texture cache ------------------------------------------------------ */
#define NTEX 24            /* at most; as many as fit in video memory */
static int s_ntex;
typedef struct { uint32_t key, win, stamp, used; uint16_t cells[16]; pvr_ptr_t tex; } TexEnt;
static TexEnt s_tex[NTEX];
static uint16_t s_conv[256 * 256] __attribute__((aligned(32)));
static uint16_t s_clut[256];

/* Texture window of a primitive: w x h texels at (u0, v0) of the page. */
static inline int win_w(const DcPrim* p) { return p->tw_w > 0 && p->tw_w < 256 ? p->tw_w : 256; }
static inline int win_h(const DcPrim* p) { return p->tw_h > 0 && p->tw_h < 256 ? p->tw_h : 256; }

static inline uint16_t to1555(uint16_t c)
{
    if (!c) return 0;
    return (uint16_t)(0x8000 | ((c & 31) << 10) | (c & (31 << 5)) | ((c >> 10) & 31));
}

static uint32_t stamp_of(int x, int w, int y, int h)
{
    uint32_t s = 0;
    int bx0 = (x & 1023) >> 6, bx1 = ((x + w - 1) & 1023) >> 6;
    int by0 = (y & 511) >> 4, by1 = ((y + h - 1) & 511) >> 4;
    for (int by = by0;; by = (by + 1) & 31) {
        for (int bx = bx0;; bx = (bx + 1) & 15) {
            s += g_dc_vram_gen[by][bx] * (uint32_t)(1 + bx + by * 16);
            if (bx == bx1) break;
        }
        if (by == by1) break;
    }
    return s;
}

/* Convert rows y0..y1-1 of a full page (window-less), or the whole texture
   for texture windows and VRAM blits. */
static void convert(const DcPrim* p, int y0, int y1, int x0, int x1)
{
    if (p->depth == 3) {            /* VRAM as is, opaque (copies and uploads) */
        uint16_t* o = s_conv;
        for (int v = 0; v < 256; v++) {
            const uint16_t* row = &g_dc_vram[((p->tp_y + (uint32_t)v) & 511) << 10];
            for (int u = 0; u < 256; u++) {
                uint16_t c = row[(p->tp_x + (uint32_t)u) & 1023];
                *o++ = (uint16_t)(0x8000 | ((c & 31) << 10) | (c & (31 << 5)) | ((c >> 10) & 31));
            }
        }
        return;
    }
    const int depth = p->depth > 2 ? 2 : p->depth;
    if (depth < 2) {
        const uint16_t* cl = &g_dc_vram[(p->clut_y & 511) << 10];
        int n = depth == 0 ? 16 : 256;
        for (int i = 0; i < n; i++) s_clut[i] = to1555(cl[(p->clut_x + (uint32_t)i) & 1023]);
    }
    const int ww = win_w(p), wh = win_h(p);
    if (ww < 256 || wh < 256) {         /* texture window: just that area */
        uint16_t* o = s_conv;
        for (int v = 0; v < wh; v++) {
            const uint16_t* row = &g_dc_vram[((p->tp_y + (uint32_t)(p->tw_v0 + v)) & 511) << 10];
            for (int u = p->tw_u0; u < p->tw_u0 + ww; u++) {
                if (depth == 0) *o++ = s_clut[(row[(p->tp_x + (uint32_t)(u >> 2)) & 1023] >> ((u & 3) * 4)) & 15];
                else if (depth == 1) *o++ = s_clut[(row[(p->tp_x + (uint32_t)(u >> 1)) & 1023] >> ((u & 1) * 8)) & 255];
                else *o++ = to1555(row[(p->tp_x + (uint32_t)u) & 1023]);
            }
        }
        return;
    }
    for (int v = y0; v < y1; v++) {
        const uint16_t* row = &g_dc_vram[((p->tp_y + (uint32_t)v) & 511) << 10];
        uint16_t* o = s_conv + v * 256 + x0;
        if (depth == 0) {
            for (int u = x0; u < x1; u += 4) {
                uint16_t w = row[(p->tp_x + (uint32_t)(u >> 2)) & 1023];
                o[0] = s_clut[w & 15]; o[1] = s_clut[(w >> 4) & 15];
                o[2] = s_clut[(w >> 8) & 15]; o[3] = s_clut[w >> 12];
                o += 4;
            }
        } else if (depth == 1) {
            for (int u = x0; u < x1; u += 2) {
                uint16_t w = row[(p->tp_x + (uint32_t)(u >> 1)) & 1023];
                o[0] = s_clut[w & 255]; o[1] = s_clut[w >> 8];
                o += 2;
            }
        } else {
            for (int u = x0; u < x1; u++) *o++ = to1555(row[(p->tp_x + (uint32_t)u) & 1023]);
        }
    }
}

/* Convert and upload the 16x16-texel cells of band b set in cols. */
static void fill_cells(TexEnt* e, const DcPrim* p, int b, uint32_t cols)
{
    for (int c = 0; c < 16;) {
        if (!(cols >> c & 1)) { c++; continue; }
        int c1 = c;
        while (c1 + 1 < 16 && (cols >> (c1 + 1) & 1)) c1++;
        int x0 = c * 16, x1 = (c1 + 1) * 16;
        convert(p, b * 16, b * 16 + 16, x0, x1);
        for (int y = b * 16; y < b * 16 + 16; y++) {
            uint32_t* d = (uint32_t*)e->tex + (y * 256 + x0) / 2;
            const uint32_t* sr = (const uint32_t*)s_conv + (y * 256 + x0) / 2;
            for (int j = 0; j < (x1 - x0) / 2; j += 8) {
                d[j] = sr[j]; d[j + 1] = sr[j + 1]; d[j + 2] = sr[j + 2]; d[j + 3] = sr[j + 3];
                d[j + 4] = sr[j + 4]; d[j + 5] = sr[j + 5]; d[j + 6] = sr[j + 6]; d[j + 7] = sr[j + 7];
            }
        }
        c = c1 + 1;
    }
}

#ifdef DC_PVR_TEXSTAT
static unsigned s_tstat[4][2];      /* conversions by depth: new key, refresh */
static unsigned s_pstat[7];         /* paletted pages converted, banks loaded, fallbacks */
#endif
static pvr_ptr_t texture_for(const DcPrim* p)
{
    const int depth = p->depth > 3 ? 2 : p->depth;
    uint32_t key = (uint32_t)depth << 30 | (p->tp_x >> 6) << 26 | (p->tp_y >> 8) << 25;
    if (depth < 2) key |= (p->clut_x >> 4) << 9 | (p->clut_y & 511);
    if (depth == 3) key = 3u << 30 | (p->tp_x & 1023) << 9 | (p->tp_y & 511);
    int tw = depth == 3 ? 256 : 64 << depth;
    uint32_t st = stamp_of((int)p->tp_x, tw, (int)p->tp_y, 256);
    if (depth < 2) st = st * 31u + stamp_of((int)p->clut_x, depth == 0 ? 16 : 256, (int)p->clut_y, 1);
    const uint32_t win = depth == 3 ? 0 : (uint32_t)(win_w(p) & 511) | (uint32_t)(win_h(p) & 511) << 9 |
                         (uint32_t)(p->tw_u0 & 255) << 18 | (uint32_t)(p->tw_v0 & 63) << 26;
    /* full pages are converted in cells of 16x16 texels, as primitives need them */
    const int banded = depth != 3 && win_w(p) == 256 && win_h(p) == 256;
    int b0 = 0, b1 = 15;
    uint32_t cols = 0xFFFF;
    if (banded) {
        int u0 = p->v[0].u, u1 = u0, v0 = p->v[0].v, v1 = v0;
        for (int k = 1; k < p->n; k++) {
            u0 = p->v[k].u < u0 ? p->v[k].u : u0; u1 = p->v[k].u > u1 ? p->v[k].u : u1;
            v0 = p->v[k].v < v0 ? p->v[k].v : v0; v1 = p->v[k].v > v1 ? p->v[k].v : v1;
        }
        u0 = u0 < 0 ? 0 : u0 > 255 ? 255 : u0; u1 = u1 < 0 ? 0 : u1 > 255 ? 255 : u1;
        v0 = v0 < 0 ? 0 : v0 > 255 ? 255 : v0; v1 = v1 < 0 ? 0 : v1 > 255 ? 255 : v1;
        b0 = v0 >> 4; b1 = v1 >> 4;
        cols = (0xFFFFu >> (15 - (u1 >> 4))) & (0xFFFFu << (u0 >> 4));
    }
    int victim = -1;
    for (int i = 0; i < s_ntex; i++) {
        TexEnt* e = &s_tex[i];
        if (e->key == key && e->win == win) {
            e->used = s_frame;
            if (e->stamp == st) {
                /* cells not drawn from before: safe to add while in use */
                for (int b = b0; b <= b1; b++) {
                    uint32_t miss = cols & ~(uint32_t)e->cells[b];
                    if (!miss) continue;
                    fill_cells(e, p, b, miss);
                    e->cells[b] |= (uint16_t)miss;
                }
                return e->tex;
            }
            victim = i;                 /* same page, VRAM changed: refresh */
#ifdef DC_PVR_TEXSTAT
            { static int n; if (n < 30 && depth == 2) { n++; char b[96]; snprintf(b, sizeof b, "[t15] refresh tp=%u,%u frame %u\n", p->tp_x, p->tp_y, s_frame); dbgio_write_str(b); } }
#endif
            break;
        }
        /* oldest entry not used in this or the previous frame (the PowerVR
           may still be drawing that one) */
        if (s_frame - e->used >= 2 && (victim < 0 || e->used < s_tex[victim].used)) victim = i;
    }
    if (victim < 0) {               /* everything is in use: take the oldest */
        victim = 0;
        for (int i = 1; i < s_ntex; i++) if (s_tex[i].used < s_tex[victim].used) victim = i;
    }
#ifdef DC_PVR_TEXSTAT
    s_tstat[depth][victim >= 0 && s_tex[victim].key == key && s_tex[victim].win == win]++;
#endif
    g_dc_pvr_stat[2]++;
    TexEnt* e = &s_tex[victim];
    /* plain CPU copy: the store queues are busy feeding the TA */
    if (banded) {
        memset(e->cells, 0, sizeof e->cells);
        for (int b = b0; b <= b1; b++) { fill_cells(e, p, b, cols); e->cells[b] = (uint16_t)cols; }
    } else {
        convert(p, 0, 256, 0, 256);
        uint32_t* d = (uint32_t*)e->tex;
        const uint32_t* s = (const uint32_t*)s_conv;
        const int words = depth == 3 ? 256 * 256 / 2 : win_w(p) * win_h(p) / 2;
        for (int i = 0; i < words; i += 4) { d[i] = s[i]; d[i + 1] = s[i + 1]; d[i + 2] = s[i + 2]; d[i + 3] = s[i + 3]; }
    }
    if (!banded) memset(e->cells, 0xFF, sizeof e->cells);
    e->key = key; e->win = win; e->stamp = st; e->used = s_frame;
    return e->tex;
}

/* ---- 4-bit pages as paletted PowerVR textures ---------------------------
   A 4-bit page is converted once (32 KB, twiddled) whatever CLUT it is
   drawn with; each CLUT goes into one of the PowerVR's 64 palette banks of
   16 colours. Banks and pages used in this or the previous frame (which the
   PowerVR may still be drawing) are never replaced. */
#define NPAGE 64
#define NPAGE8 0           /* 8-bit pages: frames use up to ~18 CLUTs, more than the 4 banks */
typedef struct { uint32_t key, stamp, used; pvr_ptr_t tex; uint32_t gen[16]; } PageEnt;
static PageEnt s_page[NPAGE], s_page8[NPAGE8 ? NPAGE8 : 1];
static int s_npage, s_npage8;
typedef struct { uint32_t key, stamp, used; } BankEnt;
/* palette RAM: all 64 banks of 16 for 4-bit CLUTs (frames use up to ~40) */
#define NBANK4 64
#define NBANK8 1
#define BANK8_FIRST 3
static BankEnt s_bank[NBANK4], s_bank8[NBANK8];
static uint16_t s_twid[256];

#ifdef DC_PVR_TEXSTAT
static uint32_t s_cl_seen[2][64]; static int s_cl_n[2], s_cl_max[2]; static uint32_t s_cl_frame;
static void clut_count(int eight, uint32_t key)
{
    if (s_cl_frame != s_frame) {
        for (int k = 0; k < 2; k++) { if (s_cl_n[k] > s_cl_max[k]) s_cl_max[k] = s_cl_n[k]; s_cl_n[k] = 0; }
        s_cl_frame = s_frame;
    }
    for (int i = 0; i < s_cl_n[eight]; i++) if (s_cl_seen[eight][i] == key) return;
    if (s_cl_n[eight] < 64) s_cl_seen[eight][s_cl_n[eight]++] = key;
}
#endif
static int bank_for(const DcPrim* p)
{
    const int eight = p->depth == 1, nb = eight ? NBANK8 : NBANK4, nc = eight ? 256 : 16;
    BankEnt* banks = eight ? s_bank8 : s_bank;
    uint32_t key = (p->clut_x >> 4) << 9 | (p->clut_y & 511);
    uint32_t st = stamp_of((int)p->clut_x, nc, (int)p->clut_y, 1);
#ifdef DC_PVR_TEXSTAT
    clut_count(eight, key);
#endif
    int victim = -1;
    for (int i = 0; i < nb; i++) {
        BankEnt* b = &banks[i];
        if (b->key == key && b->stamp == st) { b->used = s_frame; return eight ? i + BANK8_FIRST : i; }
        if (s_frame - b->used >= 2 && (victim < 0 || b->used < banks[victim].used)) victim = i;
    }
    if (victim < 0) return -1;
    const uint16_t* cl = &g_dc_vram[(p->clut_y & 511) << 10];
    const uint32_t base = eight ? (uint32_t)(victim + BANK8_FIRST) * 256 : (uint32_t)victim * 16;
    for (int i = 0; i < nc; i++) pvr_set_pal_entry(base + (uint32_t)i, to1555(cl[(p->clut_x + (uint32_t)i) & 1023]));
    banks[victim].key = key; banks[victim].stamp = st; banks[victim].used = s_frame;
    if (eight) victim += BANK8_FIRST;
#ifdef DC_PVR_TEXSTAT
    s_pstat[1]++;
#endif
    return victim;
}

static pvr_ptr_t page4_for(const DcPrim* p)
{
    const int eight = p->depth == 1, np = eight ? s_npage8 : s_npage;
    PageEnt* pages = eight ? s_page8 : s_page;
    uint32_t key = (p->tp_x >> 6) << 1 | (p->tp_y >> 8);
    uint32_t st = stamp_of((int)p->tp_x, eight ? 128 : 64, (int)p->tp_y, 256);
    int victim = -1;
    const int bx = (int)((p->tp_x & 1023) >> 6), by0 = (int)((p->tp_y & 511) >> 4);
    for (int i = 0; i < np; i++) {
        PageEnt* e = &pages[i];
        if (e->key == key) {
            if (e->stamp == st) { e->used = s_frame; return e->tex; }
            if (!eight) {
                /* 4-bit page (one 64-wide block column): redo only the
                   16-row bands written since, in place */
                uint16_t* t = (uint16_t*)e->tex;
                for (int b = 0; b < 16; b++) {
                    uint32_t g = g_dc_vram_gen[(by0 + b) & 31][bx];
                    if (g == e->gen[b]) continue;
                    e->gen[b] = g;
                    for (int y = b * 16; y < b * 16 + 16; y += 2) {
                        const uint16_t* r0 = &g_dc_vram[((p->tp_y + (uint32_t)y) & 511) << 10];
                        const uint16_t* r1 = &g_dc_vram[((p->tp_y + (uint32_t)y + 1) & 511) << 10];
                        uint32_t ty = s_twid[y >> 1];
                        for (int x = 0; x < 256; x += 4) {
                            uint32_t w0 = r0[(p->tp_x + (uint32_t)(x >> 2)) & 1023], w1 = r1[(p->tp_x + (uint32_t)(x >> 2)) & 1023];
                            t[ty | (uint32_t)s_twid[x >> 1] << 1] =
                                (uint16_t)((w0 & 0xF) | (w1 & 0xF) << 4 | (w0 & 0xF0) << 4 | (w1 & 0xF0) << 8);
                            t[ty | (uint32_t)s_twid[(x >> 1) + 1] << 1] =
                                (uint16_t)((w0 >> 8 & 0xF) | (w1 >> 8 & 0xF) << 4 | (w0 >> 12 & 0xF) << 8 | (w1 >> 12 & 0xF) << 12);
                        }
                    }
#ifdef DC_PVR_TEXSTAT
                    s_pstat[2]++;
#endif
                }
                e->stamp = st; e->used = s_frame;
                return e->tex;
            }
            if (s_frame - e->used >= 2) { victim = i; break; }
            continue;                   /* changed but still in use: another slot */
        }
        if (s_frame - e->used >= 2 && (victim < 0 || e->used < pages[victim].used)) victim = i;
    }
    if (victim < 0) return 0;
    uint16_t* o = s_conv;
    if (eight) {
        /* twiddled 8bpp: each 16-bit word is texels (x,y) and (x,y+1) */
        for (int y = 0; y < 256; y += 2) {
            const uint16_t* r0 = &g_dc_vram[((p->tp_y + (uint32_t)y) & 511) << 10];
            const uint16_t* r1 = &g_dc_vram[((p->tp_y + (uint32_t)y + 1) & 511) << 10];
            uint32_t ty = (uint32_t)s_twid[y >> 1] << 1;
            for (int x = 0; x < 256; x += 2) {
                uint32_t w0 = r0[(p->tp_x + (uint32_t)(x >> 1)) & 1023], w1 = r1[(p->tp_x + (uint32_t)(x >> 1)) & 1023];
                o[ty | s_twid[x]] = (uint16_t)((w0 & 0xFF) | (w1 & 0xFF) << 8);
                o[ty | s_twid[x + 1]] = (uint16_t)((w0 >> 8) | (w1 & 0xFF00));
            }
        }
        PageEnt* e = &pages[victim];
        uint32_t* d = (uint32_t*)e->tex;
        const uint32_t* src = (const uint32_t*)s_conv;
        for (int i = 0; i < 256 * 256 / 4; i += 4) { d[i] = src[i]; d[i + 1] = src[i + 1]; d[i + 2] = src[i + 2]; d[i + 3] = src[i + 3]; }
        e->key = key; e->stamp = st; e->used = s_frame;
#ifdef DC_PVR_TEXSTAT
        s_pstat[0]++;
#endif
        return e->tex;
    }
    /* twiddled 4bpp: each 16-bit word is a 2x2 block, texels (x,y) (x,y+1)
       (x+1,y) (x+1,y+1) from the low nibble up */
    for (int y = 0; y < 256; y += 2) {
        const uint16_t* r0 = &g_dc_vram[((p->tp_y + (uint32_t)y) & 511) << 10];
        const uint16_t* r1 = &g_dc_vram[((p->tp_y + (uint32_t)y + 1) & 511) << 10];
        uint32_t ty = s_twid[y >> 1];
        for (int x = 0; x < 256; x += 4) {
            uint32_t w0 = r0[(p->tp_x + (uint32_t)(x >> 2)) & 1023], w1 = r1[(p->tp_x + (uint32_t)(x >> 2)) & 1023];
            o[ty | (uint32_t)s_twid[x >> 1] << 1] =
                (uint16_t)((w0 & 0xF) | (w1 & 0xF) << 4 | (w0 & 0xF0) << 4 | (w1 & 0xF0) << 8);
            o[ty | (uint32_t)s_twid[(x >> 1) + 1] << 1] =
                (uint16_t)((w0 >> 8 & 0xF) | (w1 >> 8 & 0xF) << 4 | (w0 >> 12 & 0xF) << 8 | (w1 >> 12 & 0xF) << 12);
        }
    }
    PageEnt* e = &s_page[victim];
    for (int b = 0; b < 16; b++) e->gen[b] = g_dc_vram_gen[(by0 + b) & 31][bx];
    uint32_t* d = (uint32_t*)e->tex;
    const uint32_t* src = (const uint32_t*)s_conv;
    for (int i = 0; i < 256 * 256 / 8; i += 4) { d[i] = src[i]; d[i + 1] = src[i + 1]; d[i + 2] = src[i + 2]; d[i + 3] = src[i + 3]; }
    e->key = key; e->stamp = st; e->used = s_frame;
#ifdef DC_PVR_TEXSTAT
    s_pstat[0]++;
#endif
    return e->tex;
}

/* ---- scenes ------------------------------------------------------------- */
static void blit(int x, int y, int w, int h);

/* Light frames (menus, text screens) are also drawn in software, so VRAM keeps
   a true copy of their frame buffer: games that draw on top of the previous
   picture without clearing it (the title menu) then get that picture back
   as the background of the next frame. Heavy 3D frames redraw everything
   and are left to the PowerVR alone. */
#define SHADOW_MAX 96
/* Frames are drawn in software too only while they stay light: a frame
   after a heavy one is taken for heavy from the start. */
static int s_shadow = 1;
static unsigned s_last_prims;
/* VRAM blocks (64x16) where the PowerVR drew something software did not:
   VRAM's copy is wrong there. Software writes (fills, uploads, copies)
   make a block right again. */
static uint8_t s_stale[32][16];

static void mark_blocks(int x, int y, int w, int h, uint8_t v)
{
    if (w <= 0 || h <= 0) return;
    if (w > 1024) w = 1024;
    if (h > 512) h = 512;
    int bx0 = (x & 1023) >> 6, bx1 = ((x + w - 1) & 1023) >> 6;
    int by0 = (y & 511) >> 4, by1 = ((y + h - 1) & 511) >> 4;
    for (int by = by0;; by = (by + 1) & 31) {
        for (int bx = bx0;; bx = (bx + 1) & 15) {
            s_stale[by][bx] = v;
            if (bx == bx1) break;
        }
        if (by == by1) break;
    }
}

static int vram_true(int x, int y, int w, int h)
{
    int bx0 = (x & 1023) >> 6, bx1 = ((x + w - 1) & 1023) >> 6;
    int by0 = (y & 511) >> 4, by1 = ((y + h - 1) & 511) >> 4;
    for (int by = by0;; by = (by + 1) & 31) {
        for (int bx = bx0;; bx = (bx + 1) & 15) {
            if (s_stale[by][bx]) return 0;
            if (bx == bx1) break;
        }
        if (by == by1) break;
    }
    return 1;
}

/* Open a scene for the frame at (ox, oy). A frame that is not cleared first
   starts from VRAM's copy of the area when that copy is up to date. */
static void scene_open_ex(int ox, int oy, int cleared);
static void scene_open(int ox, int oy) { scene_open_ex(ox, oy, 1); }

/* Where the display-sized picture sits inside a drawing area: the display's
   own position when the area covers it (a drawing area wider than the
   picture, as on Xenogears' field), else the nearest place inside the area
   (the other buffer of a double-buffered pair). */
static int fit_axis(int disp, int size, int a0, int a1)
{
    if (size <= 0 || a1 - a0 + 1 <= size) return a0;
    if (disp < a0) return a0;
    if (disp + size - 1 > a1) return a1 - size + 1;
    return disp;
}
static void scene_open_area(int x0, int y0, int x1, int y1)
{
    DcGpuDisplay d; dc_gpu_display(&d);
    scene_open_ex(fit_axis(d.x, d.w, x0, x1), fit_axis(d.y, d.h, y0, y1), 1);
}
static void scene_open_ex(int ox, int oy, int cleared)
{
    pvr_wait_ready();
    s_rtt_scene = s_rtt && s_rtt_want;
    if (s_rtt_scene) pvr_scene_begin_rtt(s_rtt, 640, 480, RTT_W);
    else pvr_scene_begin();
    s_rtt_split = 0; s_rtt_hit = 0;
    s_shadow = s_last_prims <= SHADOW_MAX;
    pvr_list_begin(PVR_LIST_TR_POLY);
    s_scene = 1; s_hdr_key = ~0u; s_frame++; s_prims = 0; s_vblanks_open = 0;
    s_ox = ox; s_oy = oy;
#ifdef DC_PVR_TRACE
    if (oy == DC_PVR_TRACE && s_trace_left) { s_trace_on = 1; s_trace_left--; }
#endif
    DcGpuDisplay d; dc_gpu_display(&d);
    s_sx = 640.0f / (float)(d.w > 0 ? d.w : 320);
    s_sy = 480.0f / (float)(d.h > 0 ? d.h : 240);
    int bw = d.w > 0 ? d.w : 320, bh = d.h > 0 ? d.h : 240;
    int ok = !cleared && vram_true(ox, oy, bw, bh);
    s_bg = cleared ? 0 : ok ? 2 : 1;
    if (ok) blit(ox, oy, d.w > 0 ? d.w : 320, d.h > 0 ? d.h : 240);
}

static void send_header_ex(const DcPrim* p, pvr_ptr_t tex, uint32_t fmt, int tw, int th);

/* Finish the part of the frame rendered into s_rtt and start the visible
   scene from it. */
static void rtt_split(void)
{
    pvr_list_finish();
    pvr_scene_finish();
    pvr_wait_ready();
    pvr_scene_begin();
    pvr_list_begin(PVR_LIST_TR_POLY);
    s_rtt_scene = 0; s_rtt_split = 1; s_hdr_key = ~0u;
    DcPrim p;
    memset(&p, 0, sizeof p);
    p.tex = 1; p.raw = 1;
    send_header_ex(&p, s_rtt, PVR_TXRFMT_RGB565 | PVR_TXRFMT_NONTWIDDLED, RTT_W, RTT_H);
    const int cx[4] = {0, 1, 0, 1}, cy[4] = {0, 0, 1, 1};
    pvr_vertex_t vx;
    for (int k = 0; k < 4; k++) {
        vx.flags = k == 3 ? PVR_CMD_VERTEX_EOL : PVR_CMD_VERTEX;
        vx.x = (float)(cx[k] * 640); vx.y = (float)(cy[k] * 480); vx.z = 1.0f;
        vx.u = (float)(cx[k] * 640) * (1.0f / RTT_W);
        vx.v = (float)(cy[k] * 480) * (1.0f / RTT_H);
        vx.argb = 0xFFFFFFFFu; vx.oargb = 0;
        pvr_prim(&vx, sizeof vx);
    }
}

static void scene_close(void)
{
    if (!s_scene) return;
    if (s_rtt_scene) rtt_split();       /* nothing read the frame after all */
    if (s_rtt_hit) s_rtt_idle = 0;
    else if (s_rtt_want && ++s_rtt_idle > 30) s_rtt_want = 0;
    /* A light frame was drawn in software too, exactly (mask bits,
       subtraction, frame feedback): show that picture over the PowerVR's. */
    s_last_prims = s_prims;
    if (s_shadow && s_prims && s_prims <= SHADOW_MAX) {
        /* the display area as the game now shows it (closing follows a
           flip), mapped to the full screen */
        DcGpuDisplay d; dc_gpu_display(&d);
        if (d.enabled && !d.depth24 && d.w > 0 && d.h > 0) {
            s_ox = d.x; s_oy = d.y;
            blit(d.x, d.y, d.w, d.h);
        }
        s_bg = 3;
    }
#ifdef DC_PVR_SCENELOG
    { DcGpuDisplay d_; dc_gpu_display(&d_); char b[128]; snprintf(b, sizeof b, "[sc %u] at %d,%d prims %u bg %d disp %d,%d %dx%d en%d rtt%d\n", s_frame, s_ox, s_oy, s_prims, s_bg, d_.x, d_.y, d_.w, d_.h, d_.enabled, s_rtt_split); dbgio_write_str(b); }
#endif
    pvr_list_finish();
    pvr_scene_finish();
    s_scene = 0;
    g_dc_pvr_stat[3]++;
}

/* Show the VRAM display area as it is (software frames). */
static void show_vram_display(void)
{
    DcGpuDisplay d; dc_gpu_display(&d);
    if (!d.enabled || d.depth24) return;
    scene_open_ex(d.x, d.y, 1);
    blit(d.x, d.y, d.w, d.h);
    scene_close();
}

#ifdef DC_PVR_SCENELOG
static int s_rf_n, s_rf_dymin = 9999, s_rf_dymax = -9999, s_rf_dxmin = 9999, s_rf_dxmax = -9999, s_rf_ymin = 9999, s_rf_ymax = -9999;
#endif
static unsigned s_vbl_since_flip = ~0u >> 1;
void dc_gpu_hw_flip(void)
{
    s_vbl_since_flip = 0;
#ifdef DC_PVR_SCENELOG
    { static int lines; static unsigned flips; flips++;
      if (s_rf_n && lines < 400) { lines++; char b[128]; snprintf(b, sizeof b, "[rfs %u] n=%d dx=%d..%d dy=%d..%d y=%d..%d\n", flips, s_rf_n, s_rf_dxmin, s_rf_dxmax, s_rf_dymin, s_rf_dymax, s_rf_ymin, s_rf_ymax); dbgio_write_str(b); }
      s_rf_n = 0; s_rf_dymin = s_rf_dxmin = s_rf_ymin = 9999; s_rf_dymax = s_rf_dxmax = s_rf_ymax = -9999; }
#endif
    { DcGpuDisplay d; dc_gpu_display(&d); TRACE("[tr %u] flip disp=%d,%d\n", s_frame, d.x, d.y); }
    if (!g_dc_pvr_on) return;
    scene_close();
    if (s_soft_frames) { s_soft_frames--; show_vram_display(); }
}

/* Called every VBlank: a game that draws without flipping still gets
   its picture shown. */
void dc_gpu_pvr_vblank(void)
{
    static unsigned soft_vbl;
    /* only for games that do not flip: a slow frame of a flipping game is
       still being drawn */
    if (s_vbl_since_flip < 1000) s_vbl_since_flip++;
#ifdef DC_PVR_TEXSTAT
    { static unsigned n; if (++n >= 600) { n = 0; char b[200];
      snprintf(b, sizeof b, "[tex] 4bit %u+%u 8bit %u+%u 15bit %u+%u vram %u+%u (new+refresh) pal pages %u banks %u fallback/bands %u (page4 %u page8 %u bank4 %u bank8 %u)\n",
               s_tstat[0][0], s_tstat[0][1], s_tstat[1][0], s_tstat[1][1], s_tstat[2][0], s_tstat[2][1], s_tstat[3][0], s_tstat[3][1],
               s_pstat[0], s_pstat[1], s_pstat[2], s_pstat[3], s_pstat[4], s_pstat[5], s_pstat[6]);
      dbgio_write_str(b);
      snprintf(b, sizeof b, "[clut] max distinct per frame: 4-bit %d 8-bit %d\n", s_cl_max[0], s_cl_max[1]);
      s_cl_max[0] = s_cl_max[1] = 0;
      dbgio_write_str(b); memset(s_tstat, 0, sizeof s_tstat); memset(s_pstat, 0, sizeof s_pstat); } }
#endif
    if (s_scene && s_prims && ++s_vblanks_open >= 4 && s_vbl_since_flip > 120) scene_close();
    /* software frames in a game that does not flip */
    if (s_soft_frames && !s_scene && ++soft_vbl >= 4) { soft_vbl = 0; show_vram_display(); }
}

/* Is this drawing area the frame buffer being built? */
static int frame_area(int x0, int y0, int x1, int y1)
{
    DcGpuDisplay d; dc_gpu_display(&d);
    if (!d.enabled || d.depth24) return 0;
    int w = x1 - x0 + 1, h = y1 - y0 + 1;
    return w * 4 >= d.w * 3 && h * 4 >= d.h * 3 && w * 4 <= d.w * 5 && h * 4 <= d.h * 5;
}

/* ---- primitives --------------------------------------------------------- */
static void send_header(const DcPrim* p, pvr_ptr_t tex)
{
    int tw = 256, th = 256;
    if (p->depth != 3) { tw = win_w(p); th = win_h(p); }
    send_header_ex(p, tex, PVR_TXRFMT_ARGB1555 | PVR_TXRFMT_NONTWIDDLED, tw, th);
}
static void send_header_ex(const DcPrim* p, pvr_ptr_t tex, uint32_t fmt, int tw, int th)
{
    uint32_t key = (uint32_t)(uintptr_t)tex ^ (uint32_t)(p->tex << 1 | p->semi << 2 | p->semi_mode << 3 | p->gouraud << 5) ^
                   (uint32_t)(__builtin_ctz(tw) << 6 | __builtin_ctz(th) << 10) ^ (fmt >> 16) * 0x9E37u;
    if (key == s_hdr_key) return;
    s_hdr_key = key;
    pvr_poly_cxt_t c;
    if (p->tex) {
        pvr_poly_cxt_txr(&c, PVR_LIST_TR_POLY, fmt, tw, th, tex, PVR_FILTER_NONE);
        c.txr.env = PVR_TXRENV_MODULATEALPHA;
    } else {
        pvr_poly_cxt_col(&c, PVR_LIST_TR_POLY);
    }
    c.gen.shading = p->gouraud;
    c.gen.culling = PVR_CULLING_NONE;
    c.depth.comparison = PVR_DEPTHCMP_ALWAYS;
    c.depth.write = false;
    c.blend.src = PVR_BLEND_SRCALPHA; c.blend.dst = PVR_BLEND_INVSRCALPHA;
    if (p->semi) {
        switch (p->semi_mode) {
        case 1: case 3: c.blend.src = PVR_BLEND_SRCALPHA; c.blend.dst = PVR_BLEND_ONE; break;
        case 2: c.blend.src = PVR_BLEND_ZERO; c.blend.dst = PVR_BLEND_INVDESTCOLOR; break;  /* dst*(1-src) */
        default: break;
        }
    }
    pvr_poly_hdr_t h;
    pvr_poly_compile(&h, &c);
    pvr_prim(&h, sizeof h);
}

static inline uint32_t colour(const DcPrim* p, const DcPrimVtx* v, uint32_t a)
{
    int r = v->r, g = v->g, b = v->b;
    if (p->tex) {
        if (p->raw) r = g = b = 255;
        else { r = r >= 128 ? 255 : r * 2; g = g >= 128 ? 255 : g * 2; b = b >= 128 ? 255 : b * 2; }
    }
    return a << 24 | (uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b;
}


static int overlaps(int ax, int ay, int aw, int ah, int bx, int by, int bw, int bh)
{
    return ax < bx + bw && bx < ax + aw && ay < by + bh && by < ay + ah;
}

/* The VRAM rectangle a primitive's texels come from. */
static void tex_rect(const DcPrim* p, int* x, int* y, int* w, int* h)
{
    int u0 = p->v[0].u, u1 = u0, v0 = p->v[0].v, v1 = v0;
    for (int k = 1; k < p->n; k++) {
        u0 = p->v[k].u < u0 ? p->v[k].u : u0; u1 = p->v[k].u > u1 ? p->v[k].u : u1;
        v0 = p->v[k].v < v0 ? p->v[k].v : v0; v1 = p->v[k].v > v1 ? p->v[k].v : v1;
    }
    if (win_w(p) < 256) { u0 = p->tw_u0; u1 = p->tw_u0 + win_w(p) - 1; }
    if (win_h(p) < 256) { v0 = p->tw_v0; v1 = p->tw_v0 + win_h(p) - 1; }
    u0 = u0 < 0 ? 0 : u0; u1 = u1 > 255 ? 255 : u1; v0 = v0 < 0 ? 0 : v0; v1 = v1 > 255 ? 255 : v1;
    int sh = p->depth == 0 ? 2 : p->depth == 1 ? 1 : 0;
    *x = (int)p->tp_x + (u0 >> sh); *w = (u1 >> sh) - (u0 >> sh) + 1;
    *y = (int)p->tp_y + v0; *h = v1 - v0 + 1;
}

/* Does this primitive sample a frame buffer (the displayed one or the one
   being drawn)? */
static int reads_frame(const DcPrim* p)
{
    if (!p->tex) return 0;
    DcGpuDisplay d; dc_gpu_display(&d);
    int tx, ty, tw, th; tex_rect(p, &tx, &ty, &tw, &th);
    /* the displayed buffer is as tall as the one being drawn (the display
       range often shows a few more lines) */
    int fh = p->clip_y1 - p->clip_y0 + 1 < d.h ? p->clip_y1 - p->clip_y0 + 1 : d.h;
    return overlaps(tx, ty, tw, th, d.x, d.y, d.w, fh) ||
           overlaps(tx, ty, tw, th, p->clip_x0, p->clip_y0,
                    p->clip_x1 - p->clip_x0 + 1, p->clip_y1 - p->clip_y0 + 1);
}

/* Does this primitive texture from the frame being built? */
static int reads_scene(const DcPrim* p)
{
    if (!p->tex || !s_scene) return 0;
    DcGpuDisplay d; dc_gpu_display(&d);
    int tx, ty, tw, th; tex_rect(p, &tx, &ty, &tw, &th);
    int fh = p->clip_y1 - p->clip_y0 + 1 < d.h ? p->clip_y1 - p->clip_y0 + 1 : d.h;
    return overlaps(tx, ty, tw, th, s_ox, s_oy, d.w, fh);
}

int dc_gpu_hw_prim(const DcPrim* p)
{
    if (!g_dc_pvr_on) return 0;
    int rtt = 0;
    if (s_rtt && !s_soft_frames && reads_scene(p) && frame_area(p->clip_x0, p->clip_y0, p->clip_x1, p->clip_y1)) {
        s_rtt_hit = 1;
        if (!s_rtt_want) {
            /* first time: from the next frame on, render to texture; this
               one loses the effect */
            s_rtt_want = 1; s_rtt_idle = 0;
            dc_host_log("[pvr] frame reads itself: render to texture\n");
        }
        if (s_rtt_scene) rtt_split();
        if (!s_rtt_split) { g_dc_pvr_stat[1]++; return 1; }
        rtt = 1;
    }
    if (!rtt && reads_frame(p) && frame_area(p->clip_x0, p->clip_y0, p->clip_x1, p->clip_y1)) {
        if (!s_soft_frames) dc_host_log("[pvr] frame buffer used as a texture: software frames\n");
#ifdef DC_PVR_SCENELOG
        { int dy = p->v[0].y - ((int)p->tp_y + p->v[0].u * 0 + p->v[0].v), dx = p->v[0].x - ((int)p->tp_x + p->v[0].u);
          s_rf_n++; if (dy < s_rf_dymin) s_rf_dymin = dy; if (dy > s_rf_dymax) s_rf_dymax = dy;
          if (dx < s_rf_dxmin) s_rf_dxmin = dx; if (dx > s_rf_dxmax) s_rf_dxmax = dx;
          if (p->v[0].y < s_rf_ymin) s_rf_ymin = p->v[0].y; if (p->v[0].y > s_rf_ymax) s_rf_ymax = p->v[0].y; }
        { static int n; if (n < 60) { n++; DcGpuDisplay d; dc_gpu_display(&d); char b[160];
          snprintf(b, sizeof b, "[rf] tp=%u,%u d=%d clut=%u,%u n=%d semi=%d/%d clip=%d,%d-%d,%d disp=%d,%d %dx%d uv0=%d,%d v0=%d,%d\n",
                   p->tp_x, p->tp_y, p->depth, p->clut_x, p->clut_y, p->n, p->semi, p->semi_mode, p->clip_x0, p->clip_y0, p->clip_x1, p->clip_y1,
                   d.x, d.y, d.w, d.h, p->v[0].u, p->v[0].v, p->v[0].x, p->v[0].y); dbgio_write_str(b); } }
#endif
        s_soft_frames = 60;
        if (s_scene) scene_close();
    }
    TRACE("[tr %u] prim n=%d tex=%d d=%d tp=%u,%u clut=%u,%u semi=%d/%d clip=%d,%d-%d,%d v0=%d,%d c=%02x%02x%02x fa=%d\n", s_frame,
          p->n, p->tex, p->depth, p->tp_x, p->tp_y, p->clut_x, p->clut_y, p->semi, p->semi_mode,
          p->clip_x0, p->clip_y0, p->clip_x1, p->clip_y1, p->v[0].x, p->v[0].y, p->v[0].r, p->v[0].g, p->v[0].b,
          frame_area(p->clip_x0, p->clip_y0, p->clip_x1, p->clip_y1));
    if (s_soft_frames) { g_dc_pvr_stat[1]++; return 0; }
    if (!frame_area(p->clip_x0, p->clip_y0, p->clip_x1, p->clip_y1)) { g_dc_pvr_stat[1]++; return 0; }
    g_dc_pvr_stat[0]++;
    if (!s_scene) scene_open_area(p->clip_x0, p->clip_y0, p->clip_x1, p->clip_y1);
    pvr_ptr_t tex = 0;
    if (rtt) send_header_ex(p, s_rtt, PVR_TXRFMT_RGB565 | PVR_TXRFMT_NONTWIDDLED, RTT_W, RTT_H);
    else {
        int done = 0;
        if (p->tex && p->depth <= 1 && win_w(p) == 256 && win_h(p) == 256 && (p->depth ? s_npage8 : s_npage)) {
            pvr_ptr_t pg = page4_for(p);
            int bank = pg ? bank_for(p) : -1;
#ifdef DC_PVR_TEXSTAT
            if (!pg) s_pstat[3 + p->depth]++; else if (bank < 0) s_pstat[5 + p->depth]++;
#endif
            if (bank >= 0) {
                send_header_ex(p, pg, p->depth ? PVR_TXRFMT_PAL8BPP | PVR_TXRFMT_8BPP_PAL(bank) | PVR_TXRFMT_TWIDDLED
                                               : PVR_TXRFMT_PAL4BPP | PVR_TXRFMT_4BPP_PAL(bank) | PVR_TXRFMT_TWIDDLED, 256, 256);
                done = 1;
            }
#ifdef DC_PVR_TEXSTAT
            else s_pstat[2]++;
#endif
        }
        if (!done) { tex = p->tex ? texture_for(p) : 0; send_header(p, tex); }
    }
    uint32_t a = 255;
    if (p->semi) a = p->semi_mode == 0 ? 128 : p->semi_mode == 3 ? 64 : 255;
    pvr_vertex_t vx;
    for (int k = 0; k < p->n; k++) {
        const DcPrimVtx* v = &p->v[k];
        vx.flags = k == p->n - 1 ? PVR_CMD_VERTEX_EOL : PVR_CMD_VERTEX;
        vx.x = (float)(v->x - s_ox) * s_sx;
        vx.y = (float)(v->y - s_oy) * s_sy;
        vx.z = 1.0f;
        if (rtt) {
            vx.u = (float)((int)p->tp_x + v->u - s_ox) * s_sx * (1.0f / RTT_W);
            vx.v = (float)((int)p->tp_y + v->v - s_oy) * s_sy * (1.0f / RTT_H);
        } else {
            vx.u = (float)v->u * (1.0f / (float)win_w(p));
            vx.v = (float)v->v * (1.0f / (float)win_h(p));
        }
        vx.argb = colour(p, v, a);
        vx.oargb = 0;
        pvr_prim(&vx, sizeof vx);
    }
#ifdef DC_PVR_SHADOW_ALL
    ++s_prims; return 2;
#endif
    if (++s_prims <= SHADOW_MAX && s_shadow) return 2;
    int minx = p->v[0].x, maxx = minx, miny = p->v[0].y, maxy = miny;
    for (int k = 1; k < p->n; k++) {
        minx = p->v[k].x < minx ? p->v[k].x : minx; maxx = p->v[k].x > maxx ? p->v[k].x : maxx;
        miny = p->v[k].y < miny ? p->v[k].y : miny; maxy = p->v[k].y > maxy ? p->v[k].y : maxy;
    }
    minx = minx < p->clip_x0 ? p->clip_x0 : minx; maxx = maxx > p->clip_x1 ? p->clip_x1 : maxx;
    miny = miny < p->clip_y0 ? p->clip_y0 : miny; maxy = maxy > p->clip_y1 ? p->clip_y1 : maxy;
    if (minx <= maxx && miny <= maxy) mark_blocks(minx, miny, maxx - minx + 1, maxy - miny + 1, 1);
    return 1;
}

void dc_gpu_hw_fill(int x, int y, int w, int h, uint16_t c)
{
    mark_blocks(x, y, w, h, 0);
    TRACE("[tr %u] fill %d,%d %dx%d c=%04x scene=%d\n", s_frame, x, y, w, h, c, s_scene);
    if (!g_dc_pvr_on || s_soft_frames) return;
    if (!s_scene) {
        if (!frame_area(x, y, x + w - 1, y + h - 1)) return;
        scene_open_area(x, y, x + w - 1, y + h - 1);
    } else {
        /* a smaller fill (a window background...) inside the frame */
        DcGpuDisplay d; dc_gpu_display(&d);
        if (x >= s_ox + d.w || y >= s_oy + d.h || x + w <= s_ox || y + h <= s_oy) return;
    }
    DcPrim p;
    memset(&p, 0, sizeof p);
    p.n = 4;
    const int cx[4] = {0, 1, 0, 1}, cy[4] = {0, 0, 1, 1};
    int r = (c & 31) << 3, g = ((c >> 5) & 31) << 3, b = ((c >> 10) & 31) << 3;
    for (int k = 0; k < 4; k++) {
        p.v[k].x = x + cx[k] * w; p.v[k].y = y + cy[k] * h;
        p.v[k].r = r; p.v[k].g = g; p.v[k].b = b;
    }
    send_header(&p, 0);
    pvr_vertex_t vx;
    for (int k = 0; k < 4; k++) {
        vx.flags = k == 3 ? PVR_CMD_VERTEX_EOL : PVR_CMD_VERTEX;
        vx.x = (float)(p.v[k].x - s_ox) * s_sx;
        vx.y = (float)(p.v[k].y - s_oy) * s_sy;
        vx.z = 1.0f; vx.u = vx.v = 0;
        vx.argb = colour(&p, &p.v[k], 255);
        vx.oargb = 0;
        pvr_prim(&vx, sizeof vx);
    }
}

/* ---- copies and uploads into the frame -------------------------------- */
/* Draw a VRAM rectangle as opaque textured quads, 256x256 at a time. */
static void blit(int x, int y, int w, int h)
{
    DcPrim p;
    memset(&p, 0, sizeof p);
    p.n = 4; p.tex = 1; p.raw = 1; p.depth = 3;
    for (int ty = 0; ty < h; ty += 256)
        for (int tx = 0; tx < w; tx += 256) {
            int tw = w - tx < 256 ? w - tx : 256, th = h - ty < 256 ? h - ty : 256;
            p.tp_x = (uint32_t)((x + tx) & 1023); p.tp_y = (uint32_t)((y + ty) & 511);
            pvr_ptr_t tex = texture_for(&p);
            send_header(&p, tex);
            const int cx[4] = {0, 1, 0, 1}, cy[4] = {0, 0, 1, 1};
            pvr_vertex_t vx;
            for (int k = 0; k < 4; k++) {
                vx.flags = k == 3 ? PVR_CMD_VERTEX_EOL : PVR_CMD_VERTEX;
                vx.x = (float)(x + tx + cx[k] * tw - s_ox) * s_sx;
                vx.y = (float)(y + ty + cy[k] * th - s_oy) * s_sy;
                vx.z = 1.0f;
                vx.u = (float)(cx[k] * tw) * (1.0f / 256.0f);
                vx.v = (float)(cy[k] * th) * (1.0f / 256.0f);
                vx.argb = 0xFFFFFFFFu; vx.oargb = 0;
                pvr_prim(&vx, sizeof vx);
            }
        }
}

void dc_gpu_hw_blit(int x, int y, int w, int h, int cx0, int cy0, int cx1, int cy1)
{
    mark_blocks(x, y, w, h, 0);
    TRACE("[tr %u] blit %d,%d %dx%d clip=%d,%d-%d,%d\n", s_frame, x, y, w, h, cx0, cy0, cx1, cy1);
    if (!g_dc_pvr_on || s_soft_frames || !frame_area(cx0, cy0, cx1, cy1)) return;
    /* only the part inside the drawing area */
    int x0 = x > cx0 ? x : cx0, y0 = y > cy0 ? y : cy0;
    int x1 = x + w - 1 < cx1 ? x + w - 1 : cx1, y1 = y + h - 1 < cy1 ? y + h - 1 : cy1;
    if (x0 > x1 || y0 > y1) return;
    /* uploads between frames (new textures, often next to the frame) do not
       start a frame of their own */
    if (s_scene) blit(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
}

/* ---- init --------------------------------------------------------------- */
int dc_gpu_pvr_init(void)
{
    pvr_init_params_t params;
    memset(&params, 0, sizeof params);
    params.opb_sizes[PVR_LIST_TR_POLY] = PVR_BINSIZE_32;
    params.vertex_buf_size = 768 * 1024;   /* x2, double-buffered */
    params.autosort_disabled = 1;       /* translucent list in submission order */
    params.opb_overflow_count = 3;
    if (pvr_init(&params) < 0) { dc_host_log("[pvr] pvr_init failed\n"); return -1; }
    pvr_set_bg_color(0, 0, 0);
    /* keep 256 KB free for the PowerVR's own use */
    pvr_ptr_t reserve = pvr_mem_malloc(256 * 1024);
    s_rtt = pvr_mem_malloc(RTT_W * RTT_H * 2);
    pvr_set_pal_format(PVR_PAL_ARGB1555);
    for (int i = 0; i < 256; i++) {
        uint16_t t = 0;
        for (int b = 0; b < 8; b++) if (i >> b & 1) t |= (uint16_t)(1 << (2 * b));
        s_twid[i] = t;
    }
    for (int i = 0; i < NBANK4; i++) { s_bank[i].key = ~0u; s_bank[i].used = 0; }
    for (int i = 0; i < NBANK8; i++) { s_bank8[i].key = ~0u; s_bank8[i].used = 0; }
    for (s_npage = 0; s_npage < NPAGE; s_npage++) {
        if (pvr_mem_available() < 2 * 1024 * 1024) break;     /* leave room for the others */
        s_page[s_npage].tex = pvr_mem_malloc(256 * 256 / 2);
        if (!s_page[s_npage].tex) break;
        s_page[s_npage].key = ~0u; s_page[s_npage].used = 0;
    }
    for (s_npage8 = 0; s_npage8 < NPAGE8; s_npage8++) {
        if (pvr_mem_available() < 1536 * 1024) break;
        s_page8[s_npage8].tex = pvr_mem_malloc(256 * 256);
        if (!s_page8[s_npage8].tex) break;
        s_page8[s_npage8].key = ~0u; s_page8[s_npage8].used = 0;
    }
    for (s_ntex = 0; s_ntex < NTEX; s_ntex++) {
        if (pvr_mem_available() < 256 * 256 * 2 + 4096) break;
        s_tex[s_ntex].tex = pvr_mem_malloc(256 * 256 * 2);
        if (!s_tex[s_ntex].tex) break;
        s_tex[s_ntex].key = ~0u; s_tex[s_ntex].used = 0;
    }
    if (reserve) pvr_mem_free(reserve);
    char b[80]; snprintf(b, sizeof b, "[pvr] %d texture slots, %d 4-bit pages, %d 8-bit pages\n", s_ntex, s_npage, s_npage8);
    dc_host_log(b);
    if (s_ntex < 8) return -1;
    g_dc_pvr_on = 1;
    return 0;
}
