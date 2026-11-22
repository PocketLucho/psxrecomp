/* dc_gpu.c - PS1 GPU for the Dreamcast runtime: software rasteriser into a
 * 1024x512 15-bit VRAM.
 *
 * First version, written for correctness and to see the game's frames: flat,
 * Gouraud and textured (4/8/15-bit, CLUT) polygons, rectangles and lines,
 * semi-transparency, mask bit, texture window, fills and VRAM transfers.
 * No dithering. On hardware the PowerVR2 renderer replaces the drawing part;
 * the command decoding and VRAM transfers stay.
 */
#include "dc_gpu.h"
#include <string.h>

uint16_t g_dc_vram[1024 * 512];

static struct {
    /* GP0 state */
    uint32_t texpage;                 /* E1 bits 0..13 */
    uint32_t tw_mask_x, tw_mask_y, tw_off_x, tw_off_y;
    int clip_x0, clip_y0, clip_x1, clip_y1;
    int off_x, off_y;
    int set_mask, check_mask;
    /* GP1 state */
    int display_off;
    uint32_t disp_x, disp_y, disp_mode;
    uint32_t dma_dir;
    /* command assembly */
    uint32_t cmd[16];
    int ncmd, need, polyline;
    /* CPU -> VRAM / VRAM -> CPU transfers */
    int load_active, load_x, load_y, load_w, load_h, load_i, load_n;
    int store_active, store_x, store_y, store_w, store_h, store_i, store_n;
    uint32_t gpuread;
} G;

/* ---------------------------------------------------------------- helpers */
static inline int sext11(uint32_t v) { return (int)((int32_t)(v << 21) >> 21); }
static inline uint16_t rgb15(uint32_t c)
{
    return (uint16_t)(((c >> 3) & 0x1F) | (((c >> 11) & 0x1F) << 5) | (((c >> 19) & 0x1F) << 10));
}
static inline uint16_t* px(int x, int y) { return &g_dc_vram[((y & 511) << 10) | (x & 1023)]; }

static uint16_t blend(uint16_t b, uint16_t f, int mode)
{
    int br = b & 31, bg = (b >> 5) & 31, bb = (b >> 10) & 31;
    int fr = f & 31, fg = (f >> 5) & 31, fb = (f >> 10) & 31;
    int r, g, bl;
    switch (mode) {
    case 0: r = (br + fr) >> 1; g = (bg + fg) >> 1; bl = (bb + fb) >> 1; break;
    case 1: r = br + fr; g = bg + fg; bl = bb + fb; break;
    case 2: r = br - fr; g = bg - fg; bl = bb - fb; break;
    default: r = br + (fr >> 2); g = bg + (fg >> 2); bl = bb + (fb >> 2); break;
    }
    r = r < 0 ? 0 : r > 31 ? 31 : r; g = g < 0 ? 0 : g > 31 ? 31 : g; bl = bl < 0 ? 0 : bl > 31 ? 31 : bl;
    return (uint16_t)(r | (g << 5) | (bl << 10) | (f & 0x8000));
}

/* Write one pixel with mask and optional semi-transparency. */
static inline void plot(int x, int y, uint16_t c, int semi, int mode)
{
    if (x < G.clip_x0 || x > G.clip_x1 || y < G.clip_y0 || y > G.clip_y1) return;
    uint16_t* d = px(x, y);
    if (G.check_mask && (*d & 0x8000)) return;
    if (semi) c = blend(*d, c, mode);
    *d = c | (G.set_mask ? 0x8000 : 0);
}

typedef struct {
    int tex, raw, semi, gouraud;
    uint32_t clut_x, clut_y, tp_x, tp_y, depth, semi_mode;
} Shade;

static void shade_from_texpage(Shade* s, uint32_t tp)
{
    s->tp_x = (tp & 0xF) * 64;
    s->tp_y = ((tp >> 4) & 1) * 256;
    s->semi_mode = (tp >> 5) & 3;
    s->depth = (tp >> 7) & 3;
}

/* Texel at (u, v) with the texture window applied; 0 = transparent. */
static uint16_t texel(const Shade* s, uint32_t u, uint32_t v)
{
    u &= 0xFF; v &= 0xFF;
    u = (u & ~(G.tw_mask_x * 8)) | ((G.tw_off_x & G.tw_mask_x) * 8);
    v = (v & ~(G.tw_mask_y * 8)) | ((G.tw_off_y & G.tw_mask_y) * 8);
    int y = (int)(s->tp_y + v);
    switch (s->depth) {
    case 0: {
        uint16_t w = *px((int)(s->tp_x + (u >> 2)), y);
        return *px((int)(s->clut_x + ((w >> ((u & 3) * 4)) & 0xF)), (int)s->clut_y);
    }
    case 1: {
        uint16_t w = *px((int)(s->tp_x + (u >> 1)), y);
        return *px((int)(s->clut_x + ((w >> ((u & 1) * 8)) & 0xFF)), (int)s->clut_y);
    }
    default:
        return *px((int)(s->tp_x + u), y);
    }
}

/* Final colour of a pixel from interpolated colour r,g,b (8-bit) and uv. */
static inline void shade_pixel(const Shade* s, int x, int y, int r, int g, int b, int u, int v)
{
    if (!s->tex) {
        uint16_t c = (uint16_t)((r >> 3) | ((g >> 3) << 5) | ((b >> 3) << 10));
        plot(x, y, c, s->semi, s->semi_mode);
        return;
    }
    uint16_t t = texel(s, (uint32_t)u, (uint32_t)v);
    if (!t) return;
    uint16_t c = t;
    if (!s->raw) {
        int tr = ((t & 31) * r) >> 7, tg = (((t >> 5) & 31) * g) >> 7, tb = (((t >> 10) & 31) * b) >> 7;
        c = (uint16_t)((tr > 31 ? 31 : tr) | ((tg > 31 ? 31 : tg) << 5) | ((tb > 31 ? 31 : tb) << 10) | (t & 0x8000));
    }
    plot(x, y, c, s->semi && (t & 0x8000), s->semi_mode);
}

/* ---------------------------------------------------------------- triangles */
typedef struct { int x, y, r, g, b, u, v; } Vtx;

static void triangle(const Shade* s, Vtx a, Vtx b, Vtx c)
{
    int64_t area = (int64_t)(b.x - a.x) * (c.y - a.y) - (int64_t)(b.y - a.y) * (c.x - a.x);
    if (area == 0) return;
    if (area < 0) { Vtx t = b; b = c; c = t; area = -area; }
    /* the GPU drops polygons that span too far */
    int minx = a.x, maxx = a.x, miny = a.y, maxy = a.y;
    minx = b.x < minx ? b.x : minx; maxx = b.x > maxx ? b.x : maxx;
    minx = c.x < minx ? c.x : minx; maxx = c.x > maxx ? c.x : maxx;
    miny = b.y < miny ? b.y : miny; maxy = b.y > maxy ? b.y : maxy;
    miny = c.y < miny ? c.y : miny; maxy = c.y > maxy ? c.y : maxy;
    if (maxx - minx > 1023 || maxy - miny > 511) return;
    minx = minx < G.clip_x0 ? G.clip_x0 : minx; maxx = maxx > G.clip_x1 ? G.clip_x1 : maxx;
    miny = miny < G.clip_y0 ? G.clip_y0 : miny; maxy = maxy > G.clip_y1 ? G.clip_y1 : maxy;
    if (minx > maxx || miny > maxy) return;

    /* edge e(p) = (q1 - q0) x (p - q0); top-left rule via bias */
    #define EDGE(p0, p1, X, Y) ((int64_t)((p1).x - (p0).x) * ((Y) - (p0).y) - (int64_t)((p1).y - (p0).y) * ((X) - (p0).x))
    #define BIAS(p0, p1) ((((p1).y == (p0).y && (p1).x < (p0).x) || (p1).y > (p0).y) ? 0 : -1)
    int bias0 = BIAS(b, c), bias1 = BIAS(c, a), bias2 = BIAS(a, b);
    for (int y = miny; y <= maxy; y++) {
        for (int x = minx; x <= maxx; x++) {
            int64_t w0 = EDGE(b, c, x, y), w1 = EDGE(c, a, x, y), w2 = EDGE(a, b, x, y);
            if (w0 + bias0 < 0 || w1 + bias1 < 0 || w2 + bias2 < 0) continue;
            int r, g, bl, u = 0, v = 0;
            if (s->gouraud) {
                r = (int)((w0 * a.r + w1 * b.r + w2 * c.r) / area);
                g = (int)((w0 * a.g + w1 * b.g + w2 * c.g) / area);
                bl = (int)((w0 * a.b + w1 * b.b + w2 * c.b) / area);
            } else { r = a.r; g = a.g; bl = a.b; }
            if (s->tex) {
                u = (int)((w0 * a.u + w1 * b.u + w2 * c.u) / area);
                v = (int)((w0 * a.v + w1 * b.v + w2 * c.v) / area);
            }
            shade_pixel(s, x, y, r, g, bl, u, v);
        }
    }
    #undef EDGE
    #undef BIAS
}

static void polygon(const uint32_t* w)
{
    uint32_t op = w[0] >> 24;
    int quad = (op & 8) != 0, n = quad ? 4 : 3;
    Shade s = {0};
    s.tex = (op & 4) != 0; s.raw = (op & 1) != 0; s.semi = (op & 2) != 0; s.gouraud = (op & 0x10) != 0;
    shade_from_texpage(&s, G.texpage);
    Vtx v[4];
    int i = 0;
    uint32_t col = w[0] & 0xFFFFFF;
    for (int k = 0; k < n; k++) {
        if (s.gouraud && k) col = w[i] & 0xFFFFFF;
        if (s.gouraud && k) i++;
        else if (!k) i = 1;
        uint32_t xy = w[i++];
        v[k].x = sext11(xy) + G.off_x;
        v[k].y = sext11(xy >> 16) + G.off_y;
        v[k].r = col & 0xFF; v[k].g = (col >> 8) & 0xFF; v[k].b = (col >> 16) & 0xFF;
        if (s.raw && s.tex) v[k].r = v[k].g = v[k].b = 128;
        if (s.tex) {
            uint32_t uv = w[i++];
            v[k].u = uv & 0xFF; v[k].v = (uv >> 8) & 0xFF;
            if (k == 0) { s.clut_x = ((uv >> 16) & 0x3F) * 16; s.clut_y = (uv >> 22) & 0x1FF; }
            if (k == 1) {
                uint32_t tp = uv >> 16;
                shade_from_texpage(&s, tp);
                G.texpage = (G.texpage & ~0x9FFu) | (tp & 0x9FF);
            }
        } else v[k].u = v[k].v = 0;
    }
    if (!s.tex) s.semi_mode = (G.texpage >> 5) & 3;
    triangle(&s, v[0], v[1], v[2]);
    if (quad) triangle(&s, v[1], v[2], v[3]);
}

/* ---------------------------------------------------------------- rectangles */
static void rectangle(const uint32_t* w)
{
    uint32_t op = w[0] >> 24;
    Shade s = {0};
    s.tex = (op & 4) != 0; s.raw = (op & 1) != 0; s.semi = (op & 2) != 0;
    shade_from_texpage(&s, G.texpage);
    int i = 1;
    int x = sext11(w[i]) + G.off_x, y = sext11(w[i] >> 16) + G.off_y; i++;
    int u0 = 0, v0 = 0;
    if (s.tex) {
        u0 = w[i] & 0xFF; v0 = (w[i] >> 8) & 0xFF;
        s.clut_x = ((w[i] >> 16) & 0x3F) * 16; s.clut_y = (w[i] >> 22) & 0x1FF; i++;
    }
    int wdt, hgt;
    switch ((op >> 3) & 3) {
    case 0: wdt = w[i] & 0x3FF; hgt = (w[i] >> 16) & 0x1FF; break;
    case 1: wdt = hgt = 1; break;
    case 2: wdt = hgt = 8; break;
    default: wdt = hgt = 16; break;
    }
    int r = w[0] & 0xFF, g = (w[0] >> 8) & 0xFF, b = (w[0] >> 16) & 0xFF;
    if (s.raw) r = g = b = 128;
    int fx = (G.texpage >> 12) & 1, fy = (G.texpage >> 13) & 1;
    for (int yy = 0; yy < hgt; yy++) {
        int py = y + yy;
        if (py < G.clip_y0 || py > G.clip_y1) continue;
        for (int xx = 0; xx < wdt; xx++) {
            int pxx = x + xx;
            if (pxx < G.clip_x0 || pxx > G.clip_x1) continue;
            shade_pixel(&s, pxx, py, r, g, b, fx ? u0 - xx : u0 + xx, fy ? v0 - yy : v0 + yy);
        }
    }
}

/* ---------------------------------------------------------------- lines */
static void line(int x0, int y0, uint32_t c0, int x1, int y1, uint32_t c1, int gouraud, int semi)
{
    int dx = x1 - x0, dy = y1 - y0;
    int adx = dx < 0 ? -dx : dx, ady = dy < 0 ? -dy : dy;
    if (adx > 1023 || ady > 511) return;
    int n = adx > ady ? adx : ady;
    int mode = (G.texpage >> 5) & 3;
    for (int k = 0; k <= n; k++) {
        int x = n ? x0 + (dx * k + (dx < 0 ? -n / 2 : n / 2)) / n : x0;
        int y = n ? y0 + (dy * k + (dy < 0 ? -n / 2 : n / 2)) / n : y0;
        uint32_t c = c0;
        if (gouraud && n) {
            int r = (int)(c0 & 0xFF) + ((int)(c1 & 0xFF) - (int)(c0 & 0xFF)) * k / n;
            int g = (int)((c0 >> 8) & 0xFF) + ((int)((c1 >> 8) & 0xFF) - (int)((c0 >> 8) & 0xFF)) * k / n;
            int b = (int)((c0 >> 16) & 0xFF) + ((int)((c1 >> 16) & 0xFF) - (int)((c0 >> 16) & 0xFF)) * k / n;
            c = (uint32_t)r | ((uint32_t)g << 8) | ((uint32_t)b << 16);
        }
        plot(x, y, rgb15(c), semi, mode);
    }
}

/* ---------------------------------------------------------------- GP0 */
static int packet_len(uint32_t w)
{
    uint32_t op = w >> 24;
    switch (op >> 5) {
    case 1: {                                   /* polygon */
        int n = (op & 8) ? 4 : 3;
        int per = 1 + ((op & 4) ? 1 : 0) + ((op & 0x10) ? 1 : 0);
        return 1 + n * per - ((op & 0x10) ? 1 : 0);
    }
    case 2:                                     /* line (polyline: open-ended) */
        if (op & 8) return -1;
        return (op & 0x10) ? 4 : 3;
    case 3: {                                   /* rectangle */
        int n = 2 + ((op & 4) ? 1 : 0) + (((op >> 3) & 3) == 0 ? 1 : 0);
        return n;
    }
    case 4: return 4;                           /* VRAM -> VRAM */
    case 5: return 3;                           /* CPU -> VRAM */
    case 6: return 3;                           /* VRAM -> CPU */
    default:
        if (op == 0x02) return 3;
        return 1;
    }
}

static void env(uint32_t w)
{
    switch (w >> 24) {
    case 0xE1: G.texpage = w & 0x3FFF; break;
    case 0xE2: G.tw_mask_x = w & 31; G.tw_mask_y = (w >> 5) & 31; G.tw_off_x = (w >> 10) & 31; G.tw_off_y = (w >> 15) & 31; break;
    case 0xE3: G.clip_x0 = w & 0x3FF; G.clip_y0 = (w >> 10) & 0x1FF; break;
    case 0xE4: G.clip_x1 = w & 0x3FF; G.clip_y1 = (w >> 10) & 0x1FF; break;
    case 0xE5: G.off_x = sext11(w); G.off_y = sext11(w >> 11); break;
    case 0xE6: G.set_mask = w & 1; G.check_mask = (w >> 1) & 1; break;
    default: break;
    }
}

static void execute(const uint32_t* w, int n)
{
    uint32_t op = w[0] >> 24;
    switch (op >> 5) {
    case 1: polygon(w); return;
    case 2: {
        int gouraud = (op & 0x10) != 0, semi = (op & 2) != 0;
        int i = 0;
        uint32_t c0 = w[0] & 0xFFFFFF;
        int x0 = sext11(w[1]) + G.off_x, y0 = sext11(w[1] >> 16) + G.off_y;
        i = 2;
        while (i < n) {
            uint32_t c1 = c0;
            if (gouraud) c1 = w[i++] & 0xFFFFFF;
            if (i >= n) break;
            int x1 = sext11(w[i]) + G.off_x, y1 = sext11(w[i] >> 16) + G.off_y; i++;
            line(x0, y0, c0, x1, y1, c1, gouraud, semi);
            x0 = x1; y0 = y1; c0 = c1;
        }
        return;
    }
    case 3: rectangle(w); return;
    case 4: {                                   /* VRAM -> VRAM */
        int sx = w[1] & 0x3FF, sy = (w[1] >> 16) & 0x1FF, dx = w[2] & 0x3FF, dy = (w[2] >> 16) & 0x1FF;
        int ww = ((w[3] - 1) & 0x3FF) + 1, hh = (((w[3] >> 16) - 1) & 0x1FF) + 1;
        for (int y = 0; y < hh; y++)
            for (int x = 0; x < ww; x++) {
                uint16_t c = *px(sx + x, sy + y);
                uint16_t* d = px(dx + x, dy + y);
                if (G.check_mask && (*d & 0x8000)) continue;
                *d = c | (G.set_mask ? 0x8000 : 0);
            }
        return;
    }
    case 5:                                     /* CPU -> VRAM: data follows */
        G.load_x = w[1] & 0x3FF; G.load_y = (w[1] >> 16) & 0x1FF;
        G.load_w = ((w[2] - 1) & 0x3FF) + 1; G.load_h = (((w[2] >> 16) - 1) & 0x1FF) + 1;
        G.load_i = 0; G.load_n = G.load_w * G.load_h; G.load_active = 1;
        return;
    case 6:                                     /* VRAM -> CPU: read through GPUREAD */
        G.store_x = w[1] & 0x3FF; G.store_y = (w[1] >> 16) & 0x1FF;
        G.store_w = ((w[2] - 1) & 0x3FF) + 1; G.store_h = (((w[2] >> 16) - 1) & 0x1FF) + 1;
        G.store_i = 0; G.store_n = G.store_w * G.store_h; G.store_active = 1;
        return;
    default:
        if (op == 0x02) {                       /* fill, ignores clip and mask */
            uint16_t c = rgb15(w[0]);
            int x = w[1] & 0x3F0, y = (w[1] >> 16) & 0x1FF;
            int ww = ((w[2] & 0x3FF) + 0xF) & ~0xF, hh = (w[2] >> 16) & 0x1FF;
            for (int yy = 0; yy < hh; yy++) for (int xx = 0; xx < ww; xx++) *px(x + xx, y + yy) = c;
        } else if (op >= 0xE1 && op <= 0xE6) env(w[0]);
        return;
    }
}

static void load_pixel(uint16_t c)
{
    int x = G.load_x + G.load_i % G.load_w, y = G.load_y + G.load_i / G.load_w;
    uint16_t* d = px(x, y);
    if (!(G.check_mask && (*d & 0x8000))) *d = c | (G.set_mask ? 0x8000 : 0);
    if (++G.load_i >= G.load_n) G.load_active = 0;
}

void dc_gpu_gp0(uint32_t w)
{
    g_dc_stats.gp0_words++;
    if (G.load_active) {
        load_pixel((uint16_t)w);
        if (G.load_active) load_pixel((uint16_t)(w >> 16));
        return;
    }
    if (G.ncmd == 0) {
        G.need = packet_len(w);
        G.polyline = G.need < 0;
    }
    if (G.ncmd < 16) G.cmd[G.ncmd] = w;
    G.ncmd++;
    if (G.polyline) {
        /* terminator after at least two vertices */
        if (G.ncmd >= 3 && (w & 0xF000F000u) == 0x50005000u) {
            execute(G.cmd, G.ncmd - 1 < 16 ? G.ncmd - 1 : 16);
            G.ncmd = 0;
        } else if (G.ncmd >= 16) {
            /* long polyline: draw what we have, keep the last vertex */
            int gouraud = (G.cmd[0] >> 28) & 1;
            execute(G.cmd, 16);
            uint32_t head = G.cmd[0];
            uint32_t last_c = gouraud ? G.cmd[14] : head, last_v = G.cmd[15];
            G.cmd[0] = (head & 0xFF000000u) | (last_c & 0xFFFFFF);
            G.cmd[1] = last_v; G.ncmd = 2;
        }
        return;
    }
    if (G.ncmd >= G.need) {
        execute(G.cmd, G.ncmd);
        G.ncmd = 0;
    }
}

uint32_t dc_gpu_read(void)
{
    if (!G.store_active) return G.gpuread;
    uint32_t v = 0;
    for (int k = 0; k < 2 && G.store_active; k++) {
        int x = G.store_x + G.store_i % G.store_w, y = G.store_y + G.store_i / G.store_w;
        v |= (uint32_t)*px(x, y) << (16 * k);
        if (++G.store_i >= G.store_n) G.store_active = 0;
    }
    G.gpuread = v;
    return v;
}

/* ---------------------------------------------------------------- GP1 */
void dc_gpu_gp1(uint32_t w)
{
    g_dc_stats.gp1_writes++;
    switch (w >> 24) {
    case 0x00:
        memset(&G, 0, sizeof G);
        G.display_off = 1; G.clip_x1 = 1023; G.clip_y1 = 511;
        break;
    case 0x01: G.ncmd = 0; G.load_active = 0; break;
    case 0x03: G.display_off = w & 1; break;
    case 0x04: G.dma_dir = w & 3; break;
    case 0x05: G.disp_x = w & 0x3FE; G.disp_y = (w >> 10) & 0x1FF; break;
    case 0x08: G.disp_mode = w & 0xFF; break;
    case 0x10:
        switch (w & 7) {
        case 2: G.gpuread = (G.tw_mask_x | (G.tw_mask_y << 5) | (G.tw_off_x << 10) | (G.tw_off_y << 15)); break;
        case 3: G.gpuread = (uint32_t)G.clip_x0 | ((uint32_t)G.clip_y0 << 10); break;
        case 4: G.gpuread = (uint32_t)G.clip_x1 | ((uint32_t)G.clip_y1 << 10); break;
        case 5: G.gpuread = ((uint32_t)G.off_x & 0x7FF) | (((uint32_t)G.off_y & 0x7FF) << 11); break;
        case 7: G.gpuread = 2; break;
        default: break;
        }
        break;
    default: break;
    }
}

uint32_t dc_gpu_stat(void)
{
    static uint32_t odd;
    odd ^= 1u;
    uint32_t st = (G.texpage & 0x7FF) | ((uint32_t)G.set_mask << 11) | ((uint32_t)G.check_mask << 12);
    st |= ((G.disp_mode >> 6) & 1) << 16;            /* horizontal 368 */
    st |= (G.disp_mode & 0x3F) << 17;                /* width, height, video mode, depth, interlace */
    st |= (uint32_t)G.display_off << 23;
    st |= 1u << 26 | 1u << 28;                       /* ready for commands and DMA blocks */
    if (G.store_active) st |= 1u << 27;
    st |= G.dma_dir << 29;
    switch (G.dma_dir) {
    case 1: st |= 1u << 25; break;
    case 2: st |= 1u << 25; break;
    case 3: if (G.store_active) st |= 1u << 25; break;
    default: break;
    }
    return st | (odd << 31);
}

void dc_gpu_display(DcGpuDisplay* d)
{
    static const int widths[4] = { 256, 320, 512, 640 };
    d->x = (int)G.disp_x; d->y = (int)G.disp_y;
    d->w = (G.disp_mode & 0x40) ? 368 : widths[G.disp_mode & 3];
    d->h = ((G.disp_mode & 0x24) == 0x24) ? 480 : 240;
    d->depth24 = (G.disp_mode >> 4) & 1;
    d->enabled = !G.display_off;
}

void dc_gpu_reset(void) { dc_gpu_gp1(0); }
