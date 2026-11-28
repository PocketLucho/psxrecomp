/* PS1 MDEC (motion decoder) for the Dreamcast target.
 *
 * Same arithmetic as runtime/src/mdec.c (Beetle's dequantisation, IDCT and
 * YCbCr conversion), but the output is produced lazily: a decode command only
 * keeps its run-length input, and each macroblock is decoded when DMA1 (or a
 * read of the data port) drains the previous one. A full 320x240 24-bit frame
 * would otherwise need a 230 KB output buffer.
 */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
void dc_host_log(const char* s);
/* The IDCT runs on the SH4's vector unit (FTRV), about 1.6x faster per
   block; DC_MDEC_NO_FTRV keeps the integer one. */
#if defined(_arch_dreamcast) && !defined(DC_MDEC_NO_FTRV)
#include <dc/matrix.h>
#define MDEC_FTRV 1
#endif

static const uint8_t s_zz[64] = {
    0x00, 0x08, 0x01, 0x02, 0x09, 0x10, 0x18, 0x11, 0x0a, 0x03, 0x04, 0x0b, 0x12, 0x19, 0x20, 0x28,
    0x21, 0x1a, 0x13, 0x0c, 0x05, 0x06, 0x0d, 0x14, 0x1b, 0x22, 0x29, 0x30, 0x38, 0x31, 0x2a, 0x23,
    0x1c, 0x15, 0x0e, 0x07, 0x0f, 0x16, 0x1d, 0x24, 0x2b, 0x32, 0x39, 0x3a, 0x33, 0x2c, 0x25, 0x1e,
    0x17, 0x1f, 0x26, 0x2d, 0x34, 0x3b, 0x3c, 0x35, 0x2e, 0x27, 0x2f, 0x36, 0x3d, 0x3e, 0x37, 0x3f
};

static struct {
    uint32_t cmd, need, cnt, cap;
    uint16_t* in;
    uint8_t yq[64], uvq[64];
    int16_t sc[64];
    uint8_t b15, sgn, depth, busy, dma_in, dma_out, blk;
    /* lazy decode of the last decode command */
    uint32_t pos, end;
    const uint16_t* din;     /* decode input: guest RAM (DMA0) or M.in */
    int live;
    uint8_t ob[768];
    uint32_t olen, opos;
} M;

uint32_t g_dc_mdec_us, g_dc_mdec_mbs;

static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }
static inline int m9c(int v)
{
    v = (int)((int32_t)((uint32_t)v << 23) >> 23);
    return v < -128 ? -128 : v > 127 ? 127 : v;
}

/* One 1-D pass: o[x*8 + c] for the 8 inputs s. With the usual cosine table
 * (row 7-x is row x with the odd terms negated) half the products do: the
 * sums are the same integers, so the result is exact. */
static int s_sym;
static inline void pass(const int16_t* s, int16_t* o, int c, int last)
{
    if (s_sym) {
        for (int x = 0; x < 4; x++) {
            const int16_t* k = M.sc + x * 8;
            int e = s[0] * k[0] + s[2] * k[2] + s[4] * k[4] + s[6] * k[6];
            int d = s[1] * k[1] + s[3] * k[3] + s[5] * k[5] + s[7] * k[7];
            int lo = (e + d + 0x4000) >> 15, hi = (e - d + 0x4000) >> 15;
            if (last) { o[c * 8 + x] = (int16_t)m9c(lo); o[c * 8 + 7 - x] = (int16_t)m9c(hi); }
            else { o[x * 8 + c] = (int16_t)lo; o[(7 - x) * 8 + c] = (int16_t)hi; }
        }
        return;
    }
    for (int x = 0; x < 8; x++) {
        const int16_t* k = M.sc + x * 8;
        int sum = s[0] * k[0] + s[1] * k[1] + s[2] * k[2] + s[3] * k[3] + s[4] * k[4] + s[5] * k[5] + s[6] * k[6] + s[7] * k[7];
        int v = (sum + 0x4000) >> 15;
        if (last) o[c * 8 + x] = (int16_t)m9c(v); else o[x * 8 + c] = (int16_t)v;
    }
}

#ifdef MDEC_FTRV
/* The IDCT as 4x4 matrix products on the SH4's vector unit (FTRV): each
 * 1-D pass is the 8x8 cosine table split into four 4x4 blocks. Float keeps
 * 24 bits, so an output can differ from the integer IDCT by one step where
 * the integer sum sits exactly on a rounding boundary; a self-test against
 * the integer path decides whether it is used. */
static matrix_t s_m[2][2] __attribute__((aligned(32)));   /* [out half][in half] */
static int s_ftrv;
static inline int rnd(float v) { return (int)(v + 32768.5f) - 32768; }
#define FTRV4(a, b, c, d, o, ADD) do { \
    register float x0_ __asm__("fr0") = (a), x1_ __asm__("fr1") = (b), x2_ __asm__("fr2") = (c), x3_ __asm__("fr3") = (d); \
    __asm__ __volatile__("ftrv xmtrx,fv0" : "+f"(x0_), "+f"(x1_), "+f"(x2_), "+f"(x3_)); \
    if (ADD) { (o)[0] += x0_; (o)[1] += x1_; (o)[2] += x2_; (o)[3] += x3_; } \
    else { (o)[0] = x0_; (o)[1] = x1_; (o)[2] = x2_; (o)[3] = x3_; } } while (0)
/* acc[c][x] = sum_j in[c][j] * sc[x][j] for the rows c set in nz */
static void pass_ftrv(const float in[8][8], float acc[8][8], unsigned nz)
{
    for (int h = 0; h < 2; h++) {
        mat_load(&s_m[h][0]);
        for (int c = 0; c < 8; c++)
            if (nz >> c & 1) { const float* r = in[c]; FTRV4(r[0], r[1], r[2], r[3], acc[c] + h * 4, 0); }
        mat_load(&s_m[h][1]);
        for (int c = 0; c < 8; c++)
            if (nz >> c & 1) { const float* r = in[c] + 4; FTRV4(r[0], r[1], r[2], r[3], acc[c] + h * 4, 1); }
    }
}
static void idct_ftrv(int16_t* b)
{
    float in[8][8] __attribute__((aligned(8))), acc[8][8] __attribute__((aligned(8)));
    unsigned nz = 0;
    for (int c = 0; c < 8; c++) {
        const int16_t* r = b + c * 8;
        if (!(r[0] | r[1] | r[2] | r[3] | r[4] | r[5] | r[6] | r[7])) continue;
        nz |= 1u << c;
        for (int j = 0; j < 8; j++) in[c][j] = (float)r[j];
    }
    pass_ftrv(in, acc, nz);
    /* the first pass rounds to integers, as the hardware does; its output is
       transposed (t[x*8 + c]), and every row of it is in use */
    for (int c = 0; c < 8; c++)
        if (nz >> c & 1) for (int x = 0; x < 8; x++) in[x][c] = (float)rnd(acc[c][x]);
        else for (int x = 0; x < 8; x++) in[x][c] = 0.0f;
    pass_ftrv(in, acc, 0xFFu);
    for (int c = 0; c < 8; c++)
        for (int x = 0; x < 8; x++) b[c * 8 + x] = (int16_t)m9c(rnd(acc[c][x]));
}
#endif

static void idct_int(int16_t* b);
static void check_sym(void)
{
    s_sym = 1;
    for (int x = 0; x < 4; x++)
        for (int j = 0; j < 8; j++)
            if (M.sc[(7 - x) * 8 + j] != ((j & 1) ? -M.sc[x * 8 + j] : M.sc[x * 8 + j])) s_sym = 0;
#ifdef MDEC_FTRV
    s_ftrv = 0;
    /* matrix_t is column-major: m[j][x] multiplies input j into output x */
    for (int h = 0; h < 2; h++)
        for (int g = 0; g < 2; g++)
            for (int x = 0; x < 4; x++)
                for (int j = 0; j < 4; j++)
                    s_m[h][g][j][x] = (float)M.sc[(h * 4 + x) * 8 + g * 4 + j] * (1.0f / 32768.0f);
    int bad = 0;
    uint32_t seed = 12345;
    for (int n = 0; n < 64 && !bad; n++) {
        int16_t a[64], f[64];
        for (int i = 0; i < 64; i++) {
            seed = seed * 1103515245u + 12345u;
            int v = (int)((seed >> 16) & 0x7FFF) - 0x4000;
            a[i] = (int16_t)(i == 0 || (seed >> 8) % 5 == 0 ? v >> (n & 7) : 0);
        }
        memcpy(f, a, sizeof a);
        idct_int(a);
        idct_ftrv(f);
        for (int i = 0; i < 64; i++) if (abs(a[i] - f[i]) > 1) bad = 1;
    }
    s_ftrv = !bad;
    { static int said; if (!said) { said = 1; dc_host_log(s_ftrv ? "[mdec] IDCT on FTRV\n" : "[mdec] IDCT on integers (FTRV self-test failed)\n");
#ifdef DC_MDEC_BENCH
      { uint64_t dc_host_ticks_us(void); int16_t a[64]; char b[80];
        for (int i = 0; i < 64; i++) a[i] = (int16_t)((i * 37) % 200 - 100);
        uint64_t t0 = dc_host_ticks_us(); for (int k = 0; k < 2000; k++) { a[0] = (int16_t)k; idct_int(a); }
        uint64_t t1 = dc_host_ticks_us(); for (int k = 0; k < 2000; k++) { a[0] = (int16_t)k; idct_ftrv(a); }
        uint64_t t2 = dc_host_ticks_us();
        snprintf(b, sizeof b, "[mdec] 2000 IDCTs: int %u us, ftrv %u us\n", (unsigned)(t1 - t0), (unsigned)(t2 - t1)); dc_host_log(b); }
#endif
    } }
#endif
}

static void idct_int(int16_t* b)
{
    int16_t t[64];
    for (int c = 0; c < 8; c++) {
        const int16_t* s = b + c * 8;
        if (!(s[0] | s[1] | s[2] | s[3] | s[4] | s[5] | s[6] | s[7])) {
            for (int x = 0; x < 8; x++) t[x * 8 + c] = 0;
            continue;
        }
        pass(s, t, c, 0);
    }
    for (int c = 0; c < 8; c++) {
        const int16_t* s = t + c * 8;
        if (!(s[0] | s[1] | s[2] | s[3] | s[4] | s[5] | s[6] | s[7])) {
            for (int x = 0; x < 8; x++) b[c * 8 + x] = 0;
            continue;
        }
        pass(s, b, c, 1);
    }
}

static void idct(int16_t* b, int ac)
{
    if (!ac) {
        int16_t t0[8];
        for (int x = 0; x < 8; x++) t0[x] = (int16_t)((b[0] * M.sc[x * 8] + 0x4000) >> 15);
        for (int c = 0; c < 8; c++)
            for (int x = 0; x < 8; x++) b[c * 8 + x] = (int16_t)m9c((t0[c] * M.sc[x * 8] + 0x4000) >> 15);
        return;
    }
#ifdef MDEC_FTRV
    if (s_ftrv) { idct_ftrv(b); return; }
#endif
    idct_int(b);
}

static int block(int16_t* b, const uint8_t* q)
{
    memset(b, 0, 64 * sizeof *b);
    if (M.pos >= M.end) return 0;
    uint16_t w = M.din[M.pos++];
    while (w == 0xFE00u && M.pos < M.end) w = M.din[M.pos++];
    if (w == 0xFE00u) return 0;
    uint32_t qs = (w >> 10) & 0x3Fu, k = 0;
    int ci = (int16_t)(w << 6) >> 6, qq = q[0];
    int v = qq ? (((ci * qq) << 4) + (ci ? (ci < 0 ? 8 : -8) : 0)) : ((ci * 2) << 4);
    b[0] = (int16_t)clampi(v, -0x4000, 0x3FFF);
    int ac = 0;
    while (M.pos < M.end && k < 63u) {
        w = M.din[M.pos++];
        if (w == 0xFE00u) break;
        k += ((w >> 10) & 0x3Fu) + 1u;
        if (k >= 64u) break;
        ci = (int16_t)(w << 6) >> 6;
        qq = (int)qs * q[k];
        v = qq ? ((((ci * qq) >> 3) << 4) + (ci ? (ci < 0 ? 8 : -8) : 0)) : ((ci * 2) << 4);
        b[s_zz[k]] = (int16_t)clampi(v, -0x4000, 0x3FFF);
        ac |= b[s_zz[k]];
    }
    idct(b, ac);
    return 1;
}

static inline int c555(int c) { int v = (c + 4) >> 3; return v > 31 ? 31 : v; }

/* Decode the next macroblock into M.ob. */
static int next_mb(void)
{
    M.olen = M.opos = 0;
    if (!M.live) return 0;
    if (M.depth < 2) {
        int16_t y[64];
        if (!block(y, M.yq)) { M.live = 0; return 0; }
        for (int i = 0; i < 64; i++) {
            int v = clampi(y[i], -128, 127);
            M.ob[i] = M.sgn ? (uint8_t)(int8_t)v : (uint8_t)(v + 128);
        }
        M.olen = 64;
        return 1;
    }
    int16_t cr[64], cb[64], y[4][64];
    if (!block(cr, M.uvq) || !block(cb, M.uvq) || !block(y[0], M.yq) || !block(y[1], M.yq) ||
        !block(y[2], M.yq) || !block(y[3], M.yq)) { M.live = 0; return 0; }
    uint8_t* o = M.ob;
    uint16_t pxor = (uint16_t)((M.b15 ? 0x8000u : 0u) | (M.sgn ? 0x4210u : 0u));
    uint8_t rx = M.sgn ? 0x80u : 0u;
    /* m9c() only depends on the low 9 bits of its input: one table per
     * output format gives the clamped, sign-adjusted byte. */
    static uint8_t t24[512], t15[512]; static int tinit, tsgn = -1;
    if (!tinit || tsgn != M.sgn) {
        for (int i = 0; i < 512; i++) { int c = m9c(i) ^ 0x80; t24[i] = (uint8_t)(c ^ rx); t15[i] = (uint8_t)c555(c & 0xFF); }
        tinit = 1; tsgn = M.sgn;
    }
    for (int cy = 0; cy < 8; cy++)
        for (int cx = 0; cx < 8; cx++) {
            int ch = cy * 8 + cx, r_ = cr[ch], b_ = cb[ch];
            int rr = ((359 * r_) + 0x80) >> 8;
            int gg = (((-88 * b_) & ~0x1F) + ((-183 * r_) & ~0x07) + 0x80) >> 8;
            int bb = ((454 * b_) + 0x80) >> 8;
            const int16_t* yb = y[(cy >= 4 ? 2 : 0) + (cx >= 4 ? 1 : 0)] + ((cy & 3) * 2) * 8 + (cx & 3) * 2;
            for (int dy = 0; dy < 2; dy++)
                for (int dx = 0; dx < 2; dx++) {
                    int yy = yb[dy * 8 + dx], px_ = cx * 2 + dx, py_ = cy * 2 + dy;
                    if (M.depth == 3) {
                        uint8_t* q = M.ob + (py_ * 16 + px_) * 2;
                        uint16_t p = (uint16_t)((t15[(yy + rr) & 0x1FF] | (t15[(yy + gg) & 0x1FF] << 5) | (t15[(yy + bb) & 0x1FF] << 10)) ^ pxor);
                        q[0] = (uint8_t)p; q[1] = (uint8_t)(p >> 8);
                    } else {
                        uint8_t* q = M.ob + (py_ * 16 + px_) * 3;
                        q[0] = t24[(yy + rr) & 0x1FF]; q[1] = t24[(yy + gg) & 0x1FF]; q[2] = t24[(yy + bb) & 0x1FF];
                    }
                }
        }
    o = M.ob + (M.depth == 3 ? 512 : 768);
    M.olen = (uint32_t)(o - M.ob);
    g_dc_mdec_mbs++;
    return 1;
}

static uint8_t in_byte(uint32_t i) { uint16_t h = M.in[i >> 1]; return (i & 1) ? (uint8_t)(h >> 8) : (uint8_t)h; }

static void finish(void) { M.need = M.cnt = 0; M.busy = 0; }

static void execute(void)
{
    uint32_t op = M.cmd >> 29;
    if (op == 2) {
        for (uint32_t i = 0; i < 64; i++) M.yq[i] = in_byte(i);
        if (M.cmd & 1u) for (uint32_t i = 0; i < 64; i++) M.uvq[i] = in_byte(64 + i);
        else memcpy(M.uvq, M.yq, 64);
    } else if (op == 3) {
        for (uint32_t i = 0; i < 64; i++) M.sc[((i & 7) << 3) | ((i >> 3) & 7)] = (int16_t)((int16_t)M.in[i] >> 3);
        check_sym();
    } else if (op == 1) {
        M.din = M.in; M.pos = 0; M.end = M.cnt; M.live = 1; M.olen = M.opos = 0;
    }
    finish();
}

#ifdef DC_LOG
static uint32_t s_ml[24][4]; static unsigned s_mln; static int s_mdump;
#endif
static void begin(uint32_t v)
{
#ifdef DC_LOG
    { uint64_t dc_host_ticks_us(void); uint32_t* e = s_ml[s_mln++ % 24]; e[0] = (uint32_t)(dc_host_ticks_us() / 100u); e[1] = v; e[2] = M.olen - M.opos; e[3] = M.end - M.pos;
      uint32_t op = v >> 29;
      if (!s_mdump && (op == 0 || op >= 4)) { s_mdump = 1; void dc_cd_dump_recent(void); char b[96];
        for (unsigned i = 0; i < 24; i++) { uint32_t* f = s_ml[(s_mln + i) % 24]; if (!f[0]) continue;
            snprintf(b, sizeof b, "[mcmd] t=%u.%ums %08X left_out=%u left_in=%u\n", (unsigned)(f[0] / 10), (unsigned)(f[0] % 10), (unsigned)f[1], (unsigned)f[2], (unsigned)f[3]); dc_host_log(b); }
        dc_cd_dump_recent(); } }
#endif
#ifdef DC_LOG
    if (!s_mdump) { void dc_ev(uint32_t, uint32_t, uint32_t); dc_ev('C', v, M.end - M.pos); }
    { static int nc; if (nc < 40 && (v >> 29) == 1) { nc++; char b[64]; snprintf(b, sizeof b, "[mcmd0] %08X\n", (unsigned)v); dc_host_log(b); } }
#endif
    M.cmd = v;
    M.b15 = (uint8_t)((v >> 25) & 1u); M.sgn = (uint8_t)((v >> 26) & 1u); M.depth = (uint8_t)((v >> 27) & 3u);
    M.blk = 4; M.cnt = 0; M.busy = 1;
    switch (v >> 29) {
    case 1: M.need = (v & 0xFFFFu) * 2u; break;
    case 2: M.need = (v & 1u) ? 64u : 32u; break;
    case 3: M.need = 64u; break;
    default: M.need = 0; break;
    }
    if (M.need > M.cap) {
        uint32_t c = M.cap ? M.cap : 4096u;
        while (c < M.need) c *= 2u;
        uint16_t* n = (uint16_t*)realloc(M.in, c * 2u);
        if (!n) { M.need = 0; } else { M.in = n; M.cap = c; }
    }
    if (!M.need) finish();
    /* a new decode replaces whatever output the last one had left */
    if ((v >> 29) == 1) { M.live = 0; M.olen = M.opos = 0; }
}

static void put(uint32_t v)
{
    if (M.busy && M.cnt < M.need) {
        M.in[M.cnt++] = (uint16_t)v;
        if (M.cnt < M.need) M.in[M.cnt++] = (uint16_t)(v >> 16);
        if (M.cnt >= M.need) execute();
        return;
    }
    begin(v);
}

/* Padding (0xFE00) left at the end of the input is not more output. */
static int out_ready(void)
{
    if (M.opos < M.olen) return 1;
    while (M.live && M.pos < M.end && M.din[M.pos] == 0xFE00u) M.pos++;
    return M.live && M.pos < M.end;
}

/* A frame is being decoded: output is left for DMA1 to take. */
int dc_mdec_decoding(void) { return out_ready(); }

static uint32_t get(void)
{
    if (M.opos >= M.olen && !next_mb()) return 0;
    uint32_t v;
    memcpy(&v, M.ob + M.opos, 4);
    M.opos += 4;
    return v;
}

void dc_mdec_reset(void)
{
    uint16_t* in = M.in; uint32_t cap = M.cap;
    memset(&M, 0, sizeof M);
    M.in = in; M.cap = cap;
    for (int i = 0; i < 64; i++) M.yq[i] = M.uvq[i] = 1;
    M.depth = 3; M.blk = 4;
}

uint32_t dc_mdec_read(uint32_t p)
{
    if (!(p & 4)) return get();
    uint32_t rem = (M.busy && M.need > M.cnt) ? (M.need - M.cnt + 1u) / 2u : 0;
    uint32_t st = rem ? ((rem - 1u) & 0xFFFFu) : 0xFFFFu;
    st |= ((uint32_t)M.blk & 7u) << 16;
    st |= (uint32_t)M.b15 << 23 | (uint32_t)M.sgn << 24 | ((uint32_t)M.depth & 3u) << 25;
    int ordy = out_ready();
    if (M.dma_out && ordy) st |= 1u << 27;
    if (M.dma_in && M.busy) st |= 1u << 28;
    if (M.busy || ordy) st |= 1u << 29;
    if (!ordy) st |= 1u << 31;
    return st;
}

void dc_mdec_write(uint32_t p, uint32_t v)
{
    if (!(p & 4)) { put(v); return; }
    if (v & 0x80000000u) dc_mdec_reset();
    M.dma_in = (uint8_t)((v >> 30) & 1u);
    M.dma_out = (uint8_t)((v >> 29) & 1u);
}

/* DMA0: RAM -> MDEC. */
void dc_mdec_dma_in(const uint8_t* ram, uint32_t madr, uint32_t words)
{
    for (uint32_t i = 0; i < words; i++, madr += 4) {
        /* The run-length data of a decode is read where DMA0 points, as the
         * macroblocks are decoded: the game is still writing it (its VLC
         * decoder runs in pieces between slices), and the real DMA0 only
         * moves each word when the MDEC asks for it. */
        if (M.busy && (M.cmd >> 29) == 1u && M.cnt == 0 && !(madr & 3u)) {
            uint32_t a = madr & 0x1FFFFCu, left = words - i;
            if (left * 2u >= M.need && a + M.need * 2u <= 0x200000u) {
                M.din = (const uint16_t*)(ram + a);
                M.pos = 0; M.end = M.need; M.live = 1; M.olen = M.opos = 0;
                finish();
                i += (M.end + 1u) / 2u - 1u; madr += ((M.end + 1u) / 2u - 1u) * 4u;
                continue;
            }
        }
        uint32_t w; memcpy(&w, ram + (madr & 0x1FFFFCu), 4);
        put(w);
    }
}

/* DMA1: MDEC -> RAM. Returns 0 (nothing transferred) when the MDEC has no
 * output yet: the channel then waits for a decode command. */
int dc_mdec_dma_out(uint8_t* ram, uint32_t madr, uint32_t words)
{
    if (!out_ready()) return 0;
    while (words) {
        if (M.opos >= M.olen && !next_mb()) {
            /* the stream ran short: the rest stays as it was */
            break;
        }
        uint32_t n = (M.olen - M.opos) >> 2;
        if (n > words) n = words;
        uint32_t a = madr & 0x1FFFFCu;
        if (a + n * 4u <= 0x200000u) memcpy(ram + a, M.ob + M.opos, n * 4u);
        else for (uint32_t i = 0; i < n; i++) memcpy(ram + ((madr + i * 4u) & 0x1FFFFCu), M.ob + M.opos + i * 4u, 4);
        M.opos += n * 4u; madr += n * 4u; words -= n;
    }
    return 1;
}


#ifdef DC_LOG
void dc_mdec_debug(char* b, unsigned n)
{
    snprintf(b, n, "[mdec-state] cmd=%08X busy=%d need=%u cnt=%u live=%d pos=%u end=%u olen=%u opos=%u\n",
             (unsigned)M.cmd, M.busy, (unsigned)M.need, (unsigned)M.cnt, M.live, (unsigned)M.pos, (unsigned)M.end, (unsigned)M.olen, (unsigned)M.opos);
}
#endif
