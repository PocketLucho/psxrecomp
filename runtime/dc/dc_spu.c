/* dc_spu.c - PS1 SPU for the Dreamcast runtime: 24 ADPCM voices with ADSR,
 * pitch and volume, plus the CD (XA) input, mixed in software at 44.1 kHz and
 * handed to the host audio stream (AICA on Dreamcast).
 *
 * The voice, envelope and sweep logic follows runtime/src/spu.c (itself after
 * Beetle's PS_SPU). Left out for now: reverb, the capture buffers and Gaussian
 * interpolation (linear is used: it costs a fraction on the SH4).
 *
 * Mixing runs on the guest thread. dc_spu_tick() renders up to host "now";
 * key-on/off and pitch writes first render up to the moment of the write, so
 * notes start where the sound driver put them, not at the next 60 Hz poll. */
#include "dc_runtime.h"
#include <string.h>
#include <stdio.h>

#define SPU_RAM_SIZE (512u * 1024u)
#define NV 24
#ifndef DC_SPU_DIV
#define DC_SPU_DIV 2               /* mix at 22.05 kHz: half the SH4 time */
#endif
#define DIV DC_SPU_DIV
#define RATE (44100u / DIV)

uint8_t g_dc_spu_ram[SPU_RAM_SIZE] __attribute__((aligned(32)));
static uint16_t s_reg[256];                 /* 0x1F801C00..0x1F801DFF */
static uint32_t s_taddr;                    /* transfer address, bytes */
static uint32_t s_endx;
static int s_irq_flag;
static uint16_t s_noise; static int32_t s_noise_t;

enum { ATK, DEC, SUS, REL };
typedef struct {
    uint32_t cur, rep;      /* byte addresses */
    uint32_t phase;
    int16_t smp[28], prev;  /* decoded block; last sample of the block before */
    int16_t h1, h2;
    uint8_t flags, idx, active, adsr;
    uint16_t env;
    uint32_t adiv;
    int32_t cinc, cdiv; uint8_t cok;   /* cached envelope step for the current level/phase */
    /* hardware path */
    int16_t smp_i;          /* sample cache entry, -1 none */
    uint8_t hw_on;          /* AICA channel playing */
    uint16_t hw_vol, hw_pan;
    uint32_t hw_freq;
    uint64_t pos;           /* play position, 20.12 fixed samples */
} Voice;
static Voice s_v[NV];

typedef struct { int16_t level; uint32_t div; } Sweep;
static Sweep s_vsw[NV][2], s_msw[2];

#define REG(a) s_reg[((a) - 0x1F801C00u) >> 1]
static inline uint16_t vreg(int v, int r) { return s_reg[v * 8 + r]; }
static inline int16_t clamp16(int32_t v) { return v > 32767 ? 32767 : v < -32768 ? -32768 : (int16_t)v; }

/* ---------------------------------------------------------------- IRQ */
static void irq_check(uint32_t addr, uint32_t len)
{
    if (!(REG(0x1F801DAAu) & 0x40u) || s_irq_flag) return;
    uint32_t t = REG(0x1F801DA4u), a = addr & (SPU_RAM_SIZE - 1u);
    uint32_t first = a >> 3, last = (a + len - 1u) >> 3;
    if (t >= first && t <= last) { s_irq_flag = 1; dc_irq_raise(1u << 9); }
}

/* ---------------------------------------------------------------- envelopes */
static void vc_delta(uint8_t zs, uint8_t speed, int log_mode, int dec_mode, int inv,
                     int16_t cur, int* inc_out, int* div_out)
{
    int inc = 7 - (speed & 3);
    if (inv) inc = ~inc;
    int div = 32768;
    if (speed < 0x2C) inc = (int)((unsigned)inc << ((0x2F - speed) >> 2));
    if (speed >= 0x30) div >>= (speed - 0x2C) >> 2;
    if (log_mode) {
        if (dec_mode) inc = (cur * inc) >> 15;
        else if ((cur & 0x7FFF) >= 0x6000) {
            if (speed < 0x28) inc >>= 2;
            else if (speed >= 0x2C) div >>= 2;
            else { inc >>= 1; div >>= 1; }
        }
    }
    if (div == 0 && speed < zs) div = 1;
    *inc_out = inc; *div_out = div;
}

static void adsr_calc(int i, Voice* v)
{
    uint32_t raw = (uint32_t)vreg(i, 4) | ((uint32_t)vreg(i, 5) << 16);
    int dr = (int)((raw >> 4) & 15), ar = (int)((raw >> 8) & 0x7F);
    int aexp = (int)((raw >> 15) & 1), rr = (int)((raw >> 16) & 31), rexp = (int)((raw >> 21) & 1);
    int sr = (int)((raw >> 22) & 0x7F), sdec = (int)((raw >> 30) & 1), sexp = (int)((raw >> 31) & 1);
    if (v->adsr == ATK && v->env == 0x7FFF) v->adsr = DEC;
    int inc, div;
    switch (v->adsr) {
    case ATK: vc_delta(0x7F, (uint8_t)ar, aexp, 0, 0, (int16_t)v->env, &inc, &div); break;
    case DEC: vc_delta(0x7C, (uint8_t)(dr << 2), 1, 1, 1, (int16_t)v->env, &inc, &div); break;
    case SUS: vc_delta(0x7F, (uint8_t)sr, sexp, sdec, sdec, (int16_t)v->env, &inc, &div); break;
    default:  vc_delta(0x7C, (uint8_t)(rr << 2), rexp, 1, 1, (int16_t)v->env, &inc, &div); break;
    }
    v->cinc = inc; v->cdiv = div; v->cok = 1;
}

/* The step only depends on phase, registers and the level, and the level only
 * moves when the divider overflows, so it is recomputed then and not per sample. */
static inline void adsr_run(int i, Voice* v)
{
    if (!v->cok) adsr_calc(i, v);
    v->adiv += (uint32_t)v->cdiv;
    if (!(v->adiv & 0x8000u)) return;
    uint16_t prev = v->env;
    v->adiv = 0;
    v->cok = 0;
    v->env = (uint16_t)((int)v->env + v->cinc);
    if (v->adsr == ATK) { if (((prev ^ v->env) & v->env) & 0x8000u) v->env = 0x7FFF; }
    else if (v->env & 0x8000u) {
        int sdec = (vreg(i, 5) >> 14) & 1;
        v->env = (uint16_t)((v->adsr == SUS && !sdec) ? 0x7FFF : 0);
    }
    if (v->adsr == DEC && v->env < (uint16_t)(((vreg(i, 4) & 15) + 1) << 11)) v->adsr = SUS;
}

static void sweep_write(Sweep* s, uint16_t raw) { if (!(raw & 0x8000u)) s->level = (int16_t)(uint16_t)(raw << 1); s->div = 0; }
static void sweep_step(Sweep* s, uint16_t raw)
{
    if (!(raw & 0x8000u)) return;
    int neg = (raw >> 12) & 1, dec = (raw >> 13) & 1;
    int32_t w = neg ? -s->level : s->level;
    if (w < 0) w = 0;
    int inc, div;
    vc_delta(0x7F, (uint8_t)(raw & 0x7F), (raw >> 14) & 1, dec, dec, (int16_t)w, &inc, &div);
    s->div += (uint32_t)div;
    if (s->div & 0x8000u) {
        s->div = 0; w += inc;
        if (w < 0) w = 0;
        if (w > 0x7FFF) w = 0x7FFF;
        s->level = (int16_t)(neg ? -w : w);
    }
}
static inline int16_t vol(uint16_t raw, const Sweep* s) { return (raw & 0x8000u) ? s->level : (int16_t)(uint16_t)(raw << 1); }

/* ---------------------------------------------------------------- voices */
static void decode_block(int i, Voice* v)
{
    static const int f0[5] = {0, 60, 115, 98, 122}, f1[5] = {0, 0, -52, -55, -60};
    uint32_t a = v->cur & (SPU_RAM_SIZE - 16u);
    irq_check(a, 16);
    const uint8_t* b = g_dc_spu_ram + a;
    int shift = b[0] & 15, filt = (b[0] >> 4) & 15;
    if (filt > 4) filt = 0;
    if (shift > 12) shift = 12;
    int k0 = f0[filt], k1 = f1[filt];
    int32_t h1 = v->h1, h2 = v->h2;
    v->prev = v->smp[27];
    for (int n = 0; n < 14; n++) {
        uint8_t p = b[2 + n];
        int32_t s = (int32_t)(int16_t)((uint16_t)(p & 15) << 12) >> shift;
        s += (h1 * k0 + h2 * k1 + 32) >> 6;
        s = clamp16(s); h2 = h1; h1 = s; v->smp[n * 2] = (int16_t)s;
        s = (int32_t)(int16_t)((uint16_t)(p >> 4) << 12) >> shift;
        s += (h1 * k0 + h2 * k1 + 32) >> 6;
        s = clamp16(s); h2 = h1; h1 = s; v->smp[n * 2 + 1] = (int16_t)s;
    }
    v->h1 = (int16_t)h1; v->h2 = (int16_t)h2;
    uint8_t fl = b[1];
    if (fl & 4u) { v->rep = a; s_reg[i * 8 + 7] = (uint16_t)(a >> 3); }
    v->flags = fl;
    v->idx = 0;
    v->cur = (a + 16u) & (SPU_RAM_SIZE - 1u);
    if (fl & 1u) s_endx |= 1u << i;
}

/* One voice over n output frames, added into acc[] (stereo, 32-bit). Each
 * output frame is DIV hardware samples (DIV = 44100 / RATE): the envelope and
 * the volume sweeps step DIV times and the pitch counter advances DIV times
 * as far, so rates and tuning stay those of the 44.1 kHz hardware. */
static void voice_run(int i, Voice* v, int32_t* acc, unsigned n, const int16_t* noise)
{
    const uint32_t pitch = (vreg(i, 2) & 0x3FFFu) * DIV;
    const uint16_t rl = vreg(i, 0), rr = vreg(i, 1);
    const int sweep = ((rl | rr) & 0x8000u) != 0;
    int32_t vl = vol(rl, &s_vsw[i][0]), vr = vol(rr, &s_vsw[i][1]);
    for (unsigned f = 0; f < n; f++) {
        if (v->idx >= 28) {
            if (v->flags & 1u) {
                v->cur = v->rep;
                if (!(v->flags & 2u)) { v->adsr = REL; v->adiv = 0; v->env = 0; v->cok = 0; }
            }
            decode_block(i, v);
        }
        int32_t s;
        if (noise) s = noise[f];
        else {
            int32_t s1 = v->smp[v->idx], s0 = v->idx ? v->smp[v->idx - 1] : v->prev;
            s = s0 + (((s1 - s0) * (int32_t)v->phase) >> 12);
        }
        s = (s * (int32_t)v->env) >> 15;
        for (int k = 0; k < DIV; k++) adsr_run(i, v);
        if (sweep) {
            for (int k = 0; k < DIV; k++) { sweep_step(&s_vsw[i][0], rl); sweep_step(&s_vsw[i][1], rr); }
            vl = vol(rl, &s_vsw[i][0]); vr = vol(rr, &s_vsw[i][1]);
        }
        acc[f * 2] += (s * vl) >> 15;
        acc[f * 2 + 1] += (s * vr) >> 15;
        v->phase += pitch;
        if (v->phase >= 0x1000u) {
            uint32_t adv = (v->phase >> 12) + v->idx;
            v->phase &= 0xFFFu;
            v->idx = (uint8_t)(adv > 28 ? 28 : adv);
        }
        if (v->adsr == REL && v->env == 0) { v->active = 0; return; }
    }
}

/* ---------------------------------------------------------------- AICA path
 * With hardware voices the SH4 does not mix: each key-on decodes the voice's
 * sample (start block to END) to 16-bit PCM once, keeps it in sound RAM and
 * plays it on an AICA channel, looping where the PS1 data loops. The SH4
 * still runs the ADSR envelopes and volume sweeps and sends the resulting
 * volume, pan and pitch to the channel when they change. Not covered: noise
 * voices, reverb, and SPU IRQs raised by sample playback. */
static int s_hw = -1;                     /* -1 unknown, 0 software mix, 1 AICA */
#define NSMP 192
#define MAXBLK 2340                       /* 65520 samples: AICA loop end is 16 bits */
typedef struct {
    uint32_t src, end, rep;               /* SPU RAM: first block, past last block, loop block */
    uint32_t aica;                        /* sound RAM offset, 0 = free entry */
    uint16_t len, loopstart;
    uint8_t loop, stale;
    uint32_t used;                        /* LRU stamp */
} HwSmp;
static HwSmp s_smp[NSMP];
static uint32_t s_smp_clock;
static int16_t s_pcm[28 * 64];
uint32_t g_dc_spu_smp_bytes, g_dc_spu_smp_fail;

static int smp_in_use(int k)
{
    for (int i = 0; i < NV; i++) if (s_v[i].active && s_v[i].smp_i == k) return 1;
    return 0;
}
static void smp_drop(int k)
{
    dc_host_snd_free(s_smp[k].aica);
    g_dc_spu_smp_bytes -= s_smp[k].len * 2u;
    memset(&s_smp[k], 0, sizeof s_smp[k]);
}
/* sound RAM changed: entries decoded from [a, a+len) are out of date */
static void smp_dirty(uint32_t a, uint32_t len)
{
    if (s_hw != 1) return;
    for (int k = 0; k < NSMP; k++) {
        HwSmp* e = &s_smp[k];
        if (!e->aica || e->stale || a >= e->end || a + len <= e->src) continue;
        if (smp_in_use(k)) e->stale = 1; else smp_drop(k);
    }
}
static int smp_get(uint32_t start, uint32_t rep_reg)
{
    /* walk the block flags: length, loop and loop point */
    uint32_t a = start, nblk = 0; int loopblk = -1, loop = 0;
    for (;;) {
        uint8_t fl = g_dc_spu_ram[(a & (SPU_RAM_SIZE - 16u)) + 1];
        if (fl & 4u) loopblk = (int)nblk;
        nblk++;
        a = (a + 16u) & (SPU_RAM_SIZE - 1u);
        if (fl & 1u) { loop = (fl & 2u) != 0; break; }
        if (nblk >= MAXBLK || a == start) break;
    }
    if (loop && loopblk < 0) {
        if (rep_reg >= start && rep_reg < start + nblk * 16u) loopblk = (int)((rep_reg - start) >> 4);
        else loop = 0;
    }
    uint32_t rep = loopblk >= 0 ? start + (uint32_t)loopblk * 16u : 0;
    for (int k = 0; k < NSMP; k++) {
        HwSmp* e = &s_smp[k];
        if (e->aica && !e->stale && e->src == start && e->rep == rep && e->loop == loop) { e->used = ++s_smp_clock; return k; }
    }
    unsigned len = nblk * 28u;
    /* a free entry and sound RAM, evicting the least recently used */
    int slot = -1;
    uint32_t off = 0;
    for (;;) {
        if (slot < 0) for (int k = 0; k < NSMP; k++) if (!s_smp[k].aica) { slot = k; break; }
        if (slot >= 0) { off = dc_host_snd_alloc(len * 2u); if (off) break; }
        int lru = -1;
        for (int k = 0; k < NSMP; k++)
            if (s_smp[k].aica && !smp_in_use(k) && (lru < 0 || s_smp[k].used < s_smp[lru].used)) lru = k;
        if (lru < 0) { g_dc_spu_smp_fail++; return -1; }
        smp_drop(lru);
        if (slot < 0) slot = lru;
    }
    /* decode in chunks straight into the reserved sound RAM */
    static const int f0[5] = {0, 60, 115, 98, 122}, f1[5] = {0, 0, -52, -55, -60};
    int32_t h1 = 0, h2 = 0;
    a = start;
    for (uint32_t b = 0; b < nblk; ) {
        unsigned nb = nblk - b > 64 ? 64 : nblk - b, o = 0;
        for (unsigned j = 0; j < nb; j++, b++) {
            const uint8_t* p = g_dc_spu_ram + (a & (SPU_RAM_SIZE - 16u));
            int shift = p[0] & 15, filt = (p[0] >> 4) & 15;
            if (filt > 4) filt = 0;
            if (shift > 12) shift = 12;
            for (int n = 0; n < 28; n++) {
                int nib = (p[2 + (n >> 1)] >> ((n & 1) * 4)) & 15;
                int32_t x = ((int32_t)(int16_t)((uint16_t)nib << 12) >> shift) + ((h1 * f0[filt] + h2 * f1[filt] + 32) >> 6);
                x = clamp16(x); h2 = h1; h1 = x; s_pcm[o++] = (int16_t)x;
            }
            a = (a + 16u) & (SPU_RAM_SIZE - 1u);
        }
        dc_host_snd_write(off + (b - nb) * 56u, s_pcm, o);
    }
    HwSmp* e = &s_smp[slot];
    e->src = start; e->end = start + nblk * 16u; e->rep = rep;
    e->aica = off; e->len = (uint16_t)len; e->loop = (uint8_t)loop;
    e->loopstart = (uint16_t)(loopblk >= 0 ? loopblk * 28 : 0);
    e->stale = 0; e->used = ++s_smp_clock;
    g_dc_spu_smp_bytes += len * 2u;
    return slot;
}

/* Advance an envelope by n hardware samples without stepping each one. */
static void adsr_advance(int i, Voice* v, uint32_t n)
{
    while (n) {
        if (!v->cok) adsr_calc(i, v);
        if (v->cdiv <= 0) return;
        uint32_t need = (0x8000u - (v->adiv & 0x7FFFu) + (uint32_t)v->cdiv - 1u) / (uint32_t)v->cdiv;
        if (need > n) { v->adiv += n * (uint32_t)v->cdiv; return; }
        n -= need;
        v->adiv = 0x8000u - (uint32_t)v->cdiv;   /* one step short: adsr_run takes the last one */
        adsr_run(i, v);
        if (v->adsr == REL && v->env == 0) return;
    }
}

static inline uint32_t amp(int32_t x) { return (uint32_t)(x < 0 ? -x : x); }
static void hw_out(int i, Voice* v, int on)
{
    int32_t ml = vol(REG(0x1F801D80u), &s_msw[0]), mr = vol(REG(0x1F801D82u), &s_msw[1]);
    uint32_t l = amp(((int32_t)vol(vreg(i, 0), &s_vsw[i][0]) * (int32_t)v->env) >> 15);
    uint32_t r = amp(((int32_t)vol(vreg(i, 1), &s_vsw[i][1]) * (int32_t)v->env) >> 15);
    l = (l * amp(ml)) >> 15; r = (r * amp(mr)) >> 15;
    if (!on) l = r = 0;
    uint32_t loud = l > r ? l : r, quiet = l > r ? r : l;
    unsigned vo = loud * 255u / 32767u, k = 0;
    if (vo > 255) vo = 255;
    /* AICA pan attenuates the quiet side in 3 dB steps */
    if (quiet == 0) k = 15;
    else { uint32_t q = quiet; while (k < 15 && q * 1413u / 1000u <= loud) { q = q * 1413u / 1000u; k++; } }
    unsigned pan = k == 0 ? 128 : (l > r ? 127 - (k << 3) : 128 + (k << 3));
    if (pan > 255) pan = 255;
    uint32_t freq = ((vreg(i, 2) & 0x3FFFu) * 44100u) >> 12;
    if (!freq) freq = 1;
    if (vo != v->hw_vol || pan != v->hw_pan || freq != v->hw_freq) {
        v->hw_vol = (uint16_t)vo; v->hw_pan = (uint16_t)pan; v->hw_freq = freq;
        dc_host_snd_update(i, freq, vo, pan);
    }
}

static void hw_stop(int i, Voice* v)
{
    if (v->hw_on) dc_host_snd_stop(i);
    v->hw_on = 0;
    if (v->smp_i >= 0 && s_smp[v->smp_i].stale) {
        int k = v->smp_i; v->smp_i = -1;
        if (!smp_in_use(k)) smp_drop(k);
    }
    v->smp_i = -1;
}

static void hw_key_on(int i, Voice* v)
{
    int k = smp_get(v->cur, v->rep);
    v->smp_i = (int16_t)k;
    if (k < 0) { v->active = 0; return; }
    adsr_advance(i, v, 64);                  /* the attack's first ~1.5 ms */
    v->hw_vol = 0xFFFF; v->hw_freq = 0;
    hw_out(i, v, (REG(0x1F801DAAu) & 0x8000u) != 0);
    HwSmp* e = &s_smp[k];
    dc_host_snd_start(i, e->aica, e->len, e->loop, e->loopstart, v->hw_freq, v->hw_vol, v->hw_pan);
    uint32_t eon = (uint32_t)REG(0x1F801D98u) | ((uint32_t)REG(0x1F801D9Au) << 16);
    { static unsigned nlog; if (eon && nlog++ < 8) { char t[64]; snprintf(t, sizeof t, "[rev] key-on v%d eon %06X\n", i, (unsigned)eon); dc_host_log(t); } }
#ifdef DC_REV_ALL
    eon = ~0u;
#endif
    dc_host_snd_send(i, (eon >> i) & 1u ? 15u : 0u);
    v->hw_on = 1;
}

/* n hardware samples have passed: envelopes, sweeps, play positions */
/* reverb return level, in the AICA's 3 dB steps, from the PS1 reverb
 * output volume (and off while the game has reverb disabled) */
static unsigned s_rev_lvl = 99;
static void hw_reverb(void)
{
    uint32_t a = amp((int16_t)REG(0x1F801D84u)), b = amp((int16_t)REG(0x1F801D86u));
    uint32_t m = a > b ? a : b;
    unsigned lvl = 0;
    if ((REG(0x1F801DAAu) & 0x80u) && m) {
        lvl = 15;
        uint32_t t = 0x7FFF;
        while (lvl > 1 && m * 1413u / 1000u < t) { t = t * 1000u / 1413u; lvl--; }
    }
    if (lvl != s_rev_lvl) {
        char t[96]; snprintf(t, sizeof t, "[rev] level %u (cnt %04X vout %04X/%04X eon %04X)\n", lvl, REG(0x1F801DAAu),
                             REG(0x1F801D84u), REG(0x1F801D86u), REG(0x1F801D98u));
        dc_host_log(t);
        s_rev_lvl = lvl; dc_host_snd_reverb(lvl);
    }
}

static void hw_run(uint32_t n)
{
    int on = (REG(0x1F801DAAu) & 0x8000u) != 0;
    hw_reverb();
    if ((REG(0x1F801D80u) | REG(0x1F801D82u)) & 0x8000u)
        for (uint32_t j = 0; j < n; j++) { sweep_step(&s_msw[0], REG(0x1F801D80u)); sweep_step(&s_msw[1], REG(0x1F801D82u)); }
    for (int i = 0; i < NV; i++) {
        Voice* v = &s_v[i];
        if (!v->active) continue;
        if ((vreg(i, 0) | vreg(i, 1)) & 0x8000u)
            for (uint32_t j = 0; j < n; j++) { sweep_step(&s_vsw[i][0], vreg(i, 0)); sweep_step(&s_vsw[i][1], vreg(i, 1)); }
        adsr_advance(i, v, n);
        if (v->smp_i >= 0) {
            HwSmp* e = &s_smp[v->smp_i];
            v->pos += (uint64_t)n * (vreg(i, 2) & 0x3FFFu);
            uint32_t p = (uint32_t)(v->pos >> 12);
            if (p >= e->len) {
                s_endx |= 1u << i;
                if (!e->loop) { v->adsr = REL; v->env = 0; v->cok = 0; }
                else { uint32_t ll = e->len - e->loopstart; if (!ll) ll = 1;
                       v->pos = ((uint64_t)(e->loopstart + (p - e->len) % ll) << 12) | (v->pos & 0xFFFu); }
            }
        }
        if (v->adsr == REL && v->env == 0) { v->active = 0; hw_stop(i, v); continue; }
        hw_out(i, v, on);
    }
}

static void key_on(uint32_t m)
{
    for (int i = 0; i < NV; i++) {
        if (!(m & (1u << i))) continue;
        Voice* v = &s_v[i];
        if (s_hw == 1 && v->smp_i >= 0) hw_stop(i, v);
        else if (s_hw == 1 && v->hw_on) dc_host_snd_stop(i);
        memset(v, 0, sizeof *v);
        v->smp_i = -1;
        v->active = 1;
        v->cur = ((uint32_t)(vreg(i, 3) & ~1u) << 3) & (SPU_RAM_SIZE - 1u);
        v->rep = ((uint32_t)(vreg(i, 7) & ~1u) << 3) & (SPU_RAM_SIZE - 1u);
        v->idx = 28;
        v->adsr = ATK;
        s_endx &= ~(1u << i);
        if (s_hw == 1) hw_key_on(i, v);
    }
}
static void key_off(uint32_t m)
{
    for (int i = 0; i < NV; i++)
        if ((m & (1u << i)) && s_v[i].active) { s_v[i].adsr = REL; s_v[i].adiv = 0; s_v[i].cok = 0; }
}

static void noise_clock(uint16_t ctrl)
{
    int step = 4 + ((ctrl >> 8) & 3), shift = (ctrl >> 10) & 15;
    s_noise_t -= step;
    if (s_noise_t < 0) {
        uint16_t n = s_noise;
        uint16_t bit = (uint16_t)(((n >> 15) ^ (n >> 12) ^ (n >> 11) ^ (n >> 10) ^ 1) & 1);
        s_noise = (uint16_t)((n << 1) | bit);
        s_noise_t += 0x20000 >> shift;
        if (s_noise_t < 0) s_noise_t += 0x20000 >> shift;
    }
}

/* ---------------------------------------------------------------- CD input */
#define CDR 8192u                         /* stereo frames, power of two */
static int16_t s_cd[CDR * 2];
static uint32_t s_cd_w, s_cd_r;
static uint32_t s_cd_frac;                /* 16.16 resampler position */
static int16_t s_cd_last[2];

void dc_spu_cd_push(const int16_t* lr, unsigned frames, unsigned rate)
{
    /* resample to 44.1 kHz: step = rate/44100 input frames per output frame */
    uint32_t step = (uint32_t)(((uint64_t)rate << 16) / RATE);
    uint32_t pos = s_cd_frac;               /* position within lr, 16.16 */
    while ((pos >> 16) < frames) {
        uint32_t k = pos >> 16, f = pos & 0xFFFFu;
        for (int c = 0; c < 2; c++) {
            int32_t a = k ? lr[(k - 1) * 2 + c] : s_cd_last[c], b = lr[k * 2 + c];
            if (s_cd_w - s_cd_r < CDR) s_cd[(s_cd_w & (CDR - 1)) * 2 + c] = (int16_t)(a + (((b - a) * (int32_t)f) >> 16));
        }
        if (s_cd_w - s_cd_r < CDR) s_cd_w++;
        pos += step;
    }
    s_cd_frac = pos - ((uint32_t)frames << 16);
    if (frames) { s_cd_last[0] = lr[(frames - 1) * 2]; s_cd_last[1] = lr[(frames - 1) * 2 + 1]; }
}

/* ---------------------------------------------------------------- mixing */
static int16_t s_out[1024 * 2];
static uint64_t s_t0_us, s_done, s_hw_t;          /* frames rendered since s_t0_us */
uint32_t g_dc_spu_peak, g_dc_spu_keyons, g_dc_spu_us, g_dc_spu_frames;

static int32_t s_acc[1024 * 2];
static int16_t s_noisebuf[1024];
static void render(unsigned n)
{
    uint16_t ctrl = REG(0x1F801DAAu);
    int on = (ctrl & 0x8000u) != 0, cd_on = ctrl & 1u;
    int16_t cdl_v = (int16_t)REG(0x1F801DB0u), cdr_v = (int16_t)REG(0x1F801DB2u);
    uint32_t non = (uint32_t)REG(0x1F801D94u) | ((uint32_t)REG(0x1F801D96u) << 16);
    while (n) {
        unsigned m = n > 1024 ? 1024 : n;
        uint32_t act = 0;
        if (on && s_hw != 1) for (int i = 0; i < NV; i++) if (s_v[i].active) act |= 1u << i;
        if (!act && s_cd_r == s_cd_w && !((REG(0x1F801D80u) | REG(0x1F801D82u)) & 0x8000u)) {
            memset(s_out, 0, m * 4u);          /* nothing playing: no per-sample work */
            dc_host_audio_push(s_out, m, RATE);
            n -= m;
            continue;
        }
        memset(s_acc, 0, m * 8u);
        if (act && (non & act)) for (unsigned f = 0; f < m; f++) {
            for (int k = 0; k < DIV; k++) noise_clock(ctrl);
            s_noisebuf[f] = (int16_t)s_noise;
        }
        for (int i = 0; i < NV; i++)
            if (act & (1u << i)) voice_run(i, &s_v[i], s_acc, m, (non & (1u << i)) ? s_noisebuf : 0);
        const uint16_t ml = REG(0x1F801D80u), mr = REG(0x1F801D82u);
        const int msweep = ((ml | mr) & 0x8000u) != 0;
        int32_t mvl = vol(ml, &s_msw[0]), mvr = vol(mr, &s_msw[1]);
        for (unsigned f = 0; f < m; f++) {
            int32_t l = s_acc[f * 2], r = s_acc[f * 2 + 1];
            if (s_cd_r != s_cd_w) {
                const int16_t* c = &s_cd[(s_cd_r & (CDR - 1)) * 2];
                s_cd_r++;
                if (cd_on && on) { l += (c[0] * cdl_v) >> 15; r += (c[1] * cdr_v) >> 15; }
            }
            if (msweep) {
                for (int k = 0; k < DIV; k++) { sweep_step(&s_msw[0], ml); sweep_step(&s_msw[1], mr); }
                mvl = vol(ml, &s_msw[0]); mvr = vol(mr, &s_msw[1]);
            }
            l = (clamp16(l) * mvl) >> 15;
            r = (clamp16(r) * mvr) >> 15;
            if (!on) l = r = 0;
            s_out[f * 2] = clamp16(l); s_out[f * 2 + 1] = clamp16(r);
            uint32_t pk = (uint32_t)(l < 0 ? -l : l);
            if (pk > g_dc_spu_peak) g_dc_spu_peak = pk;
        }
        dc_host_audio_push(s_out, m, RATE);
        n -= m;
    }
}

/* Render everything owed up to now. Called from the I/O poll and before any
 * write that changes what is playing. */
void dc_spu_tick(void)
{
    uint64_t now = dc_host_ticks_us();
    if (!s_t0_us) { s_t0_us = now; s_hw_t = now; if (s_hw < 0) s_hw = dc_host_snd_hw(); return; }
    if (s_hw == 1) {
        uint64_t hn = (now - s_hw_t) * 44100u / 1000000u;
        if (hn) {
            s_hw_t += hn * 1000000u / 44100u;
            if (hn > 44100) hn = 44100;
            hw_run((uint32_t)hn);
        }
    }
    uint64_t want = (now - s_t0_us) * RATE / 1000000u;
    if (want <= s_done) return;
    uint64_t n = want - s_done;
    if (n > 4096) { s_done = want - 4096; n = 4096; }   /* after a long stall */
    s_done += n;
    render((unsigned)n);
    g_dc_spu_us += (uint32_t)(dc_host_ticks_us() - now); g_dc_spu_frames += (uint32_t)n;
}

/* ---------------------------------------------------------------- registers */
uint16_t dc_spu_read(uint32_t p)
{
    if (p >= 0x1F801E00u && p < 0x1F801E60u) {
        uint32_t h = (p - 0x1F801E00u) >> 1;
        int v = (int)(h >> 1), c = (int)(h & 1);
        return v < NV ? (uint16_t)vol(vreg(v, c), &s_vsw[v][c]) : 0;
    }
    if (p < 0x1F801C00u || p > 0x1F801DFFu) return 0;
    uint32_t idx = (p - 0x1F801C00u) >> 1;
    switch (p) {
    case 0x1F801DAEu: { uint16_t c = REG(0x1F801DAAu); return (uint16_t)((c & 0x3Fu) | ((c >> 5) & 1u) << 7 | (s_irq_flag ? 0x40u : 0)); }
    case 0x1F801DB8u: return (uint16_t)vol(REG(0x1F801D80u), &s_msw[0]);
    case 0x1F801DBAu: return (uint16_t)vol(REG(0x1F801D82u), &s_msw[1]);
    case 0x1F801D9Cu: return (uint16_t)s_endx;
    case 0x1F801D9Eu: return (uint16_t)((s_endx >> 16) & 0xFFu);
    case 0x1F801DA6u: return (uint16_t)(s_taddr >> 3);
    }
    if (idx < NV * 8u && (idx & 7u) == 6u) return s_v[idx >> 3].env;
    if (idx < NV * 8u && (idx & 7u) <= 1u && (s_reg[idx] & 0x8000u)) return (uint16_t)s_vsw[idx >> 3][idx & 7u].level;
    return s_reg[idx];
}

void dc_spu_write(uint32_t p, uint16_t v)
{
    if (p < 0x1F801C00u || p > 0x1F801DFFu) return;
    uint32_t idx = (p - 0x1F801C00u) >> 1;
    if (p >= 0x1F801D88u && p <= 0x1F801D8Eu) dc_spu_tick();    /* KON/KOFF */
    else if (idx < NV * 8u && (idx & 7u) == 2u) dc_spu_tick();  /* pitch */
    s_reg[idx] = v;
    if (idx < NV * 8u) {
        int vc = (int)(idx >> 3);
        if ((idx & 7u) == 7u) s_v[vc].rep = ((uint32_t)(v & ~1u) << 3) & (SPU_RAM_SIZE - 1u);
        if ((idx & 7u) <= 1u) sweep_write(&s_vsw[vc][idx & 7u], v);
        if ((idx & 7u) == 4u || (idx & 7u) == 5u) s_v[vc].cok = 0;
        return;
    }
    switch (p) {
    case 0x1F801D80u: sweep_write(&s_msw[0], v); break;
    case 0x1F801D82u: sweep_write(&s_msw[1], v); break;
    case 0x1F801D88u: key_on(v); g_dc_spu_keyons += (unsigned)__builtin_popcount(v); break;
    case 0x1F801D8Au: key_on((uint32_t)(v & 0xFFu) << 16); g_dc_spu_keyons += (unsigned)__builtin_popcount(v & 0xFFu); break;
    case 0x1F801D8Cu: key_off(v); break;
    case 0x1F801D8Eu: key_off((uint32_t)(v & 0xFFu) << 16); break;
    case 0x1F801DA4u: irq_check(s_taddr, 2); break;
    case 0x1F801DA6u: s_taddr = ((uint32_t)v << 3) & (SPU_RAM_SIZE - 1u); irq_check(s_taddr, 2); break;
    case 0x1F801DA8u:
        irq_check(s_taddr, 2);
        memcpy(g_dc_spu_ram + (s_taddr & (SPU_RAM_SIZE - 2u)), &v, 2);
        smp_dirty(s_taddr, 2);
        s_taddr = (s_taddr + 2u) & (SPU_RAM_SIZE - 1u);
        break;
    case 0x1F801DAAu: if (!(v & 0x40u)) s_irq_flag = 0; else irq_check(s_taddr, 2); break;
    }
}

/* DMA channel 4. to_spu: main RAM -> sound RAM. */
void dc_spu_dma(uint8_t* ram, uint32_t madr, uint32_t words, int to_spu)
{
    if (to_spu) smp_dirty(s_taddr, words * 4u);
    for (uint32_t i = 0; i < words; i++, madr += 4) {
        irq_check(s_taddr, 4);
        uint8_t* m = ram + (madr & 0x1FFFFCu);
        uint8_t* s = g_dc_spu_ram + (s_taddr & (SPU_RAM_SIZE - 4u));
        if (to_spu) memcpy(s, m, 4); else memcpy(m, s, 4);
        s_taddr = (s_taddr + 4u) & (SPU_RAM_SIZE - 1u);
    }
}

void dc_spu_reset(void)
{
    memset(s_reg, 0, sizeof s_reg);
    memset(s_v, 0, sizeof s_v);
    for (int i = 0; i < NV; i++) s_v[i].smp_i = -1;
    memset(s_vsw, 0, sizeof s_vsw);
    memset(s_msw, 0, sizeof s_msw);
    s_taddr = 0; s_endx = 0; s_irq_flag = 0;
    s_cd_w = s_cd_r = 0; s_cd_frac = 0;
}
