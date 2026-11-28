/* dc_mem.c - guest memory, slow-path accessors and PS1 I/O for the DC runtime. */
#include "dc_runtime.h"
#include "dc_gpu.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

uint8_t g_dc_ram[0x200000] __attribute__((aligned(32)));
uint8_t g_dc_scratch[1024] __attribute__((aligned(32)));
volatile int g_irq_pending;
DcStats g_dc_stats;

/* ---------------------------------------------------------------- I/O state */
static uint32_t s_istat, s_imask;
static uint32_t s_dma_madr[7], s_dma_bcr[7], s_dma_chcr[7], s_dpcr = 0x07654321u, s_dicr;
static uint32_t s_tmr_mode[3], s_tmr_target[3];
static uint8_t  s_io_other[0x2000];    /* plain storage for registers we do not model */
static uint32_t s_cache_ctrl;

/* SPU: dc_spu.c */
uint16_t dc_spu_read(uint32_t p);
void dc_spu_write(uint32_t p, uint16_t v);
void dc_spu_dma(uint8_t* ram, uint32_t madr, uint32_t words, int to_spu);
void dc_mdec_reset(void);
uint32_t dc_mdec_read(uint32_t p);
void dc_mdec_write(uint32_t p, uint32_t v);
void dc_mdec_dma_in(const uint8_t* ram, uint32_t madr, uint32_t words);
int dc_mdec_decoding(void);
#ifdef DC_LOG
void dc_ev(uint32_t type, uint32_t a, uint32_t b);
#else
#define dc_ev(t, a, b) ((void)0)
#endif
int dc_mdec_dma_out(uint8_t* ram, uint32_t madr, uint32_t words);
static int s_dma1_wait;
static uint64_t s_dma1_wait_t;
static uint64_t s_dma1_due;
static uint32_t s_cd_lba, s_cd_mode;
static int s_cd_burst;
static int s_cd_str;                /* the last sector delivered was a movie (STR) sector */
static uint64_t s_cd_ready_t;        /* when it was delivered */
static uint64_t s_dma1_done_t;       /* when the last MDEC output DMA finished */
void dc_spu_cd_push(const int16_t* lr, unsigned frames, unsigned rate);
void dc_spu_tick(void);
void dc_spu_reset(void);

/* CD-ROM controller: enough to answer commands with a status byte. */
static uint8_t  s_cd_index, s_cd_ie, s_cd_if, s_cd_stat = 0x02;
static uint8_t  s_cd_param[16], s_cd_nparam, s_cd_resp[16], s_cd_nresp, s_cd_rpos;

void dc_io_reset(void)
{
    s_istat = s_imask = 0;
    memset(s_dma_madr, 0, sizeof s_dma_madr);
    memset(s_dma_bcr, 0, sizeof s_dma_bcr);
    memset(s_dma_chcr, 0, sizeof s_dma_chcr);
    s_dicr = 0;
    memset(s_io_other, 0, sizeof s_io_other);
    dc_gpu_reset();
    dc_spu_reset();
    dc_mdec_reset(); s_dma1_wait = 0; s_dma1_due = 0;
}

uint32_t dc_io_istat(void) { return s_istat; }
uint32_t dc_io_imask(void) { return s_imask; }
void dc_io_ack(uint32_t bits) { s_istat &= ~bits; }

void dc_irq_raise(uint32_t bits)
{
    s_istat |= bits;
    if (s_istat & s_imask) g_irq_pending = 1;
}

static void log_io(const char* what, uint32_t a, uint32_t v)
{
    static int n;
    if (n++ < 64) {
        char b[96];
        snprintf(b, sizeof b, "[io] %s %08X = %08X\n", what, (unsigned)a, (unsigned)v);
        dc_host_log(b);
    }
}

/* ---------------------------------------------------------------- timers */
/* Counters run off host time. Each timer keeps a 64-bit tick origin; the
 * guest sees (ticks - origin) wrapped at the target (reset-at-target mode) or
 * at 0x10000. Target and overflow IRQs are raised from dc_io_poll: the sound
 * driver ticks its sequencer from a root-counter IRQ. */
static uint64_t s_tmr_org[3], s_tmr_next[3];
static uint32_t s_tmr_period[3], s_tmr_armed;
uint32_t g_dc_tmr_irqs[3];
static uint64_t tmr_now(int t)
{
    uint64_t us = dc_host_ticks_us();
    uint32_t src = (s_tmr_mode[t] >> 8) & 3u;
    if (t == 2 && (src & 2u)) return us * 4233600u / 1000000u;     /* sysclk/8 */
    if (t == 1 && (src & 1u)) return us * 15734u / 1000000u;       /* hblank */
    if (t == 0 && (src & 1u)) return us * 5320u / 1000u;           /* ~dot clock */
    return us * 33868800u / 1000000u;                              /* sysclk */
}
static uint32_t tmr_count(int t)
{
    uint64_t d = tmr_now(t) - s_tmr_org[t];
    uint32_t tg = s_tmr_target[t];
    if ((s_tmr_mode[t] & 8u) && tg) return (uint32_t)(d % (tg + 1u));
    return (uint32_t)d & 0xFFFFu;
}
static void tmr_arm(int t)
{
    uint32_t m = s_tmr_mode[t], tg = s_tmr_target[t], per = 0;
    if ((m & 0x10u) && tg) per = (m & 8u) ? tg + 1u : 0x10000u;
    else if (m & 0x20u) per = 0x10000u;
    s_tmr_period[t] = per;
    if (per) { s_tmr_armed |= 1u << t; s_tmr_next[t] = s_tmr_org[t] + ((m & 0x10u) && tg ? tg : per); }
    else s_tmr_armed &= ~(1u << t);
}
static unsigned s_tmr_owed[3];
static void tmr_poll(void)
{
    for (int t = 0; t < 3; t++) {
        if (!(s_tmr_armed & (1u << t))) continue;
        uint64_t now = tmr_now(t);
        while (now >= s_tmr_next[t] && s_tmr_owed[t] < 8) {
            s_tmr_owed[t]++;
            if (!(s_tmr_mode[t] & 0x40u)) { s_tmr_armed &= ~(1u << t); break; }
            s_tmr_next[t] += s_tmr_period[t];
        }
        if (now >= s_tmr_next[t]) s_tmr_next[t] = now + s_tmr_period[t];   /* long stall */
    }
    /* one tick per timer per poll, each once the last one was acknowledged */
    for (int t = 0; t < 3; t++) {
        uint32_t bit = 1u << (4 + t);
        if (!s_tmr_owed[t] || (s_istat & bit)) continue;
        s_tmr_owed[t]--; g_dc_tmr_irqs[t]++;
        s_tmr_mode[t] |= ((s_tmr_mode[t] & 0x10u) ? 0x800u : 0x1000u);
        dc_irq_raise(bit);
    }
}

/* ---------------------------------------------------------------- DMA */
static void dma_finish(int ch)
{
    s_dma_chcr[ch] &= ~((1u << 24) | (1u << 28));
    uint32_t en = 1u << (16 + ch);
    if (s_dicr & en) {
        s_dicr |= 1u << (24 + ch);
        if ((s_dicr & (1u << 23)) && ((s_dicr >> 24) & (s_dicr >> 16) & 0x7Fu)) {
            s_dicr |= 0x80000000u;
            dc_irq_raise(1u << 3);
        }
    }
}

uint32_t dc_cd_fifo_read(uint8_t* dst, uint32_t n);
uint32_t dc_gpu_load_words(const uint8_t* src, uint32_t n);

static void dma_start_(int ch);
static void dma_start(int ch)
{
    DC_PHASE(ch == 2 ? DC_PH_GPU : ch == 3 ? DC_PH_CD : ch == 4 ? DC_PH_SPU : ch <= 1 ? DC_PH_MDEC : g_dc_phase);
    dma_start_(ch);
    DC_PHASE_END();
}
static void dma_start_(int ch)
{
    uint32_t madr = s_dma_madr[ch] & 0x1FFFFCu, bcr = s_dma_bcr[ch], chcr = s_dma_chcr[ch];
    if (ch == 0 || ch == 1) {            /* MDEC in / out */
        uint32_t words = (bcr & 0xFFFFu) * ((bcr >> 16) ? (bcr >> 16) : 1u);
        if (!words) words = 0x10000u;
        if (ch == 0) {
            dc_ev('I', madr, words); dc_mdec_dma_in(g_dc_ram, madr, words);
            dma_finish(0);
            if (s_dma1_wait) { s_dma1_wait = 0; dma_start(1); }
            return;
        }
        dc_ev('O', madr, words);
        if (!dc_mdec_dma_out(g_dc_ram, madr, words)) { dc_ev('W', madr, words); if (!s_dma1_wait) s_dma1_wait_t = dc_host_ticks_us(); s_dma1_wait = 1; return; }
        s_dma1_wait = 0;
        /* The data is in place, but the channel finishes later, as the real
         * MDEC's would: a movie player clears its "frame decoded" flag right
         * after starting the first slice, and a whole frame finishing before
         * that left the flag cleared and the player waiting forever. */
        s_dma1_due = dc_host_ticks_us() + 20u + words / 64u;
        g_irq_pending = 1;               /* poll until it is due */
        return;
    } else if (ch == 6) {                       /* OTC: build a reverse linked list */
        uint32_t n = bcr & 0xFFFFu; if (!n) n = 0x10000u;
        uint32_t a = madr;
        for (uint32_t i = 0; i < n; i++, a -= 4) {
            uint32_t v = (i == n - 1) ? 0x00FFFFFFu : ((a - 4) & 0x1FFFFFu);
            memcpy(g_dc_ram + (a & 0x1FFFFFu), &v, 4);
        }
    } else if (ch == 2 && ((chcr >> 9) & 3u) == 2u) {   /* GPU linked list */
        uint32_t a = madr, guard = 0;
        while (guard++ < 0x40000u) {
            uint32_t hdr; memcpy(&hdr, g_dc_ram + (a & 0x1FFFFCu), 4);
            for (uint32_t i = 1; i <= (hdr >> 24); i++) {
                uint32_t w; memcpy(&w, g_dc_ram + ((a + 4 * i) & 0x1FFFFCu), 4);
                dc_gpu_gp0(w);
            }
            if (hdr & 0x800000u) break;
            a = hdr & 0x1FFFFCu;
        }
        g_dc_stats.dma_gpu++;
    } else if (ch == 3) {                /* CD-ROM -> RAM */
        uint32_t words = (bcr & 0xFFFFu) * ((bcr >> 16) ? (bcr >> 16) : 1u);
        if (!words) words = 0x10000u;
        dc_ev('T', madr, words);
        uint32_t a = madr;
        for (uint32_t i = 0; i < words; i++, a += 4) {
            uint8_t w[4]; dc_cd_fifo_read(w, 4);
            memcpy(g_dc_ram + (a & 0x1FFFFCu), w, 4);
        }
    } else if (ch == 4) {                /* SPU: RAM <-> sound RAM */
        uint32_t words = (bcr & 0xFFFFu) * ((bcr >> 16) ? (bcr >> 16) : 1u);
        dc_spu_dma(g_dc_ram, madr, words, (int)(chcr & 1u));
    } else if (ch == 2) {                /* GPU block transfer */
        uint32_t words = (bcr & 0xFFFFu) * ((bcr >> 16) ? (bcr >> 16) : 1u);
        uint32_t a = madr;
        for (uint32_t i = 0; i < words; i++, a += 4) {
            if (chcr & 1u) {
                uint32_t pa = a & 0x1FFFFCu, left = words - i;
                if (pa + left * 4u > 0x200000u) left = (0x200000u - pa) / 4u;
                uint32_t k = dc_gpu_load_words(g_dc_ram + pa, left);
                if (k) { i += k - 1u; a += (k - 1u) * 4u; continue; }
                uint32_t w; memcpy(&w, g_dc_ram + pa, 4); dc_gpu_gp0(w); }
            else { uint32_t w = dc_gpu_read(); memcpy(g_dc_ram + (a & 0x1FFFFCu), &w, 4); }
        }
        g_dc_stats.dma_gpu++;
    } else {
        log_io("dma start", (uint32_t)ch, chcr);
    }
    dma_finish(ch);
}

/* ---------------------------------------------------------------- CD-ROM */
typedef struct { uint8_t irq, n, r[8]; } CdResp;
static CdResp s_cd_pend[4];
static int s_cd_npend;

static int s_cd_trace = -1;
static void cd_deliver(const CdResp* r)
{
    if (s_cd_trace < 0) s_cd_trace = getenv("DC_CD_TRACE") != 0;
    if (s_cd_trace) { char b[64]; snprintf(b, sizeof b, "[cd] INT%u n=%u r0=%02X ie=%02X\n", r->irq, r->n, r->r[0], s_cd_ie); dc_host_log(b); }
    s_cd_nresp = r->n; s_cd_rpos = 0;
    memcpy(s_cd_resp, r->r, r->n);
    s_cd_if = r->irq;
    if (s_cd_ie & r->irq) dc_irq_raise(1u << 2);
}

/* Queue a response; the first one is delivered now, later ones each time the
 * game acknowledges the previous interrupt (as the real controller does). */
static void cd_respond(uint8_t irq, int n, const uint8_t* r)
{
    CdResp x; x.irq = irq; x.n = (uint8_t)n; memcpy(x.r, r, (size_t)n);
    if (s_cd_if == 0 && s_cd_npend == 0) cd_deliver(&x);
    else if (s_cd_npend < 4) s_cd_pend[s_cd_npend++] = x;
}

/* Sector reading: Setloc target, ReadN/ReadS stream one INT1 per sector,
 * paced by the game's acknowledgements. */

static int s_cd_reading, s_cd_sector_ready, s_cd_have;
static uint8_t s_cd_sector[2352];
static uint8_t s_cd_fifo[2352];
static uint32_t s_cd_fifo_len, s_cd_fifo_pos;

static uint32_t bcd(uint8_t v) { return (uint32_t)((v >> 4) * 10 + (v & 15)); }

/* ---- XA-ADPCM: streamed audio sectors, decoded to PCM for the host ----- */
static uint8_t s_xa_file, s_xa_chan, s_cd_muted;
static int32_t s_xa_hist[2][2];          /* per channel: last two samples */
static int16_t s_xa_pcm[18 * 8 * 28 * 2];  /* one sector, stereo frames */
static void xa_decode(const uint8_t* sec)
{
    static const int32_t k0[4] = {0, 60, 115, 98}, k1[4] = {0, 0, -52, -55};
    const uint8_t coding = sec[19];
    const int stereo = coding & 1, rate = (coding & 4) ? 18900 : 37800, eight = (coding & 0x10) != 0;
    const uint8_t* d = sec + 24;
    int16_t* o = s_xa_pcm;
    unsigned frames = 0;
    int16_t mono[28];
    for (int g = 0; g < 18; g++, d += 128) {
        const int units = eight ? 4 : 8;
        for (int u = 0; u < units; u++) {
            const uint8_t h = d[4 + u];
            const int shift = h & 15, f = (h >> 4) & 3, ch = stereo ? (u & 1) : 0;
            int32_t* hist = s_xa_hist[ch];
            int16_t* dst = stereo ? o + (u >> 1) * 28 * 2 + ch : mono;
            for (int j = 0; j < 28; j++) {
                int32_t t;
                if (eight) t = (int32_t)(int8_t)d[16 + j * 4 + u] << 8;
                else t = (int32_t)(int16_t)((uint16_t)((d[16 + j * 4 + (u >> 1)] >> ((u & 1) * 4)) & 15) << 12);
                t = (t >> (shift > 12 ? 9 : shift)) + ((hist[0] * k0[f] + hist[1] * k1[f] + 32) >> 6);
                t = t < -32768 ? -32768 : t > 32767 ? 32767 : t;
                hist[1] = hist[0]; hist[0] = t;
                if (stereo) dst[j * 2] = (int16_t)t;
                else dst[j] = (int16_t)t;
            }
            if (!stereo) { for (int j = 0; j < 28; j++) { o[0] = o[1] = mono[j]; o += 2; } frames += 28; }
            else if (u & 1) frames += 28;
        }
        if (stereo) o += units / 2 * 28 * 2;
    }
    if (!s_cd_muted) dc_spu_cd_push(s_xa_pcm, frames, (unsigned)rate);
}

#ifdef DC_LOG
static uint32_t s_evl[400][4]; static unsigned s_evn;
void dc_ev(uint32_t type, uint32_t a, uint32_t b)
{ if (type == 'O' || type == 'D' || (type == 'Q' && a == 0x8004B6BCu)) return;
  if (type == 'S' && s_evn && s_evl[(s_evn - 1) % 400][1] == 'S' && s_evn > 1 && s_evl[(s_evn - 2) % 400][1] == 'S') { s_evl[(s_evn - 1) % 400][3]++; return; }
  uint32_t* e = s_evl[s_evn++ % 400]; e[0] = (uint32_t)(dc_host_ticks_us() / 10u); e[1] = type; e[2] = a; e[3] = b; }
static void dc_ev_dump(void)
{ char b[96]; for (unsigned i = 0; i < 400; i++) { uint32_t* e = s_evl[(s_evn + i) % 400]; if (!e[0]) continue;
    snprintf(b, sizeof b, "[ev] %u.%02ums %c %08X %08X\n", (unsigned)(e[0] / 100), (unsigned)(e[0] % 100), (char)e[1], (unsigned)e[2], (unsigned)e[3]); dc_host_log(b); } }
#endif
#ifdef DC_LOG
static uint32_t s_sl[32][5]; static unsigned s_sln;
static uint32_t rd32(uint32_t a) { uint32_t v; memcpy(&v, g_dc_ram + (a & 0x1FFFFCu), 4); return v; }
void dc_cd_dump_recent(void)
{
    char b[160];
    dc_ev_dump();
    for (unsigned i = 0; i < 32; i++) { uint32_t* e = s_sl[(s_sln + i) & 31]; if (!e[0]) continue;
        snprintf(b, sizeof b, "[sec] t=%u.%ums lba=%u sn=%u/%u frame=%u hdr=%08X\n", (unsigned)(e[0] / 10), (unsigned)(e[0] % 10), (unsigned)e[1], (unsigned)(e[2] & 0xFFFF), (unsigned)(e[2] >> 16), (unsigned)e[3], (unsigned)e[4]); dc_host_log(b); }
    uint32_t tab = rd32(0x801E8A14u), n = rd32(0x801E8A18u);
    snprintf(b, sizeof b, "[ring] tab=%08X n=%u w=%08X r=%08X now=%ums dma1due=%d wait=%d istat=%03X\n", (unsigned)tab, (unsigned)n, (unsigned)rd32(0x801E89F8u), (unsigned)rd32(0x801E8A00u),
             (unsigned)(dc_host_ticks_us() / 1000u), s_dma1_due != 0, s_dma1_wait, (unsigned)s_istat); dc_host_log(b);
    for (uint32_t i = 0; i < n && i < 32; i++) {
        uint32_t a = tab + i * 32u;
        snprintf(b, sizeof b, "[slot %u] %08X %08X %08X %08X %08X %08X %08X %08X\n", (unsigned)i, (unsigned)rd32(a), (unsigned)rd32(a + 4), (unsigned)rd32(a + 8), (unsigned)rd32(a + 12),
                 (unsigned)rd32(a + 16), (unsigned)rd32(a + 20), (unsigned)rd32(a + 24), (unsigned)rd32(a + 28)); dc_host_log(b); }
}
#endif
/* Xenogears' movie library (an overlay at 801D3000) keeps its sector ring
 * here: a table of 32-byte slot headers (status in the low half of the first
 * word, 0 = free), its slot count and the write index. */
#define XG_STR_TAB  0x801E8A14u
#define XG_STR_N    0x801E8A18u
#define XG_STR_W    0x801E89F8u
static int s_cd_hold;
static uint32_t ram32(uint32_t a) { uint32_t v; memcpy(&v, g_dc_ram + (a & 0x1FFFFCu), 4); return v; }
static int str_ring_room(const uint8_t* h)
{
    if (!(h[0] == 0x60 && h[1] == 0x01 && h[2] == 0x01 && h[3] == 0x80)) return 1;
    uint32_t sn = (uint32_t)(h[4] | h[5] << 8), sc = (uint32_t)(h[6] | h[7] << 8);
    if (sn != 0 || !sc) return 1;
    uint32_t tab = ram32(XG_STR_TAB), n = ram32(XG_STR_N), w = ram32(XG_STR_W);
    if ((tab >> 24) != 0x80u || n < 8u || n > 64u || w >= n || sc >= n) return 1;   /* not that player */
    uint32_t at = (n - w - 1u >= sc) ? w : 0u;      /* the player wraps a frame that does not fit */
    for (uint32_t i = 0; i < sc; i++)
        if (ram32(tab + (at + i) * 32u) & 0xFFFFu) return 0;
    return 1;
}

static void cd_next_sector(void)
{
    static uint8_t xb[2352];
    if (dc_host_cd_read_sector(s_cd_lba, xb) != 0) {
        static const uint8_t err[2] = {0x03, 0x80};
        s_cd_reading = 0;
        cd_respond(5, 2, err);
        dc_host_log("[cd] read failed: no disc image or sector out of range\n");
        return;
    }
    s_cd_lba++;
    /* With XA-ADPCM on, audio sectors (mode 2, submode audio) go to the
       decoder, if they pass the file/channel filter, and raise nothing. They
       are read aside: a data sector the game is still to read stays put. */
    if ((s_cd_mode & 0x40) && xb[15] == 2 && (xb[18] & 0x04)) {
        if (!(s_cd_mode & 0x08) || (xb[16] == s_xa_file && xb[17] == s_xa_chan))
            xa_decode(xb);
        return;
    }
    /* The first sector of a movie frame waits until the player's sector
     * ring has room for the whole frame. On a PS1 the decoder keeps ahead of
     * the drive; here it can fall behind, the ring fills, and the player's
     * handling of a full ring (a frame start dropped where the ring wraps)
     * leaves a stray frame header that its reader later takes for a frame,
     * running off the end of the ring. The stream, with its interleaved XA
     * audio, then waits for the player, as a slower drive would. */
    if ((s_cd_mode & 0x40) && !str_ring_room(xb + 24)) { s_cd_lba--; if (!s_cd_hold) { dc_ev('H', s_cd_lba, 0);
#ifdef DC_LOG
        { static int nh; if (nh++ < 4) { char b[200]; int o = snprintf(b, sizeof b, "[hold] w=%u st:", (unsigned)ram32(XG_STR_W)); uint32_t tab = ram32(XG_STR_TAB);
          for (int i = 0; i < 32 && o < 190; i++) o += snprintf(b + o, sizeof b - o, "%X", (unsigned)(ram32(tab + i * 32u) & 0xF)); snprintf(b + o, sizeof b - o, "\n"); dc_host_log(b); } }
#endif
        } s_cd_hold = 1; return; }
    s_cd_hold = 0;
    memcpy(s_cd_sector, xb, sizeof xb);
    s_cd_sector_ready = 1; s_cd_have = 1;
    s_cd_stat = 0x22;
    /* A movie frame (STR: a 0x0160/0x8001 header, sector n of m) is read in
     * one burst once its first sector is in: the game starts decoding a
     * frame as soon as it begins to arrive, counting on the PS1's slower CPU
     * to never catch up with the drive. The schedule still advances one
     * period per sector, so the stream keeps its real-time rate. */
    {
        const uint8_t* h = s_cd_sector + 24;
        uint16_t sn = (uint16_t)(h[4] | h[5] << 8), sc = (uint16_t)(h[6] | h[7] << 8);
        s_cd_str = (s_cd_mode & 0x40) && h[0] == 0x60 && h[1] == 0x01 && h[2] == 0x01 && h[3] == 0x80;
        s_cd_burst = s_cd_str && sn + 1u < sc;
        s_cd_ready_t = dc_host_ticks_us();
    }
#ifdef DC_LOG
    { const uint8_t* h = s_cd_sector + 24; uint32_t* e = s_sl[s_sln++ & 31];
      e[0] = (uint32_t)(dc_host_ticks_us() / 100u); e[1] = s_cd_lba - 1u;
      e[2] = (uint32_t)(h[4] | h[5] << 8) | (uint32_t)(h[6] | h[7] << 8) << 16;
      e[3] = (uint32_t)(h[8] | h[9] << 8 | h[10] << 16 | (uint32_t)h[11] << 24); e[4] = (uint32_t)(h[0] | h[1] << 8 | h[2] << 16 | (uint32_t)h[3] << 24); }
#endif
    dc_ev('S', s_cd_lba - 1u, (uint32_t)(s_cd_sector[24 + 4] | s_cd_sector[24 + 8] << 8));
    cd_respond(1, 1, &s_cd_stat);
}

/* Poll at back-edges, not only at vblank, while a movie streams: its
 * sectors are due every 6.7 ms and a frame's come in a burst. */
/* A movie frame still arriving in a burst: the rest of its sectors are due
 * now. A movie player whose decoding runs natively (much faster than on a
 * PS1) calls this before decoding, so it never runs ahead of its sectors. */
int dc_cd_frame_arriving(void) { return s_cd_reading && s_cd_str && s_cd_burst; }
int dc_io_cd_kick(void) { return s_dma1_due || (s_cd_reading && (s_cd_mode & 0x40) && s_cd_if == 0 && !s_cd_npend); }
static void cd_ack(uint8_t bits)
{
    s_cd_if &= ~bits;
    if (dc_io_cd_kick()) g_irq_pending = 1;
    if (s_cd_if == 0 && s_cd_npend) {
        CdResp x = s_cd_pend[0];
        memmove(s_cd_pend, s_cd_pend + 1, sizeof(CdResp) * (size_t)--s_cd_npend);
        cd_deliver(&x);
    }
}

/* Called from the interrupt path: deliver the next sector of a running read
 * once the drive would have it (75 sectors/s, 150 at double speed). */
static uint64_t s_cd_next_us;
int dc_dma1_debug(void) { return s_dma1_wait * 2 + (s_dma1_due != 0); }
void dc_cd_debug(char* b, unsigned n)
{
    snprintf(b, n, "[cd-state] if=%02X ie=%02X reading=%d npend=%d ready=%d lba=%u mode=%02X istat=%03X imask=%03X\n",
             s_cd_if, s_cd_ie, s_cd_reading, s_cd_npend, s_cd_sector_ready, (unsigned)s_cd_lba,
             (unsigned)s_cd_mode, (unsigned)dc_io_istat(), (unsigned)dc_io_imask());
}

/* VBlanks the host counted while the guest was busy are handed over one at
 * a time, each once the guest has acknowledged the previous one: a game
 * counting VBlanks (VSync(n) waits, frame timing) then keeps real time even
 * when a poll comes late, instead of losing every VBlank but one. */
static unsigned s_vbl_owed;
unsigned dc_io_vblanks_owed(void) { return s_vbl_owed + ((s_imask >> 4) & 7u ? s_tmr_owed[0] + s_tmr_owed[1] + s_tmr_owed[2] : 0u); }
void dc_io_poll(void)
{
    s_vbl_owed += dc_host_take_vblanks();
    if (s_vbl_owed > 8) s_vbl_owed = 8;      /* after a long stall (loading) */
    if (s_vbl_owed && !(s_istat & 1u)) { s_vbl_owed--; dc_irq_raise(1u); dc_host_vblank(); }
    tmr_poll();
    dc_spu_tick();
    if (s_dma1_due && dc_host_ticks_us() >= s_dma1_due) { s_dma1_due = 0; s_dma1_done_t = dc_host_ticks_us(); dc_ev('D', 1, 0); dma_finish(1); }
    /* An output DMA left waiting for a decode that never comes (a movie that
     * ended between starting a slice and sending its data) would hold the
     * player forever: after a second the channel ends with what it has. */
    if (s_dma1_wait && dc_host_ticks_us() - s_dma1_wait_t > 1000000u) { s_dma1_wait = 0; dc_ev('w', 1, 0); dma_finish(1); }
#ifdef DC_LOG
    { static int dumped; if (s_dma1_wait && !dumped && dc_host_ticks_us() - s_dma1_wait_t > 2000000u) { uint32_t dc_last_poll_pc(void); char b[96]; dumped = 1;
        snprintf(b, sizeof b, "[stall] dma1 wait pc=%08X chcr1=%08X\n", (unsigned)dc_last_poll_pc(), (unsigned)s_dma_chcr[1]); dc_host_log(b); dc_cd_dump_recent(); } }
#endif
#ifdef DC_LOG
    if (s_cd_reading && (s_cd_mode & 0x40)) { uint32_t tab = rd32(0x801E8A14u), r = rd32(0x801E8A00u), w = rd32(0x801E89F8u);
      if ((tab >> 24) == 0x80 && r < 64) { static uint32_t lr = ~0u, l26 = ~0u, lw = ~0u; uint32_t st = rd32(tab + r * 32u) & 0xFFFF, s26 = rd32(tab + 26 * 32u) & 0xFFFF;
        if (r != lr || s26 != l26 || w != lw) { lr = r; l26 = s26; lw = w; dc_ev('Z', r << 16 | w, st << 16 | s26); } } }
#endif
    if (!s_cd_reading || s_cd_if != 0 || s_cd_npend) return;
    /* A movie player drops any sector that arrives while the MDEC output
     * DMA is busy (it checks the channel's busy bit in its sector handler),
     * and then frees more of its ring than was written. The drive keeps the
     * sector in its buffer until the slice is out. */
    uint64_t now = dc_host_ticks_us();
    if (s_dma1_due) return;
    if (s_dma1_wait && now - s_dma1_wait_t < 50000u) return;
    /* nor before the game has seen the last DMA's completion: a movie
     * player marks a frame complete from the DMA3 interrupt, and a sector
     * handled first is dropped (or the frame flag lands on the next one) */
    if (s_istat & 8u) return;
    /* nor while a movie sector put off because the MDEC was busy is still
     * unread: the player reads it from its DMA1 callback, and a new sector in
     * the drive's buffer would replace it, losing part of a frame (the title
     * movie then never sees its last frame and waits forever). Sectors the
     * game ignores outright are not waited for. */
    if (s_cd_str && s_cd_sector_ready && s_dma1_done_t >= s_cd_ready_t && now - s_dma1_done_t < 3000u) return;
    uint32_t per = (s_cd_mode & 0x80) ? 6667u : 13333u;
    /* Early (burst) delivery only once the game has read the last sector:
     * its handler acknowledges the interrupt before it asks for the data. */
    if (now < s_cd_next_us && !(s_cd_burst && !s_cd_sector_ready && s_cd_next_us - now < 16u * per)) return;
    if (now > s_cd_next_us + 4u * per) s_cd_next_us = now;     /* after a stall: no catching up */
    s_cd_next_us += per;
    cd_next_sector();
}

static void cd_load_fifo(void)
{
    if (s_cd_trace > 0) { char b[48]; snprintf(b, sizeof b, "[cd] fifo load ready=%d\n", s_cd_sector_ready); dc_host_log(b); }
    /* The drive keeps the sector in its buffer: asking for the data again
     * (a movie player's deferred sector handler does) reads it again. */
    if (!s_cd_sector_ready && !(s_cd_have && s_cd_fifo_len == 0)) return;
    if (s_cd_mode & 0x20) { memcpy(s_cd_fifo, s_cd_sector + 12, 2340); s_cd_fifo_len = 2340; }
    else { memcpy(s_cd_fifo, s_cd_sector + 24, 2048); s_cd_fifo_len = 2048; }
    s_cd_fifo_pos = 0;
    dc_ev('F', s_cd_lba - 1u, s_cd_sector_ready);
    s_cd_sector_ready = 0;
}

uint32_t dc_cd_fifo_read(uint8_t* dst, uint32_t n)
{
    uint32_t i = 0;
    for (; i < n && s_cd_fifo_pos < s_cd_fifo_len; i++) dst[i] = s_cd_fifo[s_cd_fifo_pos++];
    for (; i < n; i++) dst[i] = 0;
    return n;
}

static void cd_command(uint8_t cmd)
{
    char b[80];
    g_dc_stats.cd_cmds++;
    if (cmd == 0x02 && s_cd_nparam >= 3)
        snprintf(b, sizeof b, "[cd] Setloc %02X:%02X:%02X\n", s_cd_param[0], s_cd_param[1], s_cd_param[2]);
    else
        snprintf(b, sizeof b, "[cd] cmd %02X (%d params)\n", cmd, s_cd_nparam);
    if (cmd != 0x01 && g_dc_stats.cd_cmds < 400) dc_host_log(b);
    if (cmd == 0x0E || cmd == 0x0D) { static int nl; if (nl < 200) { nl++; snprintf(b, sizeof b, "[cd] %s %02X %02X\n", cmd == 0x0E ? "Setmode" : "Setfilter", s_cd_param[0], s_cd_param[1]); dc_host_log(b); } }
    /* status, filter, mute and position queries leave a read (an XA stream)
       running; other commands end it */
    const int keeps = cmd == 0x01 || cmd == 0x0B || cmd == 0x0C || cmd == 0x0D || cmd == 0x10 || cmd == 0x11 ||
                      cmd == 0x13 || cmd == 0x14;
    if (s_cd_reading && !keeps) { s_cd_reading = 0; s_cd_npend = 0; }
    uint8_t st = s_cd_stat = s_cd_reading ? 0x22 : 0x02;
    switch (cmd) {
    case 0x02:                           /* Setloc mm:ss:ff (BCD) */
        if (s_cd_nparam >= 3)
            s_cd_lba = (bcd(s_cd_param[0]) * 60 + bcd(s_cd_param[1])) * 75 + bcd(s_cd_param[2]) - 150;
#ifdef DC_LOG
        { static uint32_t last; static int nd; if (s_cd_lba == last && (s_cd_mode & 0x40) && nd++ == 0) dc_cd_dump_recent(); last = s_cd_lba; }
#endif
#ifdef DC_LOG
        
#endif
        cd_respond(3, 1, &st); break;
    case 0x0E: s_cd_mode = s_cd_param[0]; cd_respond(3, 1, &st); break;   /* Setmode */
    case 0x0D: s_xa_file = s_cd_param[0]; s_xa_chan = s_cd_param[1]; cd_respond(3, 1, &st); break;  /* Setfilter */
    case 0x0B: s_cd_muted = 1; cd_respond(3, 1, &st); break;   /* Mute */
    case 0x0C: s_cd_muted = 0; cd_respond(3, 1, &st); break;   /* Demute */
    case 0x10: cd_respond(3, 8, s_cd_sector + 12); break;      /* GetlocL: header and subheader */
    case 0x11: {                         /* GetlocP: track 1, position of the last sector */
        uint32_t l = s_cd_lba ? s_cd_lba - 1 : 0, a = l + 150;
        #define BCD(v) (uint8_t)(((v) / 10) << 4 | ((v) % 10))
        uint8_t r[8] = {1, 1, BCD(l / 4500), BCD(l / 75 % 60), BCD(l % 75), BCD(a / 4500), BCD(a / 75 % 60), BCD(a % 75)};
        #undef BCD
        cd_respond(3, 8, r); break;
    }
    case 0x19: {                         /* Test */
        static const uint8_t ver[4] = {0x94, 0x09, 0x19, 0xC0};
        cd_respond(3, 4, ver); break;
    }
    case 0x1A: {                         /* GetID: licensed data disc (NTSC-U) */
        static const uint8_t id[8] = {0x02, 0x00, 0x20, 0x00, 'S', 'C', 'E', 'A'};
        cd_respond(3, 1, &st); cd_respond(2, 8, id); break;
    }
    case 0x07: case 0x08: case 0x09: case 0x0A: case 0x15: case 0x16:
        cd_respond(3, 1, &st); cd_respond(2, 1, &st); break;   /* MotorOn/Stop/Pause/Init/SeekL/SeekP */
    case 0x06: case 0x1B:                /* ReadN/ReadS */
        cd_respond(3, 1, &st);
        s_cd_reading = 1;
        s_cd_next_us = dc_host_ticks_us() + 20000u;   /* seek time */
        s_cd_burst = 0; s_cd_have = 0; s_cd_str = 0;
        break;
    default:
        cd_respond(3, 1, &st);
        break;
    }
    s_cd_nparam = 0;
}

static uint32_t cd_read(uint32_t reg)
{
    switch (reg) {
    case 0: return (uint32_t)(s_cd_index | (s_cd_nparam == 0 ? 0x08 : 0) | 0x10 |
                              (s_cd_rpos < s_cd_nresp ? 0x20 : 0) |
                              (s_cd_fifo_pos < s_cd_fifo_len ? 0x40 : 0));
    case 1: return s_cd_rpos < s_cd_nresp ? s_cd_resp[s_cd_rpos++] : 0;
    case 2: return s_cd_fifo_pos < s_cd_fifo_len ? s_cd_fifo[s_cd_fifo_pos++] : 0;
    case 3: return (s_cd_index & 1) ? (0xE0u | s_cd_if) : (0xE0u | s_cd_ie);
    default: return 0;
    }
}

static void cd_write(uint32_t reg, uint8_t v)
{
    if (reg == 0) { s_cd_index = v & 3; return; }
    switch (s_cd_index * 4 + reg) {
    case 1: cd_command(v); break;                                   /* 0:1 command */
    case 2: if (s_cd_nparam < 16) s_cd_param[s_cd_nparam++] = v; break; /* 0:2 param */
    case 3: if (v & 0x80) cd_load_fifo(); else s_cd_fifo_len = s_cd_fifo_pos = 0; break; /* 0:3 request */
    case 6: s_cd_ie = v & 0x1F; break;                              /* 1:2 int enable */
    case 7: cd_ack(v & 0x1F); break;                                /* 1:3 int flag ack */
    default: break;
    }
}

/* ---------------------------------------------------------------- dispatch */
uint32_t dc_io_read(uint32_t p, int size)
{
    (void)size;
    if (p == 0x1F801070u) return s_istat;
    if (p == 0x1F801074u) return s_imask;
    if (p >= 0x1F801080u && p < 0x1F8010F0u) {
        int ch = (int)((p - 0x1F801080u) >> 4);
        switch (p & 0xC) { case 0: return s_dma_madr[ch]; case 4: return s_dma_bcr[ch]; case 8: return s_dma_chcr[ch]; }
        return 0;
    }
    if (p == 0x1F8010F0u) return s_dpcr;
    if (p == 0x1F8010F4u) return s_dicr;
    if (p >= 0x1F801100u && p < 0x1F801130u) {
        int t = (int)((p - 0x1F801100u) >> 4);
        switch (p & 0xC) { case 0: return tmr_count(t); case 4: { uint32_t m = s_tmr_mode[t]; s_tmr_mode[t] &= ~0x1800u; return m; } case 8: return s_tmr_target[t]; }
        return 0;
    }
    if (p >= 0x1F801800u && p < 0x1F801804u) return cd_read(p & 3);
    if (p == 0x1F801810u) return dc_gpu_read();
    if (p == 0x1F801814u) return dc_gpu_stat();
    if (p == 0x1F801820u || p == 0x1F801824u) return dc_mdec_read(p);
    if (p == 0x1F801044u) return 0x00000005u;                        /* JOY_STAT: TX ready */
    if (p == 0x1FFE0130u) return s_cache_ctrl;
    if (p >= 0x1F801C00u && p < 0x1F801E60u) {
        uint32_t v = dc_spu_read(p & ~1u);
        if (size == 4) v |= (uint32_t)dc_spu_read((p & ~1u) + 2u) << 16;
        return v;
    }
    if (p >= 0x1F801000u && p < 0x1F803000u) {
        uint32_t o = p - 0x1F801000u, v = 0;
        memcpy(&v, s_io_other + (o & 0x1FFCu), 4);
        return v;
    }
    if (p >= 0x1FC00000u && p < 0x1FC80000u) return 0;              /* BIOS ROM */
    log_io("read unmapped", p, 0);
    return 0;
}

void dc_io_write(uint32_t p, uint32_t v, int size)
{
    if (p == 0x1F801070u) { s_istat &= v; return; }
    if (p == 0x1F801074u) { s_imask = v & 0x7FFu; if (s_istat & s_imask) g_irq_pending = 1; return; }
    if (p >= 0x1F801080u && p < 0x1F8010F0u) {
        int ch = (int)((p - 0x1F801080u) >> 4);
        switch (p & 0xC) {
        case 0: s_dma_madr[ch] = v & 0xFFFFFFu; break;
        case 4: s_dma_bcr[ch] = v; break;
        case 8: s_dma_chcr[ch] = v;
            if (ch <= 1) dc_ev(ch ? 'c' : 'b', v, s_dma_madr[ch]);
            if (ch == 1 && !(v & (1u << 24))) s_dma1_wait = 0;
            if (v & (1u << 24)) dma_start(ch); break;
        }
        return;
    }
    if (p == 0x1F8010F0u) { s_dpcr = v; return; }
    if (p == 0x1F8010F4u) {
        s_dicr = (s_dicr & 0x7F000000u & ~(v & 0x7F000000u)) | (v & 0x00FF803Fu);
        return;
    }
    if (p >= 0x1F801100u && p < 0x1F801130u) {
        int t = (int)((p - 0x1F801100u) >> 4);
        switch (p & 0xC) {
        case 0: s_tmr_org[t] = tmr_now(t) - (v & 0xFFFFu); tmr_arm(t); break;
        case 4: { s_tmr_mode[t] = (v & 0x3FFu) | 0x400u; s_tmr_org[t] = tmr_now(t); tmr_arm(t);
                  char b[64]; snprintf(b, sizeof b, "[tmr] %d mode=%04X tgt=%04X\n", t, (unsigned)v, (unsigned)s_tmr_target[t]); dc_host_log(b); break; }
        case 8: s_tmr_target[t] = v & 0xFFFFu; tmr_arm(t); break;
        }
        return;
    }
    if (p >= 0x1F801800u && p < 0x1F801804u) { cd_write(p & 3, (uint8_t)v); return; }
    if (p == 0x1F801810u) { DC_PHASE(DC_PH_GPU); dc_gpu_gp0(v); DC_PHASE_END(); return; }
    if (p == 0x1F801820u || p == 0x1F801824u) {
        if (p & 4) dc_ev('K', v, s_dma1_wait);
        dc_mdec_write(p, v);
        if (s_dma1_wait) { s_dma1_wait = 0; dma_start(1); }
        return;
    }
    if (p == 0x1F801814u) { DC_PHASE(DC_PH_GPU); dc_gpu_gp1(v); DC_PHASE_END(); return; }
    if (p == 0x1FFE0130u) { s_cache_ctrl = v; return; }
    if (p >= 0x1F801C00u && p < 0x1F801E00u) {
        if (size == 1) return;
        dc_spu_write(p & ~1u, (uint16_t)v);
        if (size == 4) dc_spu_write((p & ~1u) + 2u, (uint16_t)(v >> 16));
        return;
    }
    if (p >= 0x1F801000u && p < 0x1F803000u) {
        uint32_t o = (p - 0x1F801000u) & 0x1FFFu;
        if (size == 4) memcpy(s_io_other + (o & ~3u), &v, 4);
        else if (size == 2) { uint16_t h = (uint16_t)v; memcpy(s_io_other + (o & ~1u), &h, 2); }
        else s_io_other[o] = (uint8_t)v;
        return;
    }
    if (p >= 0x1FC00000u && p < 0x1FC80000u) return;
    log_io("write unmapped", p, v);
}

/* ---------------------------------------------------------------- accessors */
static inline int is_ram(uint32_t p) { return p < 0x00800000u; }
static inline int is_scratch(uint32_t p) { return (p - 0x1F800000u) < 0x400u; }

uint32_t dc_read32(uint32_t a)
{
    uint32_t p = dc_phys(a), v;
    if (is_ram(p)) { memcpy(&v, g_dc_ram + (p & 0x1FFFFCu), 4); return v; }
    if (is_scratch(p)) { memcpy(&v, g_dc_scratch + (p & 0x3FCu), 4); return v; }
    return dc_io_read(p, 4);
}
uint16_t dc_read16(uint32_t a)
{
    uint32_t p = dc_phys(a); uint16_t v;
    if (is_ram(p)) { memcpy(&v, g_dc_ram + (p & 0x1FFFFEu), 2); return v; }
    if (is_scratch(p)) { memcpy(&v, g_dc_scratch + (p & 0x3FEu), 2); return v; }
    return (uint16_t)dc_io_read(p, 2);
}
uint8_t dc_read8(uint32_t a)
{
    uint32_t p = dc_phys(a);
    if (is_ram(p)) return g_dc_ram[p & 0x1FFFFFu];
    if (is_scratch(p)) return g_dc_scratch[p & 0x3FFu];
    return (uint8_t)dc_io_read(p, 1);
}
void dc_write32(uint32_t a, uint32_t v)
{
    uint32_t p = dc_phys(a);
    if (is_ram(p)) { memcpy(g_dc_ram + (p & 0x1FFFFCu), &v, 4); return; }
    if (is_scratch(p)) { memcpy(g_dc_scratch + (p & 0x3FCu), &v, 4); return; }
    dc_io_write(p, v, 4);
}
void dc_write16(uint32_t a, uint16_t v)
{
    uint32_t p = dc_phys(a);
    if (is_ram(p)) { memcpy(g_dc_ram + (p & 0x1FFFFEu), &v, 2); return; }
    if (is_scratch(p)) { memcpy(g_dc_scratch + (p & 0x3FEu), &v, 2); return; }
    dc_io_write(p, v, 2);
}
void dc_write8(uint32_t a, uint8_t v)
{
    uint32_t p = dc_phys(a);
    if (is_ram(p)) { g_dc_ram[p & 0x1FFFFFu] = v; return; }
    if (is_scratch(p)) { g_dc_scratch[p & 0x3FFu] = v; return; }
    dc_io_write(p, v, 1);
}

/* Slow paths called by DC_FAST/DC_STORE (dc_target.h). */
uint32_t dc_lw_slow(uint32_t a) { return dc_read32(a); }
uint32_t dc_lh_slow(uint32_t a) { return dc_read16(a); }
uint32_t dc_lb_slow(uint32_t a) { return dc_read8(a); }
void dc_sw_slow(uint32_t a, uint32_t v) { dc_write32(a, v); }
void dc_sh_slow(uint32_t a, uint32_t v) { dc_write16(a, (uint16_t)v); }
void dc_sb_slow(uint32_t a, uint32_t v) { dc_write8(a, (uint8_t)v); }

/* Generic word read used by a few generated helpers (unaligned access paths). */
uint32_t psx_read_word(uint32_t a) { return dc_read32(a); }
uint32_t psx_cyc_lwc2_read(CPUState* cpu, uint32_t a) { (void)cpu; return dc_read32(a); }
