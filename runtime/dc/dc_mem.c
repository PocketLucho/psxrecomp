/* dc_mem.c - guest memory, slow-path accessors and PS1 I/O for the DC runtime. */
#include "dc_runtime.h"
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
static uint32_t s_tmr_mode[3], s_tmr_target[3], s_tmr_base[3];
static uint32_t s_gpustat_flip;
static uint8_t  s_io_other[0x2000];    /* plain storage for registers we do not model */
static uint32_t s_cache_ctrl;

/* SPU: 512 KB sound RAM and the registers libspu polls. Voices are not mixed
 * yet; the AICA port will replace this. */
static uint8_t  s_spu_ram[512 * 1024];
static uint32_t s_spu_addr;              /* transfer address in bytes */
static uint16_t s_spu_cnt;
static uint16_t spu_stat(void) { return (uint16_t)(s_spu_cnt & 0x3Fu); }

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
static uint32_t tmr_now(int t)
{
    uint64_t us = dc_host_ticks_us();
    switch (t) {
    case 0: return (uint32_t)(us * 53u / 10u);       /* ~dot clock */
    case 1: return (uint32_t)(us * 157u / 10000u);   /* hblank, ~15.7 kHz */
    default: return (uint32_t)(us * 423u / 100u);    /* sysclk/8 */
    }
}
static uint32_t tmr_count(int t) { return (tmr_now(t) - s_tmr_base[t]) & 0xFFFFu; }

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

static void dma_start(int ch)
{
    uint32_t madr = s_dma_madr[ch] & 0x1FFFFCu, bcr = s_dma_bcr[ch], chcr = s_dma_chcr[ch];
    if (ch == 6) {                       /* OTC: build a reverse linked list */
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
            g_dc_stats.gp0_words += hdr >> 24;
            if (hdr & 0x800000u) break;
            a = hdr & 0x1FFFFCu;
        }
        g_dc_stats.dma_gpu++;
    } else if (ch == 3) {                /* CD-ROM -> RAM */
        uint32_t words = (bcr & 0xFFFFu) * ((bcr >> 16) ? (bcr >> 16) : 1u);
        if (!words) words = 0x10000u;
        uint32_t a = madr;
        for (uint32_t i = 0; i < words; i++, a += 4) {
            uint8_t w[4]; dc_cd_fifo_read(w, 4);
            memcpy(g_dc_ram + (a & 0x1FFFFCu), w, 4);
        }
    } else if (ch == 4) {                /* SPU: RAM <-> sound RAM */
        uint32_t words = (bcr & 0xFFFFu) * ((bcr >> 16) ? (bcr >> 16) : 1u);
        uint32_t a = madr;
        for (uint32_t i = 0; i < words; i++, a += 4) {
            uint32_t o = s_spu_addr & 0x7FFFCu;
            if (chcr & 1u) memcpy(s_spu_ram + o, g_dc_ram + (a & 0x1FFFFCu), 4);
            else memcpy(g_dc_ram + (a & 0x1FFFFCu), s_spu_ram + o, 4);
            s_spu_addr += 4;
        }
    } else if (ch == 2) {
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
static uint32_t s_cd_lba, s_cd_mode;
static int s_cd_reading, s_cd_sector_ready;
static uint8_t s_cd_sector[2352];
static uint8_t s_cd_fifo[2352];
static uint32_t s_cd_fifo_len, s_cd_fifo_pos;

static uint32_t bcd(uint8_t v) { return (uint32_t)((v >> 4) * 10 + (v & 15)); }

static void cd_next_sector(void)
{
    if (dc_host_cd_read_sector(s_cd_lba, s_cd_sector) != 0) {
        static const uint8_t err[2] = {0x03, 0x80};
        s_cd_reading = 0;
        cd_respond(5, 2, err);
        dc_host_log("[cd] read failed: no disc image or sector out of range\n");
        return;
    }
    s_cd_lba++;
    s_cd_sector_ready = 1;
    s_cd_stat = 0x22;
    cd_respond(1, 1, &s_cd_stat);
}

static void cd_ack(uint8_t bits)
{
    s_cd_if &= ~bits;
    if (s_cd_if == 0 && s_cd_npend) {
        CdResp x = s_cd_pend[0];
        memmove(s_cd_pend, s_cd_pend + 1, sizeof(CdResp) * (size_t)--s_cd_npend);
        cd_deliver(&x);
    }
}

/* Called from the interrupt path: deliver the next sector of a running read
 * once the drive would have it (75 sectors/s, 150 at double speed). */
static uint64_t s_cd_next_us;
void dc_io_poll(void)
{
    if (dc_host_take_vblanks()) dc_irq_raise(1u);
    if (!s_cd_reading || s_cd_if != 0 || s_cd_npend) return;
    uint64_t now = dc_host_ticks_us();
    if (now < s_cd_next_us) return;
    s_cd_next_us = now + ((s_cd_mode & 0x80) ? 6667u : 13333u);
    cd_next_sector();
}

static void cd_load_fifo(void)
{
    if (s_cd_trace > 0) { char b[48]; snprintf(b, sizeof b, "[cd] fifo load ready=%d\n", s_cd_sector_ready); dc_host_log(b); }
    if (!s_cd_sector_ready) return;
    if (s_cd_mode & 0x20) { memcpy(s_cd_fifo, s_cd_sector + 12, 2340); s_cd_fifo_len = 2340; }
    else { memcpy(s_cd_fifo, s_cd_sector + 24, 2048); s_cd_fifo_len = 2048; }
    s_cd_fifo_pos = 0;
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
    if (s_cd_reading && cmd != 0x01) { s_cd_reading = 0; s_cd_npend = 0; }
    uint8_t st = s_cd_stat = s_cd_reading ? 0x22 : 0x02;
    switch (cmd) {
    case 0x02:                           /* Setloc mm:ss:ff (BCD) */
        if (s_cd_nparam >= 3)
            s_cd_lba = (bcd(s_cd_param[0]) * 60 + bcd(s_cd_param[1])) * 75 + bcd(s_cd_param[2]) - 150;
        cd_respond(3, 1, &st); break;
    case 0x0E: s_cd_mode = s_cd_param[0]; cd_respond(3, 1, &st); break;   /* Setmode */
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
        switch (p & 0xC) { case 0: return tmr_count(t); case 4: return s_tmr_mode[t]; case 8: return s_tmr_target[t]; }
        return 0;
    }
    if (p >= 0x1F801800u && p < 0x1F801804u) return cd_read(p & 3);
    if (p == 0x1F801810u) return 0;                                  /* GPUREAD */
    if (p == 0x1F801814u) return 0x1C000000u | ((s_gpustat_flip ^= 1u) << 31); /* GPUSTAT */
    if (p == 0x1F801824u) return 0x80040000u;                        /* MDEC status: idle */
    if (p == 0x1F801044u) return 0x00000005u;                        /* JOY_STAT: TX ready */
    if (p == 0x1FFE0130u) return s_cache_ctrl;
    if (p == 0x1F801DAAu) return s_spu_cnt;
    if (p == 0x1F801DAEu) return spu_stat();
    if (p == 0x1F801DA6u) return (s_spu_addr >> 3) & 0xFFFFu;
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
        case 8: s_dma_chcr[ch] = v; if (v & (1u << 24)) dma_start(ch); break;
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
        case 0: s_tmr_base[t] = tmr_now(t) - (v & 0xFFFFu); break;
        case 4: s_tmr_mode[t] = v; s_tmr_base[t] = tmr_now(t); break;
        case 8: s_tmr_target[t] = v & 0xFFFFu; break;
        }
        return;
    }
    if (p >= 0x1F801800u && p < 0x1F801804u) { cd_write(p & 3, (uint8_t)v); return; }
    if (p == 0x1F801810u) { g_dc_stats.gp0_words++; return; }
    if (p == 0x1F801814u) { g_dc_stats.gp1_writes++; return; }
    if (p == 0x1FFE0130u) { s_cache_ctrl = v; return; }
    if (p == 0x1F801DAAu) { s_spu_cnt = (uint16_t)v; return; }
    if (p == 0x1F801DA6u) { s_spu_addr = (v & 0xFFFFu) << 3; return; }
    if (p == 0x1F801DA8u) {              /* manual FIFO write */
        uint16_t h = (uint16_t)v; memcpy(s_spu_ram + (s_spu_addr & 0x7FFFEu), &h, 2); s_spu_addr += 2;
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
