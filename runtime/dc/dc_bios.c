/* dc_bios.c - high-level PS1 BIOS for the DC runtime.
 *
 * Implements the A0/B0/C0 services and syscalls a PSY-Q game uses at boot:
 * string/memory helpers, heap, events, interrupt-handler chains, the
 * interrupt hook (HookEntryInt / ReturnFromException) and critical sections.
 * Unknown services are logged once and return 0.
 */
#include "dc_runtime.h"
#include <stdio.h>
#include <string.h>

void dc_request_stop(void);

#define V0 cpu->gpr[2]
#define A0 cpu->gpr[4]
#define A1 cpu->gpr[5]
#define A2 cpu->gpr[6]
#define A3 cpu->gpr[7]

/* ---- events ---- */
#define DC_MAX_EVENTS 32
typedef struct { uint32_t cls, spec, mode, func, enabled, ready, used; } DcEvent;
static DcEvent s_ev[DC_MAX_EVENTS];

/* ---- interrupt handler chains (SysEnqIntRP) and hook ---- */
static uint32_t s_pad_buf[2], s_pad_siz[2];
static int s_pad_on;
static uint32_t s_intrp[4];        /* head of each priority chain (guest address) */
static uint32_t s_hook;            /* HookEntryInt jmp_buf, 0 = none */
static int s_hook_active;

/* ---- heap (InitHeap / malloc / free): simple bump allocator ---- */
static uint32_t s_heap_cur, s_heap_end;

static const uint32_t k_tab_addr[3] = {0x00000200u, 0x00000874u, 0x00000674u};

static uint32_t rd(uint32_t a) { return dc_read32(a); }
static void wr(uint32_t a, uint32_t v) { dc_write32(a, v); }

void dc_bios_init(CPUState* cpu)
{
    memset(s_ev, 0, sizeof s_ev);
    memset(s_intrp, 0, sizeof s_intrp);
    s_hook = 0; s_hook_active = 0;
    cpu->cop0[12] = 0x00000401u;      /* interrupts enabled, IM2 (hardware) on */
    cpu->cop0[15] = 0x00000002u;      /* PRId */
    /* Kernel RAM tables at their SCPH-1001 addresses. Each entry points into
     * the stub window, so a call through a table reaches the HLE service and a
     * library that patches BIOS code relative to an entry writes harmless RAM. */
    for (uint32_t t = 0; t < 3; t++)
        for (uint32_t fn = 0; fn < 256; fn++)
            wr(k_tab_addr[t] + fn * 4u, DC_KTAB_BASE + t * 0x400u + fn * 4u);
    wr(0x00000100u, 0x00000000u);     /* TCB/PCB pointers: none */
}

int dc_bios_irq_enabled(void)
{
    return (g_dc_cpu.cop0[12] & 0x401u) == 0x401u;
}

static void log_once(char tbl, uint32_t fn)
{
    static uint8_t seen[3][256];
    int t = tbl == 'A' ? 0 : tbl == 'B' ? 1 : 2;
    if (fn < 256 && seen[t][fn]) return;
    if (fn < 256) seen[t][fn] = 1;
    char b[64];
    snprintf(b, sizeof b, "[bios] unimplemented %c0:%02X\n", tbl, (unsigned)fn);
    dc_host_log(b);
}

static void trace(char tbl, uint32_t fn, CPUState* cpu)
{
    static int n;
    if (tbl == 'B' && (fn == 0x35 || fn == 0x3D)) return;   /* tty output */
    if (tbl == 'B' && fn == 0x17) return;               /* ReturnFromException */
    int card = (tbl == 'B' && ((fn >= 0x32 && fn <= 0x36) || (fn >= 0x40 && fn <= 0x55) || fn == 0x5C || fn == 0x5D)) || (tbl == 'A' && fn >= 0x9C && fn <= 0xAF);
    if (n++ < 400 || card) {
        char b[96];
        snprintf(b, sizeof b, "[bios] %c0:%02X a0=%08X a1=%08X ra=%08X\n", tbl, (unsigned)fn,
                 (unsigned)A0, (unsigned)A1, (unsigned)cpu->gpr[31]);
        dc_host_log(b);
    }
}

static void guest_strcpy(uint32_t d, uint32_t s) { uint8_t c; do { c = dc_read8(s++); dc_write8(d++, c); } while (c); }

static uint32_t guest_strlen(uint32_t s) { uint32_t n = 0; while (dc_read8(s + n)) n++; return n; }

/* Guest TTY output goes to the serial port, which is slow: a game that
 * prints every frame (libgpu's "VSync: timeout") would spend most of its
 * time there, so only the first lines are kept. */
static unsigned s_tty_budget = 4000;
static void guest_putc(char ch) { if (s_tty_budget) { s_tty_budget--; char c[2] = {ch, 0}; dc_host_log(c); } }
static void guest_puts(uint32_t s)
{
    char b[256]; uint32_t i = 0;
    if (s_tty_budget < 64) return;
    s_tty_budget -= 64;
    while (i < sizeof b - 1 && (b[i] = (char)dc_read8(s + i))) i++;
    b[i] = 0;
    dc_host_log("[tty] "); dc_host_log(b); dc_host_log("\n");
}

/* setjmp/longjmp buffer: ra, sp, fp, s0..s7, gp */
static void do_setjmp(CPUState* cpu, uint32_t buf)
{
    wr(buf + 0, cpu->gpr[31]); wr(buf + 4, cpu->gpr[29]); wr(buf + 8, cpu->gpr[30]);
    for (int i = 0; i < 8; i++) wr(buf + 12 + 4u * (uint32_t)i, cpu->gpr[16 + i]);
    wr(buf + 44, cpu->gpr[28]);
}
static void do_longjmp(CPUState* cpu, uint32_t buf)
{
    cpu->gpr[31] = rd(buf + 0); cpu->gpr[29] = rd(buf + 4); cpu->gpr[30] = rd(buf + 8);
    for (int i = 0; i < 8; i++) cpu->gpr[16 + i] = rd(buf + 12 + 4u * (uint32_t)i);
    cpu->gpr[28] = rd(buf + 44);
}

/* ---- events ---- */
static uint32_t ev_open(uint32_t cls, uint32_t spec, uint32_t mode, uint32_t func)
{
    for (int i = 0; i < DC_MAX_EVENTS; i++) if (!s_ev[i].used) {
        s_ev[i] = (DcEvent){cls, spec, mode, func, 0, 0, 1};
        return 0xF1000000u | (uint32_t)i;
    }
    return 0xFFFFFFFFu;
}
static DcEvent* ev_get(uint32_t h) { uint32_t i = h & 0xFFFFu; return ((h >> 24) == 0xF1u && i < DC_MAX_EVENTS && s_ev[i].used) ? &s_ev[i] : 0; }

static void ev_deliver(CPUState* cpu, uint32_t cls, uint32_t spec)
{
    for (int i = 0; i < DC_MAX_EVENTS; i++) {
        DcEvent* e = &s_ev[i];
        if (!e->used || !e->enabled || e->cls != cls || e->spec != spec) continue;
        if (e->mode == 0x2000u) e->ready = 1;
        else if (e->mode == 0x1000u && e->func) dc_call_guest(cpu, e->func, 0, 0, 0, 0);
    }
}

/* ---- interrupt delivery: handler chains, then the PSY-Q hook ---- */
/* BIOS pad buffers (InitPad/StartPad): filled at every VBlank, as the real
 * BIOS does, with a digital pad on each port. Byte 0: 0x00 = connected,
 * byte 1: 0x41 = digital pad, bytes 2-3: buttons, active low. */
static void pad_update(void)
{
    if (!s_pad_on) return;
    for (int port = 0; port < 2; port++) {
        uint32_t b = s_pad_buf[port];
        if (!b || s_pad_siz[port] < 4) continue;
        uint16_t keys = (uint16_t)~dc_host_pad_buttons(port);
        if (port == 1) { dc_write8(b, 0xFF); continue; }   /* nothing in port 2 */
        dc_write8(b + 0, 0x00); dc_write8(b + 1, 0x41);
        dc_write8(b + 2, (uint8_t)keys); dc_write8(b + 3, (uint8_t)(keys >> 8));
    }
}

uint32_t g_dc_drv_us;
void dc_bios_interrupt(CPUState* cpu)
{
    uint32_t pending = dc_io_istat() & dc_io_imask();
    /* Handlers run on the kernel's exception stack, as on the real BIOS: the
     * interrupted code may sit on the 1 KB scratchpad stack, and a sound
     * driver's timer callback overflowed it into the game's data there. The
     * caller restores the interrupted registers, sp included. */
    cpu->gpr[29] = 0x8000F000u;
    if (pending & 1u) { g_dc_stats.vblanks++; pad_update(); ev_deliver(cpu, 0xF2000003u, 2u); }
    /* Root counters 0-2: the kernel's own handlers deliver RCntCNTn/EvSpINT
     * and acknowledge the IRQ (ChangeClearRCnt default). Sound drivers run
     * their sequencer off this event. */
    for (int t = 0; t < 3; t++) {
        uint32_t bit = 1u << (4 + t);
        if (!(pending & bit)) continue;
        dc_io_ack(bit);
        pending &= ~bit;
        { extern uint32_t g_dc_drv_us; uint64_t t0 = dc_host_ticks_us();
          ev_deliver(cpu, 0xF2000000u | (uint32_t)t, 2u);
          g_dc_drv_us += (uint32_t)(dc_host_ticks_us() - t0); }
    }
    for (int pr = 0; pr < 4; pr++) {
        for (uint32_t q = s_intrp[pr], guard = 0; q && guard < 16; q = rd(q), guard++) {
            uint32_t func2 = rd(q + 4), func1 = rd(q + 8);
            uint32_t r = func1 ? dc_call_guest(cpu, func1, 0, 0, 0, 0) : 1;
            if (r && func2) dc_call_guest(cpu, func2, r, 0, 0, 0);
        }
    }
    if (s_hook) {
        /* longjmp into the game's interrupt manager; it acknowledges I_STAT,
         * runs its callbacks and calls ReturnFromException, which ends the run. */
        do_longjmp(cpu, s_hook);
        cpu->gpr[2] = 1;
        s_hook_active = 1;
        dc_run(cpu, cpu->gpr[31], 0x80000F00u);
        s_hook_active = 0;
    } else {
        dc_io_ack(pending);            /* nobody listens: drop it */
    }
}

/* ---- A0 ---- */
int dc_card_a0(CPUState* cpu, uint32_t fn);
int dc_card_b0(CPUState* cpu, uint32_t fn);
void dc_bios_event(CPUState* cpu, uint32_t cls, uint32_t spec) { ev_deliver(cpu, cls, spec); }

static void bios_a0(CPUState* cpu, uint32_t fn)
{
    if (dc_card_a0(cpu, fn)) return;
    switch (fn) {
    case 0x13: do_setjmp(cpu, A0); V0 = 0; break;
    case 0x14: do_longjmp(cpu, A0); V0 = A1 ? A1 : 1; break;
    case 0x17: { uint32_t a = A0, b = A1; int r = 0; for (;;) { uint8_t x = dc_read8(a++), y = dc_read8(b++); if (x != y) { r = x - y; break; } if (!x) break; } V0 = (uint32_t)r; break; }
    case 0x19: guest_strcpy(A0, A1); V0 = A0; break;
    case 0x1B: V0 = guest_strlen(A0); break;
    case 0x25: { uint32_t c = A0 & 0xFF; V0 = (c >= 'a' && c <= 'z') ? c - 32 : c; break; }
    case 0x2A: for (uint32_t i = 0; i < A2; i++) dc_write8(A0 + i, dc_read8(A1 + i)); V0 = A0; break;
    case 0x2B: for (uint32_t i = 0; i < A2; i++) dc_write8(A0 + i, (uint8_t)A1); V0 = A0; break;
    case 0x2F: { static uint32_t seed = 0x24040001u; seed = seed * 1103515245u + 12345u; V0 = (seed >> 16) & 0x7FFFu; break; }
    case 0x33: { uint32_t n = (A0 + 7u) & ~7u; if (s_heap_cur + n > s_heap_end) V0 = 0; else { V0 = s_heap_cur; s_heap_cur += n; } break; }
    case 0x34: break;                                   /* free: bump allocator */
    case 0x39: s_heap_cur = A0; s_heap_end = A0 + A1; break;
    case 0x3C: guest_putc((char)A0); break;
    case 0x3E: guest_puts(A0); break;
    case 0x3F: guest_puts(A0); break;                   /* printf: format string only */
    case 0x44: break;                                   /* FlushCache */
    case 0x49: dc_write32(0x1F801814u, A0); break;      /* GPU_cw */
    case 0x70: case 0x72: case 0x96: case 0x97: case 0x99: case 0x9F: case 0xA3: break;
    default: log_once('A', fn); V0 = 0; break;
    }
}

/* ---- B0 ---- */
static void bios_b0(CPUState* cpu, uint32_t fn)
{
    if (dc_card_b0(cpu, fn)) return;
    switch (fn) {
    case 0x07: ev_deliver(cpu, A0, A1); break;
    case 0x08: V0 = ev_open(A0, A1, A2, A3); break;
    case 0x09: { DcEvent* e = ev_get(A0); if (e) e->used = 0; V0 = 1; break; }
    case 0x0A: {                                        /* WaitEvent */
        DcEvent* e = ev_get(A0);
        if (!e) { V0 = 0; break; }
        uint64_t t0 = dc_host_ticks_us();
        while (!e->ready) {
            if (g_irq_pending) dc_irq_slow(cpu, cpu->gpr[31]);
            if (dc_host_ticks_us() - t0 > 5000000u) dc_fatal("WaitEvent(%08X) timed out", (unsigned)A0);
        }
        e->ready = 0; V0 = 1; break;
    }
    case 0x0B: { DcEvent* e = ev_get(A0); V0 = (e && e->ready) ? 1 : 0; if (e) e->ready = 0; break; }
    case 0x0C: { DcEvent* e = ev_get(A0); if (e) e->enabled = 1; V0 = 1; break; }
    case 0x0D: { DcEvent* e = ev_get(A0); if (e) e->enabled = 0; V0 = 1; break; }
    case 0x12:                                          /* InitPad(buf1, siz1, buf2, siz2) */
        s_pad_buf[0] = A0; s_pad_siz[0] = A1; s_pad_buf[1] = A2; s_pad_siz[1] = A3; V0 = 1; break;
    case 0x13: s_pad_on = 1; V0 = 1; break;             /* StartPad */
    case 0x14: s_pad_on = 0; V0 = 1; break;             /* StopPad */
    case 0x17:                                          /* ReturnFromException */
        if (s_hook_active) dc_request_stop();
        else dc_fatal("ReturnFromException outside an interrupt");
        break;
    case 0x18: s_hook = 0; break;                       /* ResetEntryInt */
    case 0x19: s_hook = A0; break;                      /* HookEntryInt */
    case 0x20: { for (int i = 0; i < DC_MAX_EVENTS; i++) if (s_ev[i].used && s_ev[i].cls == A0 && s_ev[i].spec == A1) s_ev[i].ready = 0; break; }
    case 0x35:                                          /* write(fd, buf, len): tty only */
        if (A0 == 1) { char b[128]; uint32_t n = A2 < 127 ? A2 : 127; for (uint32_t i = 0; i < n; i++) b[i] = (char)dc_read8(A1 + i); b[n] = 0; dc_host_log(b); }
        V0 = A2; break;
    case 0x3D: guest_putc((char)A0); break;
    case 0x3F: guest_puts(A0); break;
    case 0x56: V0 = k_tab_addr[2]; break;               /* GetC0Table */
    case 0x57: V0 = k_tab_addr[1]; break;               /* GetB0Table */
    case 0x5B: break;                                   /* ChangeClearPad */
    default: log_once('B', fn); V0 = 0; break;
    }
}

/* ---- C0 ---- */
static void bios_c0(CPUState* cpu, uint32_t fn)
{
    switch (fn) {
    case 0x02: {                                        /* SysEnqIntRP(priority, struc) */
        uint32_t pr = A0 & 3u;
        wr(A1, s_intrp[pr]); s_intrp[pr] = A1; V0 = 0; break;
    }
    case 0x03: {                                        /* SysDeqIntRP(priority, struc) */
        uint32_t pr = A0 & 3u, prev = 0, q = s_intrp[pr];
        while (q && q != A1) { prev = q; q = rd(q); }
        if (q) { if (prev) wr(prev, rd(q)); else s_intrp[pr] = rd(q); }
        V0 = 0; break;
    }
    case 0x0A: V0 = 0; break;                           /* ChangeClearRCnt */
    default: log_once('C', fn); V0 = 0; break;
    }
}

int dc_bios_call(CPUState* cpu, uint32_t vector)
{
    uint32_t fn = cpu->gpr[9] & 0xFFu;
    g_dc_stats.bios_calls++;
    char t = vector == 0xA0u ? 'A' : vector == 0xB0u ? 'B' : 'C';
    trace(t, fn, cpu);
    if (vector == 0xA0u) bios_a0(cpu, fn);
    else if (vector == 0xB0u) bios_b0(cpu, fn);
    else bios_c0(cpu, fn);
    return 1;
}

static void bios_a0(CPUState* cpu, uint32_t fn);
static void bios_b0(CPUState* cpu, uint32_t fn);
static void bios_c0(CPUState* cpu, uint32_t fn);

void dc_bios_kernel_call(CPUState* cpu, uint32_t p)
{
    if (p >= DC_KTAB_BASE && p < DC_KTAB_BASE + 0xC00u) {
        uint32_t t = (p - DC_KTAB_BASE) >> 10, fn = ((p - DC_KTAB_BASE) & 0x3FFu) >> 2;
        g_dc_stats.bios_calls++;
        trace("ABC"[t], fn, cpu);
        if (t == 0) bios_a0(cpu, fn); else if (t == 1) bios_b0(cpu, fn); else bios_c0(cpu, fn);
        return;
    }
    /* BIOS-internal code reached through a patched table entry (libpad, libcard
     * and libapi do this): treat it as a routine with nothing to do. */
    static uint32_t seen[32]; static int nseen;
    int known = 0;
    for (int i = 0; i < nseen; i++) if (seen[i] == p) known = 1;
    if (!known && nseen < 32) {
        char b[80];
        seen[nseen++] = p;
        snprintf(b, sizeof b, "[bios] kernel code %08X called (ra=%08X): no-op\n", (unsigned)p, (unsigned)cpu->gpr[31]);
        dc_host_log(b);
    }
    V0 = 0;
}

int dc_bios_syscall(CPUState* cpu, uint32_t code)
{
    (void)code;
    switch (A0) {
    case 1: V0 = (cpu->cop0[12] & 1u) ? 1 : 0; cpu->cop0[12] &= ~1u; break;   /* EnterCriticalSection */
    case 2: cpu->cop0[12] |= 0x401u; if (dc_io_istat() & dc_io_imask()) g_irq_pending = 1; break; /* ExitCriticalSection */
    default: break;
    }
    return 0;
}
