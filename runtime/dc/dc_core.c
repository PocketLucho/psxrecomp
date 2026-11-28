/* dc_core.c - CPS trampoline, interrupt delivery and runtime glue for --target dc. */
#include "dc_runtime.h"
#include "dc_overlay.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

CPUState g_dc_cpu;
int g_psx_cps_mode = 1;

#define DC_SENTINEL_RA 0x80000F00u   /* kernel-area return address no guest code uses */

extern int psx_dispatch_game_compiled(CPUState* cpu, uint32_t addr);

/* Tiny sampling profile of the last dispatched address (host timer). */
static volatile uint32_t s_last_poll_pc;
volatile uint8_t g_dc_phase;
uint32_t dc_last_poll_pc(void) { return s_last_poll_pc; }
static uint32_t s_prof_addr[64], s_prof_hits[64];
void dc_profile_sample(void)
{
    uint32_t a = s_last_poll_pc ? s_last_poll_pc : g_dc_stats.last_addr;
    for (int i = 0; i < 64; i++) {
        if (s_prof_hits[i] && s_prof_addr[i] == a) { s_prof_hits[i]++; return; }
        if (!s_prof_hits[i]) { s_prof_addr[i] = a; s_prof_hits[i] = 1; return; }
    }
}
static void profile_dump(void)
{
    char b[64];
    for (int k = 0; k < 8; k++) {
        int best = -1;
        for (int i = 0; i < 64; i++) if (s_prof_hits[i] && (best < 0 || s_prof_hits[i] > s_prof_hits[best])) best = i;
        if (best < 0) break;
        snprintf(b, sizeof b, "[prof] %08X %u\n", (unsigned)s_prof_addr[best], (unsigned)s_prof_hits[best]);
        dc_host_log(b);
        s_prof_hits[best] = 0;
    }
}

/* Last dispatched guest addresses, printed by dc_fatal. */
static uint32_t s_ring[16]; static unsigned s_ring_pos;

static int s_in_irq_dbg(void);
void dc_fatal(const char* fmt, ...)
{
    char b[256];
    va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof b, fmt, ap); va_end(ap);
    dc_host_log("[fatal] "); dc_host_log(b); dc_host_log("\n");
    { extern void dc_cd_debug(char*, unsigned); dc_cd_debug(b, sizeof b); dc_host_log(b); }
    snprintf(b, sizeof b, "[irq-state] in_irq=%d enabled=%d\n", s_in_irq_dbg(), dc_bios_irq_enabled());
    dc_host_log(b);
    snprintf(b, sizeof b,
             "[stats] dispatches=%llu bios=%llu syscalls=%llu irqs=%llu vblanks=%llu "
             "gp0=%llu gp1=%llu gpu_dma=%llu cd_cmds=%llu gte=%llu last=%08X\n",
             (unsigned long long)g_dc_stats.dispatches, (unsigned long long)g_dc_stats.bios_calls,
             (unsigned long long)g_dc_stats.syscalls, (unsigned long long)g_dc_stats.irqs,
             (unsigned long long)g_dc_stats.vblanks, (unsigned long long)g_dc_stats.gp0_words,
             (unsigned long long)g_dc_stats.gp1_writes, (unsigned long long)g_dc_stats.dma_gpu,
             (unsigned long long)g_dc_stats.cd_cmds, (unsigned long long)g_dc_stats.gte_ops,
             (unsigned)g_dc_stats.last_addr);
    dc_host_log(b);
    dc_host_log("[trail]");
    for (unsigned i = 0; i < 16; i++) {
        snprintf(b, sizeof b, " %08X", (unsigned)s_ring[(s_ring_pos + i) & 15]);
        dc_host_log(b);
    }
    dc_host_log("\n");
    profile_dump();
    exit(1);
}

/* ---- interrupts -------------------------------------------------------- */
unsigned dc_io_vblanks_owed(void);
int dc_io_cd_kick(void);
static int s_in_irq;
static int s_in_irq_dbg(void) { return s_in_irq; }

static void deliver_irq(CPUState* cpu)
{
    /* Polling only for a streaming CD (2): check the host clock every 32nd
     * back-edge; reading it and running the I/O tick each time cost more
     * than the movie itself. */
    { static unsigned skip; if (g_irq_pending == 2 && (++skip & 31u)) return; }
    g_irq_pending = 0;
    /* Nothing new is raised while the guest's handler runs. Under a slow host
     * the handler can take longer than one CD sector period of host time, and
     * libcd's handler loops while the CD has an interrupt pending, so a sector
     * arriving mid-handler kept it from ever returning. VBlanks are not lost:
     * the host keeps counting them until the next poll. */
    if (s_in_irq) return;
    dc_io_poll();
#ifdef DC_LOG
    { void dc_ev(uint32_t, uint32_t, uint32_t); uint64_t dc_host_ticks_us(void); static uint64_t lx;
      uint32_t pend = dc_io_istat() & dc_io_imask();
      if (pend && !dc_bios_irq_enabled()) { uint64_t t = dc_host_ticks_us(); if (t - lx > 2000u) { lx = t; dc_ev('X', s_last_poll_pc, pend); } }
      else if (pend & 4u) dc_ev('Q', s_last_poll_pc, pend); }
#endif
    if (!dc_bios_irq_enabled() || !(dc_io_istat() & dc_io_imask())) {
        if (dc_io_cd_kick()) g_irq_pending = 2;    /* keep polling a streaming CD */
        return;
    }
    g_dc_stats.irqs++;
    CPUState saved = *cpu;
    s_in_irq = 1;
    { DC_PHASE(DC_PH_IRQ);
      dc_bios_interrupt(cpu);
      DC_PHASE_END(); }
    s_in_irq = 0;
    /* The handler may have changed RAM, I_STAT, events and the critical-section
     * state; the interrupted context's registers come back unchanged. */
    uint32_t sr = cpu->cop0[12];
    *cpu = saved;
    cpu->cop0[12] = sr;
    if ((dc_io_istat() & dc_io_imask()) || dc_io_vblanks_owed()) g_irq_pending = 1;
    else if (dc_io_cd_kick()) g_irq_pending = 2;
}

/* Service interrupts until the movie frame the CD is delivering is complete
 * (at most ~40 ms): for native code that stands in for a guest routine the
 * PS1 would have run slowly enough for the sectors to arrive meanwhile. */
int dc_cd_frame_arriving(void);
void dc_wait_cd_frame(CPUState* cpu)
{
    if (s_in_irq) return;
    uint64_t dc_host_ticks_us(void);
    uint64_t t0 = dc_host_ticks_us();
    while (dc_cd_frame_arriving() && dc_host_ticks_us() - t0 < 40000u) {
        g_irq_pending = 1;
        deliver_irq(cpu);
    }
}

/* Called by generated code at back-edges when g_irq_pending is set. Services
 * the interrupt in place and returns 0, as the --target dc contract requires. */
int dc_irq_slow(CPUState* cpu, uint32_t pc)
{
    if (!s_in_irq) s_last_poll_pc = pc;
    deliver_irq(cpu);
    return 0;
}

int psx_check_interrupts_dispatch_entry(CPUState* cpu, uint32_t resume_pc)
{
    (void)resume_pc;
    if (g_irq_pending) deliver_irq(cpu);
    return 0;
}

/* ---- trampoline -------------------------------------------------------- */
static int s_stop_request;   /* set by ReturnFromException in an interrupt handler */

void dc_request_stop(void) { s_stop_request = 1; }

void dc_run(CPUState* cpu, uint32_t addr, uint32_t stop_ra)
{
    for (;;) {
        if (stop_ra && dc_phys(addr) == dc_phys(stop_ra)) return;
        if (s_stop_request && stop_ra) { s_stop_request = 0; return; }
        if (g_irq_pending) deliver_irq(cpu);
        g_dc_stats.dispatches++;
        g_dc_stats.last_addr = addr;
        s_ring[s_ring_pos++ & 15] = addr;
        uint32_t p = dc_phys(addr);
        if (p == 0xA0u || p == 0xB0u || p == 0xC0u) {
            dc_bios_call(cpu, p);
            addr = cpu->gpr[31];
            continue;
        }
        if (p >= DC_KSTUB_LO && p < DC_KSTUB_HI) {
            /* Kernel RAM: BIOS table entries and the BIOS code that libraries
             * patch or call directly (see dc_bios.c). */
            dc_bios_kernel_call(cpu, p);
            addr = cpu->gpr[31];
            continue;
        }
        cpu->pc = 0;
        if (!psx_dispatch_game_compiled(cpu, addr) && !dc_overlay_dispatch(cpu, addr)) {
            dc_overlay_explain(addr);
            dc_fatal("no compiled code at %08X (ra=%08X sp=%08X)", (unsigned)addr,
                     (unsigned)cpu->gpr[31], (unsigned)cpu->gpr[29]);
        }
        addr = cpu->pc ? cpu->pc : cpu->gpr[31];
    }
}

uint32_t dc_call_guest(CPUState* cpu, uint32_t fn, uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3)
{
    uint32_t ra = cpu->gpr[31];
    cpu->gpr[4] = a0; cpu->gpr[5] = a1; cpu->gpr[6] = a2; cpu->gpr[7] = a3;
    cpu->gpr[31] = DC_SENTINEL_RA;
    dc_run(cpu, fn, DC_SENTINEL_RA);
    cpu->gpr[31] = ra;
    return cpu->gpr[2];
}

/* ---- glue the generated code links against ----------------------------- */
int psx_syscall(CPUState* cpu, uint32_t code) { (void)code; g_dc_stats.syscalls++; return dc_bios_syscall(cpu, code); }

void psx_unknown_dispatch(CPUState* cpu, uint32_t addr, uint32_t phys)
{
    dc_fatal("data executed as code at %08X (phys %08X, ra=%08X)", (unsigned)addr, (unsigned)phys,
             (unsigned)cpu->gpr[31]);
}

void psx_native_bad_entry(CPUState* cpu, uint32_t owner, uint32_t pc)
{
    dc_overlay_explain(pc);
    char b[64];
    snprintf(b, sizeof b, "[bad-entry] host caller %p\n", __builtin_return_address(0));
    dc_host_log(b);
    dc_fatal("bad continuation %08X in function %08X (ra=%08X)", (unsigned)pc, (unsigned)owner,
             (unsigned)cpu->gpr[31]);
}

void psx_restore_state_escape(void) { /* cpu->pc already holds the target */ }
int  psx_mod_function_entry(CPUState* cpu, uint32_t a) { (void)cpu; (void)a; return 0; }
int  psx_vsync_query_hle_try(CPUState* cpu, uint32_t a) { (void)cpu; (void)a; return 0; }
int  psx_game_identity_gate(const void* id) { (void)id; return 1; }
int  psx_game_identity_bind_static(const void* id) { (void)id; return 1; }
int  dirty_ram_text_native_ok_ranges_from(const uint32_t* p, uint32_t n, uint32_t pc) { (void)p; (void)n; (void)pc; return 1; }
int  dirty_ram_text_native_ok_ranges(const uint32_t* p, uint32_t n) { (void)p; (void)n; return 1; }
int  psx_ws_backdrop_x(int x) { return x; }
uint32_t psx_ws_guest_cull_xclip_bound(uint32_t v) { return v; }
void psx_xg_render_auth(CPUState* c, uint32_t hook, uint32_t pc, uint32_t a, uint32_t b)
{ (void)c; (void)hook; (void)pc; (void)a; (void)b; }
_Bool psx_xg_render_auth_resident_ft4_observe(CPUState* c, uint32_t s, uint32_t pc, uint32_t w)
{ (void)c; (void)s; (void)pc; (void)w; return 0; }

/* GTE: dc_gte.c */

__attribute__((weak)) void dc_game_init(CPUState* cpu) { (void)cpu; }

/* ---- boot -------------------------------------------------------------- */
static uint32_t rd32(const uint8_t* p) { uint32_t v; memcpy(&v, p, 4); return v; }

int dc_boot_exe(const void* exe, size_t size)
{
    const uint8_t* h = (const uint8_t*)exe;
    if (size < 0x800 || memcmp(h, "PS-X EXE", 8) != 0) return -1;
    uint32_t pc = rd32(h + 0x10), gp = rd32(h + 0x14), taddr = rd32(h + 0x18), tsize = rd32(h + 0x1C);
    uint32_t baddr = rd32(h + 0x28), bsize = rd32(h + 0x2C), saddr = rd32(h + 0x30), ssize = rd32(h + 0x34);
    if (tsize > size - 0x800) tsize = (uint32_t)(size - 0x800);
    memcpy(dc_ram_ptr(taddr), h + 0x800, tsize);
    if (bsize) memset(dc_ram_ptr(baddr), 0, bsize);

    CPUState* cpu = &g_dc_cpu;
    memset(cpu, 0, sizeof *cpu);
    cpu->read_word = dc_read32;  cpu->write_word = dc_write32;
    cpu->read_half = dc_read16;  cpu->write_half = dc_write16;
    cpu->read_byte = dc_read8;   cpu->write_byte = dc_write8;
    dc_io_reset();
    dc_bios_init(cpu);
    dc_game_init(cpu);
    cpu->gpr[28] = gp;
    cpu->gpr[29] = cpu->gpr[30] = saddr ? saddr + ssize : 0x801FFFF0u;
    cpu->gpr[31] = DC_SENTINEL_RA;
    char b[128];
    snprintf(b, sizeof b, "[boot] text %08X+%X entry %08X gp %08X sp %08X\n", (unsigned)taddr,
             (unsigned)tsize, (unsigned)pc, (unsigned)gp, (unsigned)cpu->gpr[29]);
    dc_host_log(b);
    dc_host_start_vblank_timer();
    dc_run(cpu, pc, DC_SENTINEL_RA);
    dc_fatal("main returned");
    return 0;
}
