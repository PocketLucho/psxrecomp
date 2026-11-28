/* dc_target.h - overrides for generated code built with `psxrecomp-game --target dc`.
 *
 * Included by every TU the recompiler rewrites for the Dreamcast (SH-4) runtime,
 * right after the TU's normal declarations. It replaces the PC runtime's
 * tooling hooks and per-instruction timing with direct guest-memory access.
 *
 * Guest memory model:
 *   default            main RAM (2 MB, mirrored to 8 MB) and the 1 KB scratchpad
 *                      are accessed inline; everything else calls dc_*_slow().
 *   DC_GUEST_RAM_MMU   every access is mask + base. The DC runtime maps guest RAM
 *                      with the SH-4 MMU and services MMIO from the TLB-miss /
 *                      address-error handler, so no range check is emitted.
 *
 * The DC runtime provides: g_dc_ram, g_dc_scratch, g_irq_pending, dc_irq_slow
 * and the dc_*_slow accessors.
 *
 * dc_irq_slow must service the interrupt in place and return 0: only call
 * returns are registered as resumable continuations, unless the code was
 * generated with PSXRECOMP_DC_RESUME_ALL_LEADERS=1.
 */
#pragma once
#include <stdint.h>

extern uint8_t g_dc_ram[0x200000];
extern uint8_t g_dc_scratch[1024];
extern volatile int g_irq_pending;
uint32_t dc_lw_slow(uint32_t a);
uint32_t dc_lh_slow(uint32_t a);
uint32_t dc_lb_slow(uint32_t a);
void dc_sw_slow(uint32_t a, uint32_t v);
void dc_sh_slow(uint32_t a, uint32_t v);
void dc_sb_slow(uint32_t a, uint32_t v);
int dc_irq_slow(CPUState* cpu, uint32_t pc);

#define DC_P(a) ((a) & 0x1FFFFFFFu)

#if defined(DC_GUEST_RAM_MMU)
#define DC_FAST(T, a, slow) ((uint32_t)*(const T*)(g_dc_ram + ((a) & 0x1FFFFFu)))
#define DC_STORE(T, a, v, slow) (*(T*)(g_dc_ram + ((a) & 0x1FFFFFu)) = (T)(v))
#else
#define DC_FAST(T, a, slow) ({ uint32_t _a = (a), _p = DC_P(_a); \
    __builtin_expect(_p < 0x00800000u, 1) ? (uint32_t)*(const T*)(g_dc_ram + (_p & 0x1FFFFFu)) : \
    ((_p - 0x1F800000u) < 0x400u ? (uint32_t)*(const T*)(g_dc_scratch + (_p & 0x3FFu)) : slow(_a)); })
#define DC_STORE(T, a, v, slow) do { uint32_t _a = (a), _p = DC_P(_a); uint32_t _v = (v); \
    if (__builtin_expect(_p < 0x00800000u, 1)) *(T*)(g_dc_ram + (_p & 0x1FFFFFu)) = (T)_v; \
    else if ((_p - 0x1F800000u) < 0x400u) *(T*)(g_dc_scratch + (_p & 0x3FFu)) = (T)_v; \
    else slow(_a, _v); } while (0)
#endif

#define DC_SW(a, v) DC_STORE(uint32_t, a, v, dc_sw_slow)
#define DC_SH(a, v) DC_STORE(uint16_t, a, v, dc_sh_slow)
#define DC_SB(a, v) DC_STORE(uint8_t,  a, v, dc_sb_slow)

#undef PGXP_LOAD
#undef PGXP_STORE
#undef PGXP_ALU
#undef PGXP_MULDIV
#undef PGXP_COP2
#define PGXP_LOAD(...) ((void)0)
#define PGXP_STORE(...) ((void)0)
#define PGXP_ALU(...) ((void)0)
#define PGXP_MULDIV(...) ((void)0)
#define PGXP_COP2(...) ((void)0)
#define psx_store_cycle_barrier() ((void)0)
#define debug_server_log_call_entry(x) ((void)0)
#define gte_precision_store_word(...) ((void)0)
#define psx_cyc_load_word(cpu, a, rt, m) DC_FAST(uint32_t, a, dc_lw_slow)
#define psx_cyc_load_half(cpu, a, rt, m) ((uint16_t)DC_FAST(uint16_t, a, dc_lh_slow))
#define psx_cyc_load_byte(cpu, a, rt, m) ((uint8_t)DC_FAST(uint8_t, a, dc_lb_slow))
#define psx_check_interrupts_at(cpu, pc) (__builtin_expect(g_irq_pending, 0) ? dc_irq_slow(cpu, pc) : 0)
#define psx_muldiv_stall(...) ((void)0)
#define psx_muldiv_set(...) ((void)0)
