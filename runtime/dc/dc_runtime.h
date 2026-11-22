/* dc_runtime.h - minimal runtime for code generated with `psxrecomp-game --target dc`.
 *
 * Platform-neutral core: guest memory and I/O, the CPS dispatch trampoline,
 * a high-level BIOS and interrupt delivery. A platform layer (dc_host_*.c)
 * supplies file loading, timing and output: dc_host_linux.c runs the core
 * under qemu-sh4 for testing; a KallistiOS layer replaces it on hardware.
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "cpu_state.h"

/* ---- guest memory ---- */
extern uint8_t g_dc_ram[0x200000];
extern uint8_t g_dc_scratch[1024];
extern volatile int g_irq_pending;

static inline uint32_t dc_phys(uint32_t a) { return a & 0x1FFFFFFFu; }
static inline uint8_t* dc_ram_ptr(uint32_t a) { return g_dc_ram + (dc_phys(a) & 0x1FFFFFu); }
uint32_t dc_read32(uint32_t a);
uint16_t dc_read16(uint32_t a);
uint8_t  dc_read8(uint32_t a);
void     dc_write32(uint32_t a, uint32_t v);
void     dc_write16(uint32_t a, uint16_t v);
void     dc_write8(uint32_t a, uint8_t v);

/* ---- I/O ---- */
void     dc_io_reset(void);
uint32_t dc_io_read(uint32_t phys, int size);
void     dc_io_write(uint32_t phys, uint32_t v, int size);
void     dc_irq_raise(uint32_t bits);   /* sets I_STAT bits, arms g_irq_pending */
uint32_t dc_io_istat(void);
uint32_t dc_io_imask(void);
void     dc_io_ack(uint32_t bits);
void     dc_io_poll(void);              /* timed device events (CD sectors) */

/* ---- execution ---- */
extern CPUState g_dc_cpu;
/* Run guest code at addr until it returns to stop_ra (a sentinel $ra), or forever if 0. */
void dc_run(CPUState* cpu, uint32_t addr, uint32_t stop_ra);
/* Call a guest function with up to 4 args and return v0 (nested, preserves caller state). */
uint32_t dc_call_guest(CPUState* cpu, uint32_t fn, uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3);
void dc_fatal(const char* fmt, ...);
void dc_profile_sample(void);
int  dc_irq_slow(CPUState* cpu, uint32_t pc);

/* Game hook, called once after the BIOS is set up and before the EXE runs.
 * The runtime's version does nothing; a game links its own (overlay skips). */
void dc_game_init(CPUState* cpu);

/* ---- BIOS HLE ---- */
void dc_bios_init(CPUState* cpu);
int  dc_bios_call(CPUState* cpu, uint32_t vector);   /* 1 if vector is A0/B0/C0 */
int  dc_bios_syscall(CPUState* cpu, uint32_t code);
void dc_bios_interrupt(CPUState* cpu);               /* run the guest's IRQ handlers */
int  dc_bios_irq_enabled(void);
/* Kernel stub window: [DC_KTAB_BASE, +0xC00) are the A0/B0/C0 table targets,
 * the rest of the window is BIOS code that games reach through those tables. */
#define DC_KSTUB_LO  0x00001000u
#define DC_KSTUB_HI  0x00008000u
#define DC_KTAB_BASE 0x00004000u
void dc_bios_kernel_call(CPUState* cpu, uint32_t phys);

/* ---- statistics ---- */
typedef struct {
    uint64_t dispatches, bios_calls, syscalls, irqs, vblanks;
    uint64_t gp0_words, gp1_writes, dma_gpu, cd_cmds, gte_ops;
    uint32_t last_addr;
} DcStats;
extern DcStats g_dc_stats;

/* ---- platform layer ---- */
void dc_host_log(const char* s);
uint64_t dc_host_ticks_us(void);
int  dc_host_read_file(const char* path, void** data, size_t* size);
void dc_host_start_vblank_timer(void);
/* VBlanks that elapsed since the last call. The host's timer only counts
 * them and sets g_irq_pending; the core raises the interrupt from dc_io_poll,
 * so device state is touched by the guest thread alone. */
unsigned dc_host_take_vblanks(void);
/* Pad buttons on port 0/1, active high, PS1 bit order (bit 3 Start, 14 Cross...). */
uint16_t dc_host_pad_buttons(int port);
/* Called on the guest thread at each delivered VBlank: present a frame. */
void dc_host_vblank(void);
/* Read one raw 2352-byte sector (LBA from the start of the data track); 0 on success. */
int  dc_host_cd_read_sector(uint32_t lba, uint8_t* out2352);
