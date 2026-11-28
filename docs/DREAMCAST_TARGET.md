# Dreamcast (SH-4) target — experimental

`psxrecomp-game ... --target dc` rewrites every emitted translation unit for a
small 32-bit host instead of the PC runtime. The default (`--target pc`) output
is unchanged and stays the reference.

What the DC output does (`recompiler/src/dc_target.cpp`):

1. Guest stores use `DC_SW/DC_SH/DC_SB` and loads use inline fast paths from
   `runtime/include/dc_target.h`, not the host `write_*` function pointers.
   Tooling hooks (PGXP, provenance, store-PC attribution, call logging) and
   per-instruction cycle charging compile to nothing.
2. Each generated function keeps the MIPS GPRs and HI/LO it touches in C locals.
   They are written back before guest calls, dispatch, syscalls, exceptions,
   interrupt service and returns, and reloaded after them.
3. `if (psx_check_interrupts_at(cpu, X)) return;` shares one cold write-back stub
   per function (computed goto), so the register cache does not grow code size.
4. Interrupts are polled on back-edges only: a check whose constant target label
   has not been emitted yet (a forward branch) is dropped, since every forward
   path reaches a back-edge, call, dispatch or return. Set
   `PSXRECOMP_DC_IRQ_ALL_BRANCHES=1` when generating to keep a poll on every branch.

Memory model: by default main RAM (mirrored to 8 MB) and the scratchpad are
inline and everything else calls `dc_*_slow()`. With `-DDC_GUEST_RAM_MMU` every
access is mask + base; the DC runtime is expected to map guest RAM with the SH-4
MMU and service MMIO from the fault handler.

## Measurements (SH-4 instructions executed per MIPS instruction, qemu-sh4)

Synthetic PS1-style workloads in `tools/dc_bench/` (vertex transform, script VM,
ordering-table build, LZ decompression):

| Variant | Vertices | VM | OT | LZ |
|---|---|---|---|---|
| PC output, faithful timing (hooks stubbed) | 166 | 174 | 207 | — |
| PC output, no cycle timing | 9.2 | 9.5 | 10.4 | 23 |
| `--target dc`, poll on every branch | 6.4 | 5.1 | 5.2 | 17 |
| same + `DC_GUEST_RAM_MMU` | 5.5 | 4.5 | 3.9 | 16 |
| `--target dc` (back-edge polling) + `DC_GUEST_RAM_MMU` | 5.2 | 3.3 | 3.4 | 15 |
| same, call-return continuations only | 4.9 | 3.2 | 3.1 | 15 |
| Same C compiled natively for SH-4 | 3.7 | 1.3 | 1.3 | 2.5 |

The LZ case is dominated by function fragmentation (loop blocks emitted as
separate functions re-entered through dispatch).

Xenogears (USA-serial disc-1 EXE) main executable compiled for SH-4:

| Output | Size |
|---|---|
| PC output, no cycle timing | 5.45 MB |
| `--target dc` (back-edge polling) | 3.34 MB |
| same + `DC_GUEST_RAM_MMU` | 2.03 MB |
| `--target dc` (data sections no longer split) | 2.12 MB |
| same + `DC_GUEST_RAM_MMU` | 1.04 MB |
| `--target dc` (call-return continuations only) | 1.69 MB |
| same + `DC_GUEST_RAM_MMU` | 0.65 MB |

Interrupt poll sites drop from 17,771 to 2,181.

### Data sections no longer mint entry points

The main-EXE split pre-pass (code_generator.cpp) scans every CFG for branch
targets outside it and turns them into new functions. That included the CFGs of
data sections, which are only emitted as dispatch stubs: their words decode as
random branches, and each one minted a function. In Xenogears that pre-pass
added 792 entries, most of them translating data as code, and several gap
functions of one pass all ran to the same next start, so the same bytes were
emitted up to 24 times (generated C 1.87x the size of its unique blocks).

With `--target dc` the pre-pass skips data-section CFGs
(`CodeGeneratorConfig::split_skip_data_sources`). Xenogears then adds 15 entries,
the duplication drops to 1.09x (host + alias body only) and out-of-function
fallthroughs drop from 537 to 14. Every entry that disappears was reached only
from a data section; branches and call returns from real code are still split as
before. The PC output does not change.

### Entry switches

Every PC function starts with two switches over its block leaders: one for
savestate resume (`g_psx_resume_seed`) and one for CPS continuations
(`cpu->pc`), which registers every leader so an interrupt escape can resume at
any of them. With `--target dc` the savestate switch is not emitted and only
call returns (the instruction after a `jal`/`jalr` delay slot) are registered as
continuations. Xenogears goes from 18,505 to 2,983 switch cases.

This sets a contract for the DC runtime: `dc_irq_slow` must service the
interrupt in place and return 0. A runtime that needs to escape at an interrupt
poll must generate with `PSXRECOMP_DC_RESUME_ALL_LEADERS=1`.

## Still to do

- Remaining fragmentation: 15 split entries, 14 out-of-function fallthroughs,
  and alias hosts emitted twice (host body + shared alias body).
- A KallistiOS runtime: dispatch, GTE, CD, GPU translation, SPU→AICA.
