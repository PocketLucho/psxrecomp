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
| Same C compiled natively for SH-4 | 3.7 | 1.3 | 1.3 | 2.5 |

The LZ case is dominated by function fragmentation (loop blocks emitted as
separate functions re-entered through dispatch).

Xenogears (USA-serial disc-1 EXE) main executable compiled for SH-4:

| Output | Size |
|---|---|
| PC output, no cycle timing | 5.45 MB |
| `--target dc` (back-edge polling) | 3.34 MB |
| same + `DC_GUEST_RAM_MMU` | 2.03 MB |

Interrupt poll sites drop from 17,771 to 2,181.

## Still to do

- Function fragmentation. Two causes seen in the benchmark: a JAL target whose
  prologue is preceded by setup code becomes an 8-byte function plus a separate
  prologue function, and jump-table case targets become function entries that
  each span to the end of the switch (duplicating the switch body and routing
  cases through dispatch). Xenogears shows 537 fallthroughs into the next
  function and 108 out-of-function branches.
- Drop entry resume switches that nothing can reach.
- A KallistiOS runtime: dispatch, GTE, CD, GPU translation, SPU→AICA.
