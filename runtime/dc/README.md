# Dreamcast runtime (experimental)

Minimal runtime for code generated with `psxrecomp-game --target dc`. It does
not use the PC runtime (`runtime/src`): it implements only the ~30 symbols the
DC output links against.

| File | Contents |
|---|---|
| `dc_runtime.h` | Interfaces between the core and the platform layer |
| `dc_mem.c` | Guest RAM and scratchpad, slow-path accessors, PS1 I/O: IRQ controller, DMA (OTC, GPU linked list), timers, CD-ROM command/response queue, GPU status |
| `dc_core.c` | CPS dispatch trampoline, interrupt delivery (in place, as `--target dc` requires), glue symbols, PS-X EXE boot |
| `dc_bios.c` | High-level BIOS: A0/B0/C0 services, kernel tables, events, `SysEnqIntRP` chains, `HookEntryInt`/`ReturnFromException`, critical sections |
| `dc_host_linux.c` | Test platform layer: runs as a Linux SH-4 program under `qemu-sh4`, 60 Hz VBlank from `SIGALRM` |

A KallistiOS platform layer will replace `dc_host_linux.c` on hardware.

## Build and run under qemu-sh4

```sh
R=psxrecomp/runtime
CFLAGS="-std=gnu11 -O2 -m4 -ml -DPSX_NO_DEBUG_TOOLS -I$R/include -I$R/dc"
for f in generated/*_full_*.c generated/*_dispatch.c; do
  sh4-linux-gnu-gcc $CFLAGS -Igenerated -c $f -o obj/$(basename $f .c).o
done
for f in dc_mem dc_core dc_bios dc_host_linux; do
  sh4-linux-gnu-gcc $CFLAGS -c $R/dc/$f.c -o obj/$f.o
done
sh4-linux-gnu-gcc -static -o xg_dc obj/*.o
qemu-sh4 xg_dc path/to/SLUS_006.64 20   # second argument: CPU-time limit in seconds
```

## Status

Xenogears' main EXE boots, runs its library initialisation (pad, card, CD,
interrupt manager), receives VBlank interrupts through its own handler, prints
`CD_init:` and starts reading the disc (`Setloc 24:12:04`, `ReadN`). Disc
access is the next step. GTE commands are counted but not executed yet, and
the GPU only counts GP0 words.
