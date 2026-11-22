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
# arguments: EXE, CPU-time limit in seconds, raw 2352-byte disc image (.bin)
DC_VBLANK_DIV=1 qemu-sh4 xg_dc path/to/SLUS_006.64 60 path/to/disc1.bin
```

`DC_VBLANK_DIV` sets the VBlank rate as a divider of a 600 Hz tick (10 =
60 Hz, the default). qemu runs the guest much faster than a PS1, so PSY-Q's
iteration-count timeouts (for example in `VSync`) fire before a 60 Hz VBlank;
`DC_VBLANK_DIV=1` avoids most of them. `DC_CD_TRACE=1` logs CD interrupts.

## Status

Xenogears' main EXE boots, runs its library initialisation (pad, card, CD,
SPU, interrupt manager), receives VBlank interrupts through its own handler,
reads real sectors from the disc (Setloc/ReadN/Pause, CD DMA), uploads sound
data to SPU RAM and sends its first GPU commands. It then jumps to
`0x800737EC`, code it loaded from the disc into RAM above the main EXE: the
overlays have to be compiled for `--target dc` next.

Not done yet: GTE commands are counted but not executed, the GPU only counts
GP0 words, the SPU stores sound RAM but does not mix voices.
