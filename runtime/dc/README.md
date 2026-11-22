# Dreamcast runtime (experimental)

Minimal runtime for code generated with `psxrecomp-game --target dc`. It does
not use the PC runtime (`runtime/src`): it implements only the ~30 symbols the
DC output links against.

| File | Contents |
|---|---|
| `dc_runtime.h` | Interfaces between the core and the platform layer |
| `dc_mem.c` | Guest RAM and scratchpad, slow-path accessors, PS1 I/O: IRQ controller, DMA (OTC, GPU linked list), timers, CD-ROM command/response queue, GPU status |
| `dc_core.c` | CPS dispatch trampoline, interrupt delivery (in place, as `--target dc` requires), glue symbols, PS-X EXE boot |
| `dc_overlay.h`, `dc_overlay.c` | Overlay selection: on a dispatch miss in the main program, runs the registered overlay whose range holds the address and whose sample code words match guest RAM |
| `dc_bios.c` | High-level BIOS: A0/B0/C0 services, kernel tables, events, `SysEnqIntRP` chains, `HookEntryInt`/`ReturnFromException`, critical sections |
| `dc_host_linux.c` | Test platform layer: runs as a Linux SH-4 program under `qemu-sh4`, 60 Hz VBlank from a host thread |

A KallistiOS platform layer will replace `dc_host_linux.c` on hardware.

## Build and run under qemu-sh4

```sh
R=psxrecomp/runtime
CFLAGS="-std=gnu11 -O2 -m4 -ml -DPSX_NO_DEBUG_TOOLS -I$R/include -I$R/dc"
for f in generated/*_full_*.c generated/*_dispatch.c; do
  sh4-linux-gnu-gcc $CFLAGS -Igenerated -c $f -o obj/$(basename $f .c).o
done
for f in dc_mem dc_core dc_bios dc_overlay dc_host_linux; do
  sh4-linux-gnu-gcc $CFLAGS -c $R/dc/$f.c -o obj/$f.o
done
# overlays (optional): objects and registry from the game's overlay build
sh4-linux-gnu-gcc $CFLAGS -c ov/dc_overlay_registry.c -o obj/registry.o
sh4-linux-gnu-gcc -static -pthread -o xg_dc obj/*.o ov/obj/*.o
# arguments: EXE, CPU-time limit in seconds, raw 2352-byte disc image (.bin)
DC_VBLANK_DIV=1 qemu-sh4 xg_dc path/to/SLUS_006.64 60 path/to/disc1.bin
```

`DC_VBLANK_DIV` sets the VBlank rate as a divider of a 600 Hz tick (10 =
60 Hz, the default). qemu runs the guest much faster than a PS1, so PSY-Q's
iteration-count timeouts (for example in `VSync`) fire before a 60 Hz VBlank;
`DC_VBLANK_DIV=1` avoids most of them. `DC_CD_TRACE=1` logs CD interrupts.

## Overlays

Each overlay is compiled on its own with `psxrecomp-game --overlay --target dc`.
The game's build wraps the output in one translation unit with the dispatcher
renamed (`#define psx_dispatch_game_compiled ov_<n>_dispatch`), then runs
`objcopy --keep-global-symbol=ov_<n>_dispatch` so the `func_XXXXXXXX` names of
overlays that share a load address do not collide. It also generates
`dc_overlay_registry.c` with one `DcOverlay` per image (see `dc_overlay.h`).
Without a registry a weak empty one is linked. For Xenogears this is
`dreamcast/build_overlays.py` in the game repository.

## Game hooks

`dc_game_init(CPUState*)` is weak in the runtime and called once before the
EXE runs. A game overrides it to register `dc_overlay_skip_fn()` entries:
calls to that overlay function return at once to `$ra`, after an optional
callback that leaves the state the skipped code would have (for example
"choose the next scene"). The Linux host also reads
`DC_SKIP=overlay-id@hexaddr[,...]` for quick experiments.

## Status

Xenogears' main EXE boots, runs its library initialisation (pad, card, CD,
SPU, interrupt manager), receives VBlank interrupts through its own handler,
reads real sectors from the disc, uploads sound data to SPU RAM and loads its
first overlays. With the 22 retail overlays linked, the runtime selects the
movie overlay and the STR library and the game starts the opening FMV, which
then retries its stream reads: MDEC and streaming reads (ReadS) are not
emulated yet. With the FMV skipped (Xenogears' `dreamcast/xg_dc_game.c`), the
game loads the field overlay and runs its frame loop: it issues GTE commands
and GPU display lists every frame (still neither executed nor drawn).

The test host drives VBlank from a thread, not from `SIGALRM`: qemu-sh4 can
corrupt guest registers when a signal arrives next to an SH-4 delayed branch,
which made about half the runs crash at random (bad continuations, host
SIGSEGV, stack-protector aborts). With the thread, 8 of 8 runs reach the time
limit. `[trail]` in fatal reports lists the last 16 dispatched addresses.

Linked statically, the 22 overlays add about 12.6 MB of SH-4 code, too much to
keep resident next to the main program in the Dreamcast's 16 MB. On hardware
the overlay code will have to be loaded on demand.

Not done yet: GTE commands are counted but not executed, the GPU only counts
GP0 words, the SPU stores sound RAM but does not mix voices, no MDEC.
