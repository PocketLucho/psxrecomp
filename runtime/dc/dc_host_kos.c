/* dc_host_kos.c - Dreamcast platform layer on KallistiOS.
 *
 * Boots the PS-X EXE found next to the program, reads the game disc from a
 * raw 2352-byte image file, takes VBlank from the PowerVR's interrupt, reads
 * the first controller through maple, and shows the PS1 display area in a
 * 640x480 RGB555 framebuffer (pixel doubled; the PowerVR renderer comes later).
 *
 * File locations (first that opens wins):
 *   EXE:  /cd/PSX.EXE, /pc/PSX.EXE, /rd/PSX.EXE
 *   disc: /cd/DISC.BIN, /pc/DISC.BIN
 */
#include "dc_runtime.h"
#include "dc_overlay.h"
#include "dc_gpu.h"
#include <kos.h>
#include <dc/vblank.h>
#include <dc/maple.h>
#include <dc/maple/controller.h>
#include <dc/video.h>
#include <arch/timer.h>
#include <arch/cache.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

KOS_INIT_FLAGS(INIT_DEFAULT);

int dc_boot_exe(const void* exe, size_t size);

void dc_host_log(const char* s) { dbgio_write_str(s); }

uint64_t dc_host_ticks_us(void) { return timer_us_gettime64(); }

int dc_host_read_file(const char* path, void** data, size_t* size)
{
    FILE* f = fopen(path, "rb"); if (!f) return -1;
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    void* p = malloc((size_t)n);
    if (!p || fread(p, 1, (size_t)n, f) != (size_t)n) { fclose(f); free(p); return -1; }
    fclose(f); *data = p; *size = (size_t)n; return 0;
}

/* ---- disc ------------------------------------------------------------- */
static file_t s_disc = FILEHND_INVALID;

int dc_host_cd_read_sector(uint32_t lba, uint8_t* out)
{
    if (s_disc == FILEHND_INVALID) return -1;
    if (fs_seek(s_disc, (off_t)lba * 2352, SEEK_SET) < 0) return -1;
    return fs_read(s_disc, out, 2352) == 2352 ? 0 : -1;
}

/* ---- loadable overlays ---------------------------------------------- */
int dc_host_load_overlay(const char* file, void* dst, uint32_t max)
{
    static const char* const dirs[] = {"/cd/OV/", "/pc/OV/", "/rd/OV/", 0};
    for (int i = 0; dirs[i]; i++) {
        char path[128]; snprintf(path, sizeof path, "%s%s", dirs[i], file);
        file_t f = fs_open(path, O_RDONLY);
        if (f == FILEHND_INVALID) continue;
        ssize_t n = fs_read(f, dst, max);
        fs_close(f);
        return n < 0 ? -1 : (int)n;
    }
    return -1;
}

void dc_host_code_written(void* p, uint32_t n)
{
    dcache_wback_range((uintptr_t)p, n);
    icache_sync_range((uintptr_t)p, n);
}

/* ---- VBlank ----------------------------------------------------------- */
static volatile unsigned s_vblanks;

unsigned dc_host_take_vblanks(void) { return __atomic_exchange_n(&s_vblanks, 0u, __ATOMIC_ACQ_REL); }

/* PowerVR VBlank interrupt (60 Hz on NTSC/VGA). It only counts; the core
 * raises the PS1 interrupt from the guest thread at its next poll. */
static void on_vblank(uint32_t code, void* data)
{
    (void)code; (void)data;
    s_vblanks++;
    g_irq_pending = 1;
}

void dc_host_start_vblank_timer(void) { vblank_handler_add(on_vblank, 0); }

/* ---- controller ------------------------------------------------------- */
/* Dreamcast pad to PS1 bits: A=Cross, B=Circle, X=Square, Y=Triangle,
 * Start=Start, L/R triggers=L1/R1, D-pad as is. Start+A+B+X+Y is left to
 * the BIOS-less environment (no reset combo). */
uint16_t dc_host_pad_buttons(int port)
{
    maple_device_t* dev = maple_enum_type(port, MAPLE_FUNC_CONTROLLER);
    if (!dev) return 0;
    cont_state_t* st = (cont_state_t*)maple_dev_status(dev);
    if (!st) return 0;
    uint32_t b = st->buttons;
    uint16_t m = 0;
    if (b & CONT_START)      m |= 0x0008;
    if (b & CONT_DPAD_UP)    m |= 0x0010;
    if (b & CONT_DPAD_RIGHT) m |= 0x0020;
    if (b & CONT_DPAD_DOWN)  m |= 0x0040;
    if (b & CONT_DPAD_LEFT)  m |= 0x0080;
    if (st->ltrig > 128)     m |= 0x0400;
    if (st->rtrig > 128)     m |= 0x0800;
    if (b & CONT_Y)          m |= 0x1000;
    if (b & CONT_B)          m |= 0x2000;
    if (b & CONT_A)          m |= 0x4000;
    if (b & CONT_X)          m |= 0x8000;
    return m;
}

/* ---- video ------------------------------------------------------------ */
/* PS1 15-bit is BGR (R in the low bits); the framebuffer is RGB555. */
static uint16_t s_bgr2rgb[32768];

static void video_init(void)
{
    vid_set_mode(DM_640x480, PM_RGB555);
    vid_clear(0, 0, 0);
    for (unsigned c = 0; c < 32768; c++)
        s_bgr2rgb[c] = (uint16_t)(((c & 31) << 10) | (c & (31 << 5)) | ((c >> 10) & 31));
}

/* Copy the PS1 display area to the 640x480 framebuffer, doubling pixels for
 * 320-wide modes and lines for 240-line modes. 24-bit (movie) modes are not
 * shown yet. */
void dc_host_vblank(void)
{
    static unsigned n;
    if (++n < 180) return;          /* leave the boot messages up for ~3 s */
    DcGpuDisplay d; dc_gpu_display(&d);
    if (!d.enabled || d.depth24 || d.w <= 0 || d.h <= 0) return;
    int sx = d.w <= 320 ? 2 : 1, sy = d.h <= 240 ? 2 : 1;
    int w = d.w * sx > 640 ? 640 / sx : d.w, h = d.h * sy > 480 ? 480 / sy : d.h;
    for (int y = 0; y < h; y++) {
        const uint16_t* src = &g_dc_vram[((d.y + y) & 511) * 1024];
        uint16_t* dst = vram_s + (y * sy) * 640;
        if (sx == 2) {
            uint32_t* d32 = (uint32_t*)dst;
            for (int x = 0; x < w; x++) {
                uint32_t p = s_bgr2rgb[src[(d.x + x) & 1023] & 0x7FFF];
                d32[x] = p | (p << 16);
            }
        } else {
            for (int x = 0; x < w; x++) dst[x] = s_bgr2rgb[src[(d.x + x) & 1023] & 0x7FFF];
        }
        if (sy == 2) memcpy(dst + 640, dst, (size_t)(w * sx) * 2);
    }
}

/* ---- entry ------------------------------------------------------------ */
static int open_first(const char* const* paths, void** data, size_t* size, const char** used)
{
    for (int i = 0; paths[i]; i++)
        if (dc_host_read_file(paths[i], data, size) == 0) { *used = paths[i]; return 0; }
    return -1;
}

/* Boot steps are printed on screen (KOS debug output on the framebuffer) with
 * a pause after each, so a crash leaves the last step reached visible. The
 * PS1 picture replaces them once the game has run for a few seconds. */
static void step(const char* msg)
{
    dbglog(DBG_INFO, "%s\n", msg);
    thd_sleep(700);
}

/* Keep the screen as it is after a fatal error instead of rebooting. */
static void hang_at_exit(void) { for (;;) thd_sleep(1000); }

static void* s_exe; static size_t s_exe_size;

static void* game_thread(void* arg)
{
    (void)arg;
    step("[xg] 5/5 starting the game");
    return (void*)(intptr_t)dc_boot_exe(s_exe, s_exe_size);
}

int main(int argc, char** argv)
{
    (void)argc; (void)argv;
    video_init();
    dbgio_dev_select("fb");
    atexit(hang_at_exit);
    step("[xg] XenoTest Dreamcast - diagnostic boot");
    step("[xg] 1/5 KallistiOS and video ready");
    static const char* const exe_paths[] = {"/cd/PSX.EXE", "/pc/PSX.EXE", "/rd/PSX.EXE", 0};
    static const char* const disc_paths[] = {"/cd/DISC.BIN", "/pc/DISC.BIN", 0};
    const char* used;
    if (open_first(exe_paths, &s_exe, &s_exe_size, &used)) { step("[xg] ERROR: PSX.EXE not found"); return 1; }
    dbglog(DBG_INFO, "[xg] 2/5 EXE %s (%u bytes)\n", used, (unsigned)s_exe_size);
    thd_sleep(700);
    for (int i = 0; disc_paths[i] && s_disc == FILEHND_INVALID; i++)
        s_disc = fs_open(disc_paths[i], O_RDONLY);
    step(s_disc == FILEHND_INVALID ? "[xg] 3/5 WARNING: DISC.BIN not found" : "[xg] 3/5 disc image open");
    uint8_t sec[2352];
    step(dc_host_cd_read_sector(16, sec) == 0 ? "[xg] 4/5 disc sector read ok" : "[xg] 4/5 ERROR: disc sector read failed");
    /* The kernel thread has a 64 KB stack; give the game 1 MB. */
    kthread_attr_t at = {0};
    at.stack_size = 1024 * 1024;
    at.label = "game";
    kthread_t* t = thd_create_ex(&at, game_thread, 0);
    if (!t) { step("[xg] ERROR: cannot create the game thread"); return 1; }
    void* rv = 0;
    thd_join(t, &rv);
    return (int)(intptr_t)rv;
}
