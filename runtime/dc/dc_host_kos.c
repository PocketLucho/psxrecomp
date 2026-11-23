/* dc_host_kos.c - Dreamcast platform layer on KallistiOS.
 *
 * Boots the PS-X EXE found next to the program, reads the game disc from a
 * raw 2352-byte image file, takes VBlank from the PowerVR's interrupt, reads
 * the first controller through maple, and shows the PS1 display area in a
 * 640x480 RGB555 framebuffer (pixel doubled; the PowerVR renderer comes later).
 *
 * File locations (first that opens wins):
 *   EXE:  /hd/PSX.EXE, /cd/PSX.EXE, /pc/PSX.EXE, /rd/PSX.EXE
 *   disc: /hd/DISC.BIN, /cd/DISC.BIN, /pc/DISC.BIN
 * "/hd/" is the ISO9660 filesystem of the GD-ROM high-density area, read
 * here directly: KallistiOS's /cd only mounts the low-density area of a
 * GD-ROM (track 1), so on a GDI it only sees the small track-1 filesystem.
 */
#include "dc_runtime.h"
#include "dc_overlay.h"
#include "dc_gpu.h"
#include <kos.h>
#include <dc/vblank.h>
#include <dc/maple.h>
#include <dc/maple/controller.h>
#include <dc/video.h>
#include <dc/cdrom.h>
#include <dc/biosfont.h>
#include <dc/sq.h>
#include <dc/sound/stream.h>
#include <arch/timer.h>
#include <arch/cache.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

KOS_INIT_FLAGS(INIT_DEFAULT);

int dc_boot_exe(const void* exe, size_t size);

/* ---- on-screen text ---------------------------------------------------- */
/* Lines of log text drawn with the BIOS font until the PS1 picture takes over
 * the framebuffer; a fatal error stays readable since nothing redraws then. */
static int s_text_y = 16;
static char s_line[96]; static int s_line_n;

extern int g_dc_pvr_on;
int dc_gpu_pvr_init(void);
void dc_gpu_pvr_vblank(void);

static void screen_line(const char* t)
{
    if (g_dc_pvr_on) return;        /* the PowerVR owns the screen now */
    if (s_text_y + 24 > 480) { vid_clear(0, 0, 0); s_text_y = 16; }
    char buf[56]; int n = 0;
    for (; *t && n < 52; t++) buf[n++] = *t;
    buf[n] = 0;
    bfont_draw_str(vram_s + s_text_y * 640 + 12, 640, 1, buf);
    s_text_y += 24;
}

void dc_host_log(const char* s)
{
    dbgio_write_str(s);
    for (; *s; s++) {
        if (*s == '\n' || s_line_n == (int)sizeof s_line - 1) {
            s_line[s_line_n] = 0; screen_line(s_line); s_line_n = 0;
            if (*s == '\n') continue;
        }
        if (*s != '\r') s_line[s_line_n++] = *s;
    }
}

/* ---- GD-ROM high-density area ------------------------------------------ */
/* Sectors are read in 32-sector chunks through a cache, which serves the
 * 2352-byte disc-image sectors (they straddle 2048-byte sectors) and
 * sequential reads with few drive commands. FADs are LBA + 150. */
#define HD_CHUNK 32
static uint8_t s_hd_buf[HD_CHUNK * 2048] __attribute__((aligned(32)));
static uint32_t s_hd_buf_fad = 0xFFFFFFFFu;
static uint32_t s_hd_root_fad, s_hd_root_size;
static int s_hd_ok = -1;                       /* -1 untried, 0 no, 1 yes */
static mutex_t s_hd_lock = MUTEX_INITIALIZER;

static int hd_chunk(uint32_t fad)
{
    uint32_t base = fad - fad % HD_CHUNK;
    if (base == s_hd_buf_fad) return 0;
    s_hd_buf_fad = 0xFFFFFFFFu;
    for (int tries = 0; tries < 3; tries++)
        if (cdrom_read_sectors(s_hd_buf, base, HD_CHUNK) == ERR_OK) { s_hd_buf_fad = base; return 0; }
    /* the chunk may run past the end of the disc: read sector by sector */
    for (uint32_t i = 0; i < HD_CHUNK; i++)
        if (cdrom_read_sectors(s_hd_buf + i * 2048, base + i, 1) != ERR_OK) {
            if (base + i <= fad) return -1;
            break;
        }
    s_hd_buf_fad = base;
    return 0;
}

/* Read `len` bytes at byte offset `off` of the extent starting at `fad`. */
static int hd_read(uint32_t fad, uint64_t off, void* out, uint32_t len)
{
    uint8_t* o = out;
    mutex_lock(&s_hd_lock);
    while (len) {
        uint32_t sec = fad + (uint32_t)(off / 2048), in = (uint32_t)(off % 2048);
        if (hd_chunk(sec)) { mutex_unlock(&s_hd_lock); return -1; }
        uint32_t avail = (s_hd_buf_fad + HD_CHUNK - sec) * 2048 - in;
        uint32_t n = len < avail ? len : avail;
        memcpy(o, s_hd_buf + (sec - s_hd_buf_fad) * 2048 + in, n);
        o += n; off += n; len -= n;
    }
    mutex_unlock(&s_hd_lock);
    return 0;
}

static uint32_t le32(const uint8_t* p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

static int hd_init(void)
{
    if (s_hd_ok >= 0) return s_hd_ok ? 0 : -1;
    s_hd_ok = 0;
    cd_toc_t toc;
    if (cdrom_read_toc(&toc, true) != ERR_OK) return -1;   /* no high-density area: a CD */
    uint32_t base = cdrom_locate_data_track(&toc);
    if (!base) return -1;
    uint8_t pvd[2048];
    if (hd_read(base + 16, 0, pvd, 2048) || memcmp(pvd, "\001CD001", 6)) return -1;
    s_hd_root_fad = le32(pvd + 156 + 2) + 150;
    s_hd_root_size = le32(pvd + 156 + 10);
    s_hd_ok = 1;
    return 0;
}

static int name_eq(const uint8_t* iso, int n, const char* want, int wn)
{
    int k = 0;
    while (k < n && iso[k] != ';') k++;
    if (k && iso[k - 1] == '.') k--;
    if (k != wn) return 0;
    for (int i = 0; i < k; i++) {
        char a = (char)iso[i], b = want[i];
        if (a >= 'a' && a <= 'z') a -= 32;
        if (b >= 'a' && b <= 'z') b -= 32;
        if (a != b) return 0;
    }
    return 1;
}

/* Find `path` (relative to the root, '/' separated). */
static int hd_find(const char* path, uint32_t* fad, uint32_t* size)
{
    if (hd_init()) return -1;
    uint32_t dfad = s_hd_root_fad, dsize = s_hd_root_size;
    while (*path == '/') path++;
    while (*path) {
        const char* end = strchr(path, '/');
        int wn = end ? (int)(end - path) : (int)strlen(path);
        int found = 0;
        for (uint32_t off = 0; off < dsize && !found; off += 2048) {
            uint8_t sec[2048];
            if (hd_read(dfad, off, sec, 2048)) return -1;
            for (uint32_t p = 0; p < 2048 && sec[p]; p += sec[p]) {
                const uint8_t* r = sec + p;
                if (r[32] == 1 && (r[33] == 0 || r[33] == 1)) continue;
                if (name_eq(r + 33, r[32], path, wn)) {
                    dfad = le32(r + 2) + 150; dsize = le32(r + 10);
                    found = 1; break;
                }
            }
        }
        if (!found) return -1;
        if (!end) break;
        path = end + 1;
    }
    *fad = dfad; *size = dsize;
    return 0;
}

uint64_t dc_host_ticks_us(void) { return timer_us_gettime64(); }

int dc_host_read_file(const char* path, void** data, size_t* size)
{
    if (!strncmp(path, "/hd/", 4)) {
        uint32_t fad, n;
        if (hd_find(path + 4, &fad, &n)) return -1;
        void* p = malloc(n ? n : 1);
        if (!p || hd_read(fad, 0, p, n)) { free(p); return -1; }
        *data = p; *size = n; return 0;
    }
    FILE* f = fopen(path, "rb"); if (!f) return -1;
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    void* p = malloc((size_t)n);
    if (!p || fread(p, 1, (size_t)n, f) != (size_t)n) { fclose(f); free(p); return -1; }
    fclose(f); *data = p; *size = (size_t)n; return 0;
}

/* ---- disc ------------------------------------------------------------- */
static file_t s_disc = FILEHND_INVALID;
static uint32_t s_disc_fad, s_disc_size;    /* DISC.BIN in the high-density area */

int dc_host_cd_read_sector(uint32_t lba, uint8_t* out)
{
    if (s_disc_fad) {
        if ((uint64_t)(lba + 1) * 2352 > s_disc_size) return -1;
        return hd_read(s_disc_fad, (uint64_t)lba * 2352, out, 2352);
    }
    if (s_disc == FILEHND_INVALID) return -1;
    if (fs_seek(s_disc, (off_t)lba * 2352, SEEK_SET) < 0) return -1;
    return fs_read(s_disc, out, 2352) == 2352 ? 0 : -1;
}

/* ---- loadable overlays ---------------------------------------------- */
int dc_host_load_overlay(const char* file, void* dst, uint32_t max)
{
    char hp[64]; snprintf(hp, sizeof hp, "OV/%s", file);
    uint32_t fad, n;
    if (hd_find(hp, &fad, &n) == 0) {
        if (n > max) n = max;
        return hd_read(fad, 0, dst, n) ? -1 : (int)n;
    }
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
static volatile unsigned s_vblanks, s_vblanks_total;

unsigned dc_host_take_vblanks(void) { return __atomic_exchange_n(&s_vblanks, 0u, __ATOMIC_ACQ_REL); }

/* PowerVR VBlank interrupt (60 Hz on NTSC/VGA). It only counts; the core
 * raises the PS1 interrupt from the guest thread at its next poll. */
static void on_vblank(uint32_t code, void* data)
{
    (void)code; (void)data;
    s_vblanks++; s_vblanks_total++;
#ifdef DC_PROFILE
    { extern uint32_t g_prof_pc[1024], g_prof_pr[1024]; extern volatile unsigned g_prof_n;
      irq_context_t* c = arch_irq_get_context();
      if (c && g_prof_n < 1024) { g_prof_pc[g_prof_n] = c->pc; g_prof_pr[g_prof_n] = c->pr; g_prof_n++; } }
#endif
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

/* ---- CD audio (XA) through an AICA stream ------------------------------ */
/* The CD emulation pushes decoded frames into a ring; the stream callback
 * takes them at the XA rate (37.8 or 18.9 kHz). Everything runs on the guest
 * thread: pushes come from sector reads, polls from VBlanks and pushes. */
#define AUD_RING 32768                      /* stereo frames, ~0.9 s at 37.8 kHz */
static int16_t s_aud[AUD_RING * 2];
static unsigned s_aud_r, s_aud_w;           /* frame indices, free-running */
static unsigned s_aud_rate;
static snd_stream_hnd_t s_aud_hnd = SND_STREAM_INVALID;
static int16_t s_aud_out[8192] __attribute__((aligned(32)));

static void* aud_cb(snd_stream_hnd_t hnd, int req, int* recv)
{
    (void)hnd;
    unsigned want = (unsigned)req / 4;        /* bytes -> stereo frames */
    if (want > sizeof s_aud_out / 4) want = sizeof s_aud_out / 4;
    unsigned have = s_aud_w - s_aud_r, n = have < want ? have : want;
    for (unsigned i = 0; i < n; i++) {
        unsigned k = (s_aud_r + i) % AUD_RING;
        s_aud_out[i * 2] = s_aud[k * 2]; s_aud_out[i * 2 + 1] = s_aud[k * 2 + 1];
    }
    s_aud_r += n;
    if (n < want) memset(s_aud_out + n * 2, 0, (want - n) * 4);
    *recv = (int)(want * 4);
    return s_aud_out;
}

static void audio_poll(void)
{
    if (s_aud_hnd != SND_STREAM_INVALID && s_aud_rate) snd_stream_poll(s_aud_hnd);
}

void dc_host_audio_push(const int16_t* lr, unsigned frames, unsigned rate)
{
    if (s_aud_hnd == SND_STREAM_INVALID) {
        if (snd_stream_init() < 0) return;
        s_aud_hnd = snd_stream_alloc(aud_cb, 16384);
        if (s_aud_hnd == SND_STREAM_INVALID) return;
        snd_stream_volume(s_aud_hnd, 240);
    }
    if (rate != s_aud_rate) {
        if (s_aud_rate) snd_stream_stop(s_aud_hnd);
        s_aud_r = s_aud_w = 0;
        s_aud_rate = rate;
        snd_stream_start(s_aud_hnd, rate, 1);
    }
    for (unsigned i = 0; i < frames; i++) {
        if (s_aud_w - s_aud_r >= AUD_RING) s_aud_r++;     /* full: drop the oldest */
        unsigned k = s_aud_w % AUD_RING;
        s_aud[k * 2] = lr[i * 2]; s_aud[k * 2 + 1] = lr[i * 2 + 1];
        s_aud_w++;
    }
    audio_poll();
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
/* Speed report on the serial console every 10 s of console time: how many
 * VBlanks the guest saw and how many times it moved its display area (one
 * per finished game frame), against the 600 real VBlanks. */
#ifdef DC_PROFILE
uint32_t g_prof_pc[1024], g_prof_pr[1024]; volatile unsigned g_prof_n;
#endif
static void perf_report(const DcGpuDisplay* d)
{
    static unsigned polls, flips, last_total, lx = ~0u, ly = ~0u;
    polls++;
    if ((unsigned)d->x != lx || (unsigned)d->y != ly) { flips++; lx = (unsigned)d->x; ly = (unsigned)d->y; }
    unsigned t = s_vblanks_total;
    if (t - last_total >= 600) {
        char b[96];
        snprintf(b, sizeof b, "[perf] %u vblanks: guest saw %u, %u frames drawn\n", t - last_total, polls, flips);
        dbgio_write_str(b);
        { extern uint32_t g_dc_spu_peak, g_dc_spu_keyons, g_dc_tmr_irqs[3], g_dc_spu_us, g_dc_spu_frames, g_dc_drv_us;
          snprintf(b, sizeof b, "[snd] keyons %u peak %u tmr2 %u mix %u ms/%u fr drv %u ms\n", (unsigned)g_dc_spu_keyons, (unsigned)g_dc_spu_peak,
                   (unsigned)g_dc_tmr_irqs[2], (unsigned)(g_dc_spu_us / 1000), (unsigned)g_dc_spu_frames, (unsigned)(g_dc_drv_us / 1000));
          g_dc_spu_peak = 0; g_dc_spu_us = g_dc_spu_frames = g_dc_drv_us = 0; dbgio_write_str(b); }
        if (g_dc_pvr_on) {
            extern unsigned g_dc_pvr_stat[4];
            snprintf(b, sizeof b, "[pvr] prims %u, software %u, textures %u, scenes %u\n",
                     g_dc_pvr_stat[0], g_dc_pvr_stat[1], g_dc_pvr_stat[2], g_dc_pvr_stat[3]);
            dbgio_write_str(b);
            memset(g_dc_pvr_stat, 0, sizeof g_dc_pvr_stat);
        }
        last_total = t; polls = flips = 0;
#ifdef DC_PROFILE
        unsigned n = g_prof_n;
        for (unsigned i = 0; i < n; i++) {
            snprintf(b, sizeof b, "[pc] %08lx %08lx\n", (unsigned long)g_prof_pc[i], (unsigned long)g_prof_pr[i]);
            dbgio_write_str(b);
        }
        g_prof_n = 0;
#endif
    }
}

void dc_host_vblank(void)
{
    static unsigned n;
    DcGpuDisplay d; dc_gpu_display(&d);
    perf_report(&d);
    audio_poll();
    if (g_dc_pvr_on) { dc_gpu_pvr_vblank(); return; }
    if (++n < 180) return;          /* leave the boot messages up for ~3 s */
    if (!d.enabled || d.depth24 || d.w <= 0 || d.h <= 0) return;
    int sx = d.w <= 320 ? 2 : 1, sy = d.h <= 240 ? 2 : 1;
    int w = d.w * sx > 640 ? 640 / sx : d.w, h = d.h * sy > 480 ? 480 / sy : d.h;
    /* Each output line is built in a cached buffer and sent to video memory
     * with the store queues (32-byte bursts), once or twice for doubled
     * lines: writing video memory a pixel at a time, and reading it back to
     * double a line, cost several milliseconds per frame. */
    static uint32_t line[320] __attribute__((aligned(32)));
    int bytes = ((w * sx * 2) + 31) & ~31;
    for (int y = 0; y < h; y++) {
        const uint16_t* src = &g_dc_vram[((d.y + y) & 511) * 1024];
        if (sx == 2) {
            for (int x = 0; x < w; x++) {
                uint32_t p = s_bgr2rgb[src[(d.x + x) & 1023] & 0x7FFF];
                line[x] = p | (p << 16);
            }
        } else {
            uint16_t* l16 = (uint16_t*)line;
            for (int x = 0; x < w; x++) l16[x] = s_bgr2rgb[src[(d.x + x) & 1023] & 0x7FFF];
        }
        uint16_t* dst = vram_s + (y * sy) * 640;
        sq_cpy(dst, line, (size_t)bytes);
        if (sy == 2) sq_cpy(dst + 640, line, (size_t)bytes);
    }
}

/* ---- entry ------------------------------------------------------------ */
static int open_first(const char* const* paths, void** data, size_t* size, const char** used)
{
    for (int i = 0; paths[i]; i++)
        if (dc_host_read_file(paths[i], data, size) == 0) { *used = paths[i]; return 0; }
    return -1;
}

/* Boot steps are printed on screen with a pause after each, so a crash
 * leaves the last step reached visible. The PS1 picture replaces them once
 * the game has run for a few seconds. */
static void step(const char* msg)
{
    dc_host_log(msg); dc_host_log("\n");
    thd_sleep(700);
}

/* Keep the screen as it is after a fatal error instead of rebooting. */
static void hang_at_exit(void) { for (;;) thd_sleep(1000); }

static void* s_exe; static size_t s_exe_size;

static void* game_thread(void* arg)
{
    (void)arg;
    step("[xg] 5/5 starting the game");
#ifndef DC_SOFT_GPU
    /* PowerVR rendering; build with -DDC_SOFT_GPU for the software one */
    if (dc_gpu_pvr_init()) dc_host_log("[xg] PowerVR init failed, software rendering\n");
#endif
    return (void*)(intptr_t)dc_boot_exe(s_exe, s_exe_size);
}

int main(int argc, char** argv)
{
    (void)argc; (void)argv;
    video_init();
    atexit(hang_at_exit);
    step("[xg] XenoTest Dreamcast - diagnostic boot");
    step("[xg] 1/5 KallistiOS and video ready");
    static const char* const exe_paths[] = {"/hd/PSX.EXE", "/cd/PSX.EXE", "/pc/PSX.EXE", "/rd/PSX.EXE", 0};
    static const char* const disc_paths[] = {"/cd/DISC.BIN", "/pc/DISC.BIN", 0};
    const char* used;
    char msg[96];
    if (open_first(exe_paths, &s_exe, &s_exe_size, &used)) { step("[xg] ERROR: PSX.EXE not found"); return 1; }
    snprintf(msg, sizeof msg, "[xg] 2/5 EXE %s (%u bytes)", used, (unsigned)s_exe_size);
    step(msg);
    if (hd_find("DISC.BIN", &s_disc_fad, &s_disc_size)) s_disc_fad = 0;
    for (int i = 0; disc_paths[i] && !s_disc_fad && s_disc == FILEHND_INVALID; i++)
        s_disc = fs_open(disc_paths[i], O_RDONLY);
    step(!s_disc_fad && s_disc == FILEHND_INVALID ? "[xg] 3/5 WARNING: DISC.BIN not found" :
         s_disc_fad ? "[xg] 3/5 disc image open (GD-ROM)" : "[xg] 3/5 disc image open");
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
