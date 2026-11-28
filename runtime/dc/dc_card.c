/* dc_card.c - PS1 memory card through the BIOS file API.
 *
 * Games reach the card through the kernel: _card_info/_card_load to probe it
 * and the "bu00:" device of open/read/write/close/firstfile/nextfile/erase.
 * The card in slot 1 is kept here as a small file system in RAM (up to 15
 * files of whole 8 KB blocks, as on a real card); slot 2 is empty. Each file
 * is stored by the host on its own (a VMU file on Dreamcast): loaded at the
 * first card access and written back when a file written to is closed.
 * Every operation completes at once; asynchronous ones (open mode 0x8000)
 * signal their end through the SwCARD "I/O end" event, as the BIOS does. */
#include "dc_runtime.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define V0 cpu->gpr[2]
#define A0 cpu->gpr[4]
#define A1 cpu->gpr[5]
#define A2 cpu->gpr[6]

#define CARD_FILES 15
#define CARD_BLOCK 8192u

typedef struct { char name[21]; uint32_t size; uint8_t* data; } CardFile;
typedef struct { int file; uint32_t pos; uint32_t mode; int dirty; } CardFd;

static CardFile s_file[CARD_FILES];
static CardFd s_fd[8];                  /* fds 2..9; 0/1 are the tty */
static int s_loaded;
static uint32_t s_errno;
static char s_pat[21];
static int s_next;

void dc_bios_event(CPUState* cpu, uint32_t cls, uint32_t spec);

/* ---- host storage: none by default (the card starts empty each run) ---- */
__attribute__((weak)) int dc_host_card_load(void (*add)(const char* name, const uint8_t* data, uint32_t size))
{ (void)add; return 0; }
__attribute__((weak)) int dc_host_card_save(const char* name, const uint8_t* data, uint32_t size)
{ (void)name; (void)data; (void)size; return 0; }
__attribute__((weak)) int dc_host_card_erase(const char* name) { (void)name; return 0; }

static void card_log(const char* f, const char* name, uint32_t a, uint32_t b)
{
#ifdef DC_LOG
    char t[96];
    snprintf(t, sizeof t, "[card] %s %s %X %X\n", f, name ? name : "", (unsigned)a, (unsigned)b);
    dc_host_log(t);
#else
    (void)f; (void)name; (void)a; (void)b;
#endif
}

static int find(const char* name)
{
    for (int i = 0; i < CARD_FILES; i++) if (s_file[i].data && !strcmp(s_file[i].name, name)) return i;
    return -1;
}

static int add_file(const char* name, uint32_t size)
{
    int used = 0;
    for (int i = 0; i < CARD_FILES; i++) if (s_file[i].data) used += (int)(s_file[i].size / CARD_BLOCK);
    if (used + (int)(size / CARD_BLOCK) > CARD_FILES) return -1;
    for (int i = 0; i < CARD_FILES; i++) if (!s_file[i].data) {
        uint8_t* d = (uint8_t*)calloc(1, size);
        if (!d) return -1;
        snprintf(s_file[i].name, sizeof s_file[i].name, "%s", name);
        s_file[i].size = size; s_file[i].data = d;
        return i;
    }
    return -1;
}

static void on_host_file(const char* name, const uint8_t* data, uint32_t size)
{
    size &= ~(CARD_BLOCK - 1u);
    if (!size || find(name) >= 0) return;
    int i = add_file(name, size);
    if (i >= 0) memcpy(s_file[i].data, data, size);
    card_log("load", name, size, (uint32_t)i);
}

static void load(void)
{
    if (s_loaded) return;
    s_loaded = 1;
    dc_host_card_load(on_host_file);
}

static void guest_str(uint32_t a, char* out, int max)
{
    int i = 0;
    for (; i < max - 1; i++) { char c = (char)dc_read8(a + (uint32_t)i); if (!c) break; out[i] = c; }
    out[i] = 0;
}

/* "bu00:NAME" -> port (0 or 1) and the name; -1 for any other device. */
static int parse(uint32_t a, char name[21])
{
    char p[40];
    guest_str(a, p, sizeof p);
    if (strncmp(p, "bu", 2) || !p[2] || !p[3] || p[4] != ':') return -1;
    snprintf(name, 21, "%s", p + 5);
    return p[2] == '1' ? 1 : 0;
}

static int match(const char* pat, const char* s)
{
    if (!*pat) return 1;                                /* "bu00:" lists every file */
    for (;; pat++, s++) {
        if (*pat == '*') return 1;
        if (!*pat || !*s) return *pat == *s;
        if (*pat != '?' && *pat != *s) return 0;
    }
}

/* struct DIRENTRY { char name[20]; long attr, size; DIRENTRY* next; long head; char system[4]; } */
static uint32_t dirent(uint32_t d, int from)
{
    for (int i = from; i < CARD_FILES; i++) {
        if (!s_file[i].data || !match(s_pat, s_file[i].name)) continue;
        for (uint32_t k = 0; k < 20; k++) dc_write8(d + k, (uint8_t)s_file[i].name[k]);
        dc_write32(d + 20, 0x50u);
        dc_write32(d + 24, s_file[i].size);
        dc_write32(d + 28, 0);
        dc_write32(d + 32, (uint32_t)i + 1u);
        dc_write32(d + 36, 0);
        s_next = i + 1;
        return d;
    }
    s_next = CARD_FILES;
    return 0;
}

static CardFd* fd_get(uint32_t fd) { return fd >= 2 && fd < 10 && s_fd[fd - 2].file >= 0 ? &s_fd[fd - 2] : 0; }

static void io_end(CPUState* cpu, uint32_t mode)
{
    if (mode & 0x8000u) { dc_bios_event(cpu, 0xF4000001u, 0x0004u); dc_bios_event(cpu, 0xF0000011u, 0x0004u); }
}

static void flush(CardFd* f)
{
    if (!f->dirty) return;
    CardFile* c = &s_file[f->file];
    int r = dc_host_card_save(c->name, c->data, c->size);
    card_log("save", c->name, c->size, (uint32_t)r);
    f->dirty = 0;
}

/* B0 functions; returns 0 when fn is not a card/file call. */
int dc_card_b0(CPUState* cpu, uint32_t fn)
{
    char name[21];
    static int init;
    if (!init) { init = 1; for (int i = 0; i < 8; i++) s_fd[i].file = -1; }
    switch (fn) {
    case 0x32: {                                        /* open(name, mode) */
        load();
        uint32_t mode = A1;
        int port = parse(A0, name), i = -1;
        V0 = 0xFFFFFFFFu;
        if (port != 0) { s_errno = 2; card_log("open-fail", name, mode, (uint32_t)port); break; }
        i = find(name);
        if (mode & 0x200u) {
            if (i >= 0) { s_errno = 17; card_log("open-exists", name, mode, 0); break; }
            uint32_t blocks = mode >> 16 ? mode >> 16 : 1u;
            i = add_file(name, blocks * CARD_BLOCK);
            if (i < 0) { s_errno = 28; card_log("open-full", name, mode, 0); break; }
        } else if (i < 0) { s_errno = 2; card_log("open-none", name, mode, 0); break; }
        for (int k = 0; k < 8; k++) if (s_fd[k].file < 0) {
            s_fd[k] = (CardFd){i, 0, mode, (mode & 0x200u) != 0};   /* a new file is stored at close */
            V0 = (uint32_t)k + 2u;
            break;
        }
        card_log("open", name, mode, V0);
        break;
    }
    case 0x33: {                                        /* lseek(fd, off, whence) */
        CardFd* f = fd_get(A0);
        if (!f) { V0 = 0xFFFFFFFFu; break; }
        f->pos = A2 == 1 ? f->pos + A1 : A1;
        V0 = f->pos;
        break;
    }
    case 0x34: case 0x35: {                             /* read/write(fd, buf, len) */
        CardFd* f = fd_get(A0);
        if (!f) return 0;                               /* tty write: the BIOS handles it */
        CardFile* c = &s_file[f->file];
        uint32_t n = A2;
        if (f->pos >= c->size) n = 0;
        else if (n > c->size - f->pos) n = c->size - f->pos;
        if (fn == 0x34) for (uint32_t k = 0; k < n; k++) dc_write8(A1 + k, c->data[f->pos + k]);
        else { for (uint32_t k = 0; k < n; k++) c->data[f->pos + k] = dc_read8(A1 + k); if (n) f->dirty = 1; }
        card_log(fn == 0x34 ? "read" : "write", c->name, f->pos, n);
        f->pos += n;
        V0 = n;
        io_end(cpu, f->mode);
        break;
    }
    case 0x36: {                                        /* close(fd) */
        CardFd* f = fd_get(A0);
        if (!f) { V0 = A0; break; }
        flush(f);
        f->file = -1;
        V0 = A0;
        break;
    }
    case 0x41: V0 = 1; break;                           /* format */
    case 0x42:                                          /* firstfile(name, dirent) */
        load();
        if (parse(A0, s_pat) != 0) { V0 = 0; break; }
        V0 = dirent(A1, 0);
        card_log("firstfile", s_pat, V0, 0);
        break;
    case 0x43: V0 = dirent(A0, s_next); break;          /* nextfile(dirent) */
    case 0x44: {                                        /* rename(old, new) */
        char nn[21];
        load();
        int i = parse(A0, name) == 0 ? find(name) : -1;
        V0 = 0;
        if (i < 0 || parse(A1, nn) != 0 || find(nn) >= 0) break;
        dc_host_card_erase(name);
        snprintf(s_file[i].name, sizeof s_file[i].name, "%s", nn);
        dc_host_card_save(nn, s_file[i].data, s_file[i].size);
        V0 = 1;
        break;
    }
    case 0x45: {                                        /* erase(name) */
        load();
        int i = parse(A0, name) == 0 ? find(name) : -1;
        V0 = 0;
        if (i < 0) break;
        for (int k = 0; k < 8; k++) if (s_fd[k].file == i) s_fd[k].file = -1;
        free(s_file[i].data); s_file[i].data = 0;
        dc_host_card_erase(name);
        card_log("erase", name, 0, 0);
        V0 = 1;
        break;
    }
    case 0x4A: case 0x4B: case 0x4C: V0 = 1; break;     /* InitCARD/StartCARD/StopCARD */
    case 0x4D: case 0x4E: case 0x4F:                    /* _card_info_subfunc/_card_write/_card_read */
        V0 = 1; dc_bios_event(cpu, 0xF4000001u, A0 & 0x10u ? 0x0100u : 0x0004u); break;
    case 0x50: break;                                   /* _new_card */
    case 0x54: V0 = s_errno; break;                     /* _get_errno */
    case 0x55: V0 = s_errno; break;                     /* _get_error(fd) */
    case 0x5C: V0 = (A0 & 0x10u) ? 0 : 1; break;        /* _card_status(slot) */
    case 0x5D: V0 = 1; break;                           /* _card_wait(slot) */
    default: return 0;
    }
    return 1;
}

/* A0 functions; returns 0 when fn is not a card call. */
int dc_card_a0(CPUState* cpu, uint32_t fn)
{
    switch (fn) {
    case 0xAB: case 0xAC: {                             /* _card_info(port) / _card_load(port) */
        load();
        uint32_t spec = (A0 & 0x10u) ? 0x0100u : 0x0004u;   /* slot 2: no card (time out) */
        dc_bios_event(cpu, 0xF4000001u, spec);
        dc_bios_event(cpu, 0xF0000011u, spec);
        V0 = 1;
        break;
    }
    case 0xAD: case 0xAE: case 0xAF: V0 = 1; break;     /* _card_auto, bufs_cd, _card_clear... */
    default: return 0;
    }
    return 1;
}
