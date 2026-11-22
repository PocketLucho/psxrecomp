/* dc_host_linux.c - test platform layer: runs the DC runtime as a Linux SH-4
 * program (qemu-sh4), with a 60 Hz VBlank from a host thread. Not for hardware. */
#include "dc_runtime.h"
#include "dc_overlay.h"
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <ucontext.h>

int dc_boot_exe(const void* exe, size_t size);

void dc_host_log(const char* s) { fputs(s, stdout); fflush(stdout); }

uint64_t dc_host_ticks_us(void)
{
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
}

int dc_host_read_file(const char* path, void** data, size_t* size)
{
    FILE* f = fopen(path, "rb"); if (!f) return -1;
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    void* p = malloc((size_t)n);
    if (!p || fread(p, 1, (size_t)n, f) != (size_t)n) { fclose(f); free(p); return -1; }
    fclose(f); *data = p; *size = (size_t)n; return 0;
}

static FILE* s_disc;

int dc_host_cd_read_sector(uint32_t lba, uint8_t* out)
{
    if (!s_disc) return -1;
    if (fseek(s_disc, (long)lba * 2352L, SEEK_SET) != 0) { perror("[host] disc seek"); return -1; }
    size_t n = fread(out, 1, 2352, s_disc);
    if (n != 2352) { fprintf(stdout, "[host] disc read lba %u: got %zu bytes\n", (unsigned)lba, n); return -1; }
    return 0;
}

/* VBlank divider: 10 = 60 Hz. DC_VBLANK_DIV=1 gives 600 Hz, which keeps
 * PSY-Q's iteration-count timeouts from firing when qemu runs the guest much
 * faster than a PS1. */
static int s_vblank_div = 10;

/* 600 Hz tick on a host thread: every tick lets timed devices advance, every
 * s_vblank_div-th one counts a VBlank. A thread rather than SIGALRM because
 * qemu-sh4 can corrupt guest registers when a signal lands next to an SH-4
 * delayed branch, which showed up as random crashes. */
static volatile unsigned s_vblanks;

unsigned dc_host_take_vblanks(void) { return __atomic_exchange_n(&s_vblanks, 0u, __ATOMIC_ACQ_REL); }

static void* tick_thread(void* arg)
{
    (void)arg;
    struct timespec next; clock_gettime(CLOCK_MONOTONIC, &next);
    for (int n = 0;;) {
        next.tv_nsec += 1668000;
        if (next.tv_nsec >= 1000000000) { next.tv_nsec -= 1000000000; next.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, 0);
        dc_profile_sample();
        if (++n >= s_vblank_div) { n = 0; __atomic_add_fetch(&s_vblanks, 1u, __ATOMIC_ACQ_REL); }
        g_irq_pending = 1;
    }
    return 0;
}

void dc_host_start_vblank_timer(void)
{
    pthread_t t;
    sigset_t all, old; sigfillset(&all);
    pthread_sigmask(SIG_BLOCK, &all, &old);      /* signals stay on the guest thread */
    pthread_create(&t, 0, tick_thread, 0);
    pthread_sigmask(SIG_SETMASK, &old, 0);
}

static void on_timeout(int sig) { (void)sig; dc_fatal("time limit reached"); }
static void on_segv(int sig, siginfo_t* si, void* uc)
{
    (void)sig; int here;
#if defined(__sh__)
    const mcontext_t* m = &((const ucontext_t*)uc)->uc_mcontext;
    dc_fatal("host crash (signal %d) at %p, host pc=%08X pr=%08X r15=%08X, stack near %p", sig, si->si_addr,
             m->pc, m->pr, m->gregs[15], (void*)&here);
#else
    (void)uc;
    dc_fatal("host crash (signal %d) at %p, host stack near %p", sig, si->si_addr, (void*)&here);
#endif
}

int main(int argc, char** argv)
{
    { int here; fprintf(stdout, "[host] main stack %p\n", (void*)&here); }
    if (argc < 2) { fprintf(stderr, "usage: %s <PS-X EXE> [seconds] [disc.bin]\n", argv[0]); return 2; }
    void* exe; size_t n;
    if (dc_host_read_file(argv[1], &exe, &n)) { perror(argv[1]); return 1; }
    static char altstack[65536];
    stack_t ss = {.ss_sp = altstack, .ss_size = sizeof altstack};
    sigaltstack(&ss, 0);
    struct sigaction sa; memset(&sa, 0, sizeof sa);
    sa.sa_sigaction = on_segv; sa.sa_flags = SA_ONSTACK | SA_SIGINFO;
    sigaction(SIGSEGV, &sa, 0); sigaction(SIGBUS, &sa, 0); sigaction(SIGILL, &sa, 0);
    if (getenv("DC_VBLANK_DIV")) s_vblank_div = atoi(getenv("DC_VBLANK_DIV")) > 0 ? atoi(getenv("DC_VBLANK_DIV")) : 10;
    /* DC_SKIP=overlay-id@hexaddr[,...]: return at once from those functions */
    if (getenv("DC_SKIP")) dc_overlay_skip_parse(getenv("DC_SKIP"));
    if (argc > 3 && !(s_disc = fopen(argv[3], "rb"))) { perror(argv[3]); return 1; }
    if (argc > 2) {
        signal(SIGVTALRM, on_timeout);
        struct itimerval it = {{0, 0}, {atoi(argv[2]), 0}};
        setitimer(ITIMER_VIRTUAL, &it, 0);
    }
    return dc_boot_exe(exe, n);
}

/* Report where a stack-protector check failed instead of libc's bare abort. */
void __stack_chk_fail(void)
{
    char b[80];
    snprintf(b, sizeof b, "stack smashing in host code called from %p", __builtin_return_address(0));
    dc_fatal("%s", b);
}
