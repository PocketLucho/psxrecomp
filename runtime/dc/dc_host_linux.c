/* dc_host_linux.c - test platform layer: runs the DC runtime as a Linux SH-4
 * program (qemu-sh4), with a 60 Hz VBlank from SIGALRM. Not for hardware. */
#include "dc_runtime.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

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

static void on_alarm(int sig) { (void)sig; dc_irq_raise(1u); }

void dc_host_start_vblank_timer(void)
{
    signal(SIGALRM, on_alarm);
    struct itimerval it = {{0, 16683}, {0, 16683}};
    setitimer(ITIMER_REAL, &it, 0);
}

static void on_timeout(int sig) { (void)sig; dc_fatal("time limit reached"); }
static void on_segv(int sig, siginfo_t* si, void* uc)
{
    (void)sig; (void)uc; int here;
    dc_fatal("host crash (SIGSEGV) at %p, host stack near %p", si->si_addr, (void*)&here);
}

int main(int argc, char** argv)
{
    { int here; fprintf(stdout, "[host] main stack %p\n", (void*)&here); }
    if (argc < 2) { fprintf(stderr, "usage: %s <PS-X EXE> [seconds]\n", argv[0]); return 2; }
    void* exe; size_t n;
    if (dc_host_read_file(argv[1], &exe, &n)) { perror(argv[1]); return 1; }
    static char altstack[65536];
    stack_t ss = {.ss_sp = altstack, .ss_size = sizeof altstack};
    sigaltstack(&ss, 0);
    struct sigaction sa; memset(&sa, 0, sizeof sa);
    sa.sa_sigaction = on_segv; sa.sa_flags = SA_ONSTACK | SA_SIGINFO;
    sigaction(SIGSEGV, &sa, 0);
    if (argc > 2) {
        signal(SIGVTALRM, on_timeout);
        struct itimerval it = {{0, 0}, {atoi(argv[2]), 0}};
        setitimer(ITIMER_VIRTUAL, &it, 0);
    }
    return dc_boot_exe(exe, n);
}
