/* dc_overlay.c - pick the loaded overlay for an address (see dc_overlay.h). */
#include "dc_overlay.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Weak references: a build without overlays links no registry. */
extern const DcOverlay g_dc_overlays[] __attribute__((weak));
extern const int g_dc_overlay_count __attribute__((weak));
static int count(void) { return &g_dc_overlay_count ? g_dc_overlay_count : 0; }

extern const DcOvSlot g_dc_ov_slots[] __attribute__((weak));
extern const int g_dc_ov_slot_count __attribute__((weak));

static const DcOverlay* s_last;      /* most recent match, checked first */
static const DcOverlay* s_in_slot[16];                       /* overlay loaded in each slot */
static int (*s_slot_fn[16])(CPUState*, uint32_t);            /* its dispatcher */

/* Make sure a loadable overlay's code is in its slot; returns its dispatcher. */
static int (*load(const DcOverlay* o))(CPUState*, uint32_t)
{
    if (o->dispatch) return o->dispatch;
    int s = o->slot;
    if (!&g_dc_ov_slot_count || s < 0 || s >= g_dc_ov_slot_count || s >= 16)
        dc_fatal("overlay %s: no code slot %d", o->id, s);
    if (s_in_slot[s] == o) return s_slot_fn[s];
    const DcOvSlot* sl = &g_dc_ov_slots[s];
    s_in_slot[s] = 0;
    int n = dc_host_load_overlay(o->file, sl->base, sl->size);
    const DcOvHeader* h = (const DcOvHeader*)sl->base;
    uint32_t base = (uint32_t)(uintptr_t)sl->base;
    if (n < (int)sizeof *h || h->magic != DC_OV_MAGIC || h->end < base + (uint32_t)n || h->end > base + sl->size)
        dc_fatal("overlay %s: cannot load %s into slot %d (%d bytes)", o->id, o->file, s, n);
    memset(sl->base + n, 0, h->end - base - (uint32_t)n);
    dc_host_code_written(sl->base, h->end - base);
    s_slot_fn[s] = (int (*)(CPUState*, uint32_t))(uintptr_t)h->dispatch;
    s_in_slot[s] = o;
    char b[128];
    snprintf(b, sizeof b, "[overlay] loaded %s (%d bytes) into slot %d\n", o->file, n, s);
    dc_host_log(b);
    return s_slot_fn[s];
}
static const DcOverlay* s_logged[8]; /* overlays already reported as resident */

static struct { char id[48]; uint32_t addr; DcSkipFn fn; int hle; } s_skip[8];
static int s_nskip;
int g_dc_hle_off;   /* run the guest code of HLE'd functions (to check them) */

void dc_overlay_skip_fn(const char* id, uint32_t addr, DcSkipFn fn)
{
    if (s_nskip >= 8) return;
    snprintf(s_skip[s_nskip].id, sizeof s_skip[0].id, "%s", id);
    s_skip[s_nskip].fn = fn;
    s_skip[s_nskip++].addr = dc_phys(addr);
}

void dc_overlay_skip(const char* id, uint32_t addr) { dc_overlay_skip_fn(id, addr, 0); }
void dc_overlay_hle(const char* id, uint32_t addr, DcSkipFn fn)
{
    if (s_nskip >= 8) return;
    dc_overlay_skip_fn(id, addr, fn);
    s_skip[s_nskip - 1].hle = 1;
}

void dc_overlay_skip_parse(const char* spec)
{
    while (spec && *spec) {
        const char* at = strchr(spec, '@');
        if (!at) break;
        char id[48]; size_t n = (size_t)(at - spec);
        if (n >= sizeof id) n = sizeof id - 1;
        memcpy(id, spec, n); id[n] = 0;
        char* end; uint32_t a = (uint32_t)strtoul(at + 1, &end, 16);
        dc_overlay_skip(id, a);
        char b[96]; snprintf(b, sizeof b, "[overlay] will skip %s at %08X\n", id, (unsigned)a);
        dc_host_log(b);
        spec = *end == ',' ? end + 1 : end;
    }
}

static int skipped(CPUState* cpu, const DcOverlay* o, uint32_t addr)
{
    for (int i = 0; i < s_nskip; i++) {
        if (s_skip[i].addr != dc_phys(addr) || strcmp(s_skip[i].id, o->id)) continue;
        if (s_skip[i].hle && g_dc_hle_off) continue;
        if (s_skip[i].hle) { s_skip[i].fn(cpu); cpu->pc = cpu->gpr[31]; return 1; }
        char b[96]; snprintf(b, sizeof b, "[overlay] skipped %s at %08X\n", o->id, (unsigned)addr);
        dc_host_log(b);
        if (s_skip[i].fn) s_skip[i].fn(cpu);
        cpu->gpr[2] = 0;
        cpu->pc = cpu->gpr[31];
        return 1;
    }
    return 0;
}

/* Runs at every call into overlay code: sample words are code words, so
   aligned, and read directly (a memcpy call per word cost ~5% of a frame). */
static int resident(const DcOverlay* o)
{
    for (int i = 0; i < o->nsamples; i++) {
        uint32_t a = o->samples[i].addr, w;
        if (__builtin_expect(!(a & 3), 1)) w = *(const uint32_t*)(const void*)dc_ram_ptr(a);
        else memcpy(&w, dc_ram_ptr(a), 4);
        if (w != o->samples[i].word) return 0;
    }
    return 1;
}

static int covers(const DcOverlay* o, uint32_t a)
{
    return dc_phys(a) - dc_phys(o->load) < o->size;
}

/* The overlay that ran last is checked at three of its sample words only:
   one loaded over it at the same place differs there too. */
static int still_resident(const DcOverlay* o)
{
    int n = o->nsamples;
    if (n < 4) return resident(o);
    const int k[3] = { 0, n / 2, n - 1 };
    for (int i = 0; i < 3; i++) {
        uint32_t a = o->samples[k[i]].addr, w;
        if (a & 3) return resident(o);
        w = *(const uint32_t*)(const void*)dc_ram_ptr(a);
        if (w != o->samples[k[i]].word) return 0;
    }
    return 1;
}

int dc_overlay_dispatch(CPUState* cpu, uint32_t addr)
{
    if (s_last && covers(s_last, addr) && still_resident(s_last)) {
        if (skipped(cpu, s_last, addr) || load(s_last)(cpu, addr)) return 1;
    }
    for (int i = 0; i < count(); i++) {
        const DcOverlay* o = &g_dc_overlays[i];
        if (o == s_last || !covers(o, addr) || !resident(o)) continue;
        if (skipped(cpu, o, addr)) return 1;
        if (!load(o)(cpu, addr)) continue;
        int seen = 0, k;
        for (k = 0; k < 8; k++) if (s_logged[k] == o) seen = 1;
        if (!seen) {
            /* forget overlays this one replaced in RAM, so a later reload is reported */
            for (k = 0; k < 8; k++)
                if (s_logged[k] && covers(s_logged[k], o->load)) s_logged[k] = 0;
            for (k = 0; k < 8; k++) if (!s_logged[k]) { s_logged[k] = o; break; }
            char b[96];
            snprintf(b, sizeof b, "[overlay] %s at %08X\n", o->id, (unsigned)addr);
            dc_host_log(b);
        }
        s_last = o;
        return 1;
    }
    return 0;
}

void dc_overlay_explain(uint32_t addr)
{
    char b[128];
    for (int i = 0; i < count(); i++) {
        const DcOverlay* o = &g_dc_overlays[i];
        if (!covers(o, addr)) continue;
        int hit = 0;
        for (int k = 0; k < o->nsamples; k++) {
            uint32_t w;
            memcpy(&w, dc_ram_ptr(o->samples[k].addr), 4);
            hit += w == o->samples[k].word;
        }
        snprintf(b, sizeof b, "[overlay] %s covers %08X, %d/%d sample words in RAM\n", o->id,
                 (unsigned)addr, hit, o->nsamples);
        dc_host_log(b);
    }
}
