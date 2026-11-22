/* dc_overlay.c - pick the loaded overlay for an address (see dc_overlay.h). */
#include "dc_overlay.h"
#include <stdio.h>
#include <string.h>

/* Weak references: a build without overlays links no registry. */
extern const DcOverlay g_dc_overlays[] __attribute__((weak));
extern const int g_dc_overlay_count __attribute__((weak));
static int count(void) { return &g_dc_overlay_count ? g_dc_overlay_count : 0; }

static const DcOverlay* s_last;      /* most recent match, checked first */
static const DcOverlay* s_logged[8]; /* overlays already reported as resident */

static int resident(const DcOverlay* o)
{
    for (int i = 0; i < o->nsamples; i++) {
        uint32_t w;
        memcpy(&w, dc_ram_ptr(o->samples[i].addr), 4);
        if (w != o->samples[i].word) return 0;
    }
    return 1;
}

static int covers(const DcOverlay* o, uint32_t a)
{
    return dc_phys(a) - dc_phys(o->load) < o->size;
}

int dc_overlay_dispatch(CPUState* cpu, uint32_t addr)
{
    if (s_last && covers(s_last, addr) && resident(s_last) && s_last->dispatch(cpu, addr)) return 1;
    for (int i = 0; i < count(); i++) {
        const DcOverlay* o = &g_dc_overlays[i];
        if (o == s_last || !covers(o, addr) || !resident(o)) continue;
        if (!o->dispatch(cpu, addr)) continue;
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
