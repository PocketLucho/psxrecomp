/* dc_gpu.h - PS1 GPU (GP0/GP1, VRAM) for the Dreamcast runtime. */
#pragma once
#include "dc_runtime.h"

extern uint16_t g_dc_vram[1024 * 512];   /* 15-bit VRAM, 1024x512 */

void     dc_gpu_reset(void);
void     dc_gpu_gp0(uint32_t w);
void     dc_gpu_gp1(uint32_t w);
uint32_t dc_gpu_read(void);
uint32_t dc_gpu_stat(void);

typedef struct { int x, y, w, h, depth24, enabled; } DcGpuDisplay;
void dc_gpu_display(DcGpuDisplay* d);    /* current display area */

/* Hooks for a hardware renderer (the PowerVR on Dreamcast). The defaults are
 * weak no-ops, so without one everything is drawn in software into VRAM. */
typedef struct { int x, y, r, g, b, u, v; } DcPrimVtx;
typedef struct {
    int n;                         /* 3 or 4 vertices, in strip order */
    int tex, raw, semi, semi_mode, gouraud, depth;
    uint32_t tp_x, tp_y, clut_x, clut_y;
    int clip_x0, clip_y0, clip_x1, clip_y1;
    int tw_w, tw_h, tw_u0, tw_v0;  /* texture window: repeat this w x h texel area (256: none) */
    DcPrimVtx v[4];
} DcPrim;
int  dc_gpu_hw_prim(const DcPrim* p);   /* 1: drawn by the host; 2: also draw it into VRAM */
void dc_gpu_hw_fill(int x, int y, int w, int h, uint16_t c);
void dc_gpu_hw_flip(void);                            /* display start changed */
/* VRAM rectangle written by a copy or an upload; clip is the drawing area */
void dc_gpu_hw_blit(int x, int y, int w, int h, int cx0, int cy0, int cx1, int cy1);
void dc_gpu_hw_sync(int x, int y, int w, int h);   /* VRAM area about to be read: bring it up to date */

/* VRAM write generations per 64x16 block, for texture caches. */
extern uint32_t g_dc_vram_gen[32][16];
