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
