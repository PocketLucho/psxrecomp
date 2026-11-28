#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
uint8_t g_dc_scratch[1024];
static void die(const char* k, uint32_t a){ fprintf(stderr,"MMIO %s %08x (not modelled)\n",k,a); exit(5); }
uint32_t dc_lw_slow(uint32_t a){ die("lw",a); return 0; }
uint32_t dc_lh_slow(uint32_t a){ die("lh",a); return 0; }
uint32_t dc_lb_slow(uint32_t a){ die("lb",a); return 0; }
void dc_sw_slow(uint32_t a, uint32_t v){ (void)v; die("sw",a); }
void dc_sh_slow(uint32_t a, uint32_t v){ (void)v; die("sh",a); }
void dc_sb_slow(uint32_t a, uint32_t v){ (void)v; die("sb",a); }
int dc_irq_slow(void* cpu, uint32_t pc){ (void)cpu; (void)pc; return 0; }
