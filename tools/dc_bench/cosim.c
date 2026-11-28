#include <stdint.h>
#include <stdio.h>
unsigned long long g_mips_insns, g_mips_blocks;
void cosim_instr(uint32_t pc){ (void)pc; g_mips_insns++; }
void cosim_block(uint32_t pc){ (void)pc; g_mips_blocks++; }
__attribute__((destructor)) static void rep(void){ fprintf(stderr,"mips_insns=%llu blocks=%llu\n",g_mips_insns,g_mips_blocks); }
