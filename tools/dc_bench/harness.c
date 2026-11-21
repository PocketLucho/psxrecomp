/* Minimal SH-4 harness: runs recompiled bench_main(which, iters) on a flat 2 MB RAM. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef struct CPUState { uint32_t gpr[32]; uint32_t pc, hi, lo; uint32_t cop0[32], gte_data[32], gte_ctrl[32];
  uint32_t (*read_word)(uint32_t); void (*write_word)(uint32_t,uint32_t); uint16_t (*read_half)(uint32_t);
  void (*write_half)(uint32_t,uint16_t); uint8_t (*read_byte)(uint32_t); void (*write_byte)(uint32_t,uint8_t);
  uint8_t pad[256]; } CPUState;

uint8_t g_dc_ram[0x200000] __attribute__((aligned(32)));
#define ram_buf g_dc_ram
uint8_t *g_psx_ram = ram_buf;
uint32_t g_psx_ram_mask = 0x1FFFFF;
uint32_t g_debug_last_store_pc, g_psx_resume_seed, g_psx_last_fn_entry;
int g_ds_recording, g_ls_mode, g_ls_replay_active, g_event_step_conservative;
int g_gte_native_provenance_active, g_psx_load_delay, g_psx_precise_slice;
uint32_t g_psx_cyc_batch, g_psx_cyc_batch_limit=0xFFFFFFFFu; int g_psx_cyc_bb_defer; static uint32_t acc_; uint32_t *g_psx_cyc_local_acc=&acc_;
uint64_t psx_cycle_count, psx_next_service_cycle=~0ull;
volatile int g_irq_pending;

static inline uint32_t phys(uint32_t a){ return a & 0x1FFFFF; }
uint32_t psx_read_word(uint32_t a){ uint32_t v; memcpy(&v, ram_buf+(phys(a)&~3u), 4); return v; }
uint16_t psx_read_half(uint32_t a){ uint16_t v; memcpy(&v, ram_buf+(phys(a)&~1u), 2); return v; }
uint8_t  psx_read_byte(uint32_t a){ return ram_buf[phys(a)]; }
static void w32(uint32_t a, uint32_t v){ memcpy(ram_buf+(phys(a)&~3u), &v, 4); }
static void w16(uint32_t a, uint16_t v){ memcpy(ram_buf+(phys(a)&~1u), &v, 2); }
static void w8 (uint32_t a, uint8_t v){ ram_buf[phys(a)] = v; }

/* Cheap stubs for faithful-timing / tooling hooks (lower bound of their cost). */
void gte_native_provenance_cpu_alu(){} void gte_native_provenance_cpu_load(){} void gte_native_provenance_cpu_store(){}
void ram_provenance_note_cpu_store(uint32_t i, uint32_t a, uint32_t v){(void)i;(void)a;(void)v;}
void psx_advance_cycles_slow(){} void psx_devices_service_to_now(){} int psx_in_device_service(){return 0;}
int psx_load_delay_enabled(void){ g_psx_load_delay = 0; return 0; }
void psx_icache_fetch(){} int psx_slice_block_impl(){ return 0; }
void psx_muldiv_set(){} void psx_muldiv_stall(){} uint32_t psx_mult_latency_s(){ return 0; }
uint32_t psx_cyc_load_word_slow(CPUState*c,uint32_t a,uint32_t r,uint32_t m){(void)c;(void)r;(void)m;return psx_read_word(a);}
uint16_t psx_cyc_load_half_slow(CPUState*c,uint32_t a,uint32_t r,uint32_t m){(void)c;(void)r;(void)m;return psx_read_half(a);}
uint8_t psx_cyc_load_byte(CPUState*c,uint32_t a,uint32_t r,uint32_t m){(void)c;(void)r;(void)m;return psx_read_byte(a);}
int psx_check_interrupts_at(CPUState*c, uint32_t pc){ (void)c;(void)pc; return g_irq_pending; }
void psx_unknown_dispatch(CPUState*c, uint32_t a, uint32_t p){ fprintf(stderr,"unknown dispatch %08x\n",a); exit(3); }
int __ctzsi2(unsigned x){ int n=0; if(!x) return 32; while(!(x&1u)){x>>=1;n++;} return n; }

extern void func_800106C8(CPUState*);
extern void psx_dispatch(CPUState*, uint32_t);

int main(int argc, char** argv){
  int which = argc>1 ? atoi(argv[1]) : 9, iters = argc>2 ? atoi(argv[2]) : 1;
  FILE* f = fopen(argc>3?argv[3]:"KERN.EXE","rb"); if(!f){perror("exe");return 1;}
  static uint8_t buf[0x10000]; size_t n = fread(buf,1,sizeof buf,f); fclose(f);
  memcpy(ram_buf+0x10000, buf+2048, n-2048);
  static CPUState cpu; memset(&cpu,0,sizeof cpu);
  cpu.write_word=w32; cpu.write_half=w16; cpu.write_byte=w8;
  cpu.read_word=psx_read_word; cpu.read_half=psx_read_half; cpu.read_byte=psx_read_byte;
  cpu.gpr[4]=which; cpu.gpr[5]=iters; cpu.gpr[29]=0x801FFF00u; cpu.gpr[31]=0x80000000u;
  func_800106C8(&cpu);
  /* follow tail-published continuations until we reach the magic return address */
  int guard=0; while (cpu.pc && cpu.pc!=0x80000000u && guard++<1000000){ uint32_t pc=cpu.pc; cpu.pc=0; psx_dispatch(&cpu,pc); }
  printf("result=%d pc=%08x\n",(int)cpu.gpr[2],cpu.pc);
  return 0;
}
/* dispatch-side stubs */
int g_psx_cps_mode;
int dirty_ram_text_native_ok_ranges(const uint32_t* p, uint32_t n){ (void)p;(void)n; return 1; }
int dirty_ram_text_native_ok_ranges_from(const uint32_t* p, uint32_t n, uint32_t pc){ (void)p;(void)n;(void)pc; return 1; }
int psx_check_interrupts_dispatch_entry(CPUState* c, uint32_t pc){ (void)c;(void)pc; return g_irq_pending; }
int psx_game_identity_bind_static(const void* id){ (void)id; return 1; }
int psx_game_identity_gate(const void* id){ (void)id; return 1; }
int psx_vsync_query_hle_try(CPUState* c, uint32_t a){ (void)c;(void)a; return 0; }
extern int psx_dispatch_game_compiled(CPUState*, uint32_t);
void psx_dispatch(CPUState* c, uint32_t a){ if(!psx_dispatch_game_compiled(c,a)){ fprintf(stderr,"no entry %08x\n",a); exit(4);} }
