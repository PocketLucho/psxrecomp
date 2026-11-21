/* Synthetic PS1-style workloads (own code, not game code). */
typedef unsigned int u32; typedef int s32; typedef unsigned short u16; typedef short s16; typedef unsigned char u8;

#define NV 256
static s16 verts[NV*3];
static s32 outv[NV*3];
static s16 mat[9] = {4096,0,0, 0,3547,-2048, 0,2048,3547};
static u8 lzsrc[4096];
static u8 lzdst[8192];
static u8 script[1024];
static s32 vmvars[64];
static u32 ot[256];
static u32 prims[NV*8];
static u32 seed = 12345;

static u32 rnd(void){ seed = seed*1103515245u + 12345u; return seed>>16; }

__attribute__((noinline)) static void init(void){
  for (int i=0;i<NV*3;i++) verts[i] = (s16)((rnd()&0x3FF)-512);
  /* LZSS-ish stream: flag byte, then literals or (offset,len) pairs */
  int p=0;
  while (p < 4000){
    u8 f = (u8)rnd(); lzsrc[p++] = f;
    for (int b=0;b<8 && p<4000;b++){
      if (f & (1<<b)) lzsrc[p++] = (u8)rnd();
      else { lzsrc[p++] = (u8)rnd(); lzsrc[p++] = (u8)((rnd()&0xF0)|(rnd()&7)); }
    }
  }
  for (int i=0;i<1024;i++) script[i] = (u8)(rnd() % 12);
}

__attribute__((noinline)) static s32 transform(void){
  s32 acc=0;
  for (int i=0;i<NV;i++){
    s32 x=verts[i*3], y=verts[i*3+1], z=verts[i*3+2];
    s32 tx=(mat[0]*x+mat[1]*y+mat[2]*z)>>12;
    s32 ty=(mat[3]*x+mat[4]*y+mat[5]*z)>>12;
    s32 tz=((mat[6]*x+mat[7]*y+mat[8]*z)>>12)+2048;
    if (tz < 16) tz = 16;
    outv[i*3]=(tx*256)/tz; outv[i*3+1]=(ty*256)/tz; outv[i*3+2]=tz;
    acc += outv[i*3] ^ outv[i*3+1];
  }
  return acc;
}

__attribute__((noinline)) static s32 lzs(void){
  int sp=0, dp=0;
  while (sp < 4000 && dp < 8000){
    u8 f = lzsrc[sp++];
    for (int b=0;b<8 && sp<4000 && dp<8000;b++){
      if (f & 1) lzdst[dp++] = lzsrc[sp++];
      else {
        u32 lo=lzsrc[sp], hi=lzsrc[sp+1]; sp+=2;
        int off = (int)(((hi&0xF0)<<4)|lo); int len=(int)(hi&7)+3;
        int from = dp - 1 - (off % (dp+1));
        if (from < 0) from = 0;
        while (len-- && dp<8000) lzdst[dp++] = lzdst[from++];
      }
      f >>= 1;
    }
  }
  s32 h=0; for (int i=0;i<dp;i+=16) h = h*31 + lzdst[i];
  return h;
}

__attribute__((noinline)) static s32 vm(void){
  int pc=0; s32 a=0; int steps=0;
  while (steps++ < 4000){
    u8 op = script[pc & 1023];
    switch (op){
      case 0: a += vmvars[pc&63]; break;
      case 1: vmvars[(pc+1)&63] = a; break;
      case 2: a ^= (s32)pc; break;
      case 3: if (a & 1) pc += 3; break;
      case 4: a = a*3 + 1; break;
      case 5: a >>= 1; break;
      case 6: vmvars[a&63] += 1; break;
      case 7: pc -= (a & 7); if (pc<0) pc=0; break;
      case 8: a -= vmvars[(pc*7)&63]; break;
      case 9: a = (a<<2) | (a>>30); break;
      case 10: a += script[(pc+5)&1023]; break;
      default: a = ~a; break;
    }
    pc++;
  }
  return a;
}

__attribute__((noinline)) static s32 build_ot(void){
  for (int i=0;i<256;i++) ot[i] = 0x00FFFFFF;
  u32 *p = prims;
  for (int i=0;i<NV-2;i+=1){
    s32 z = outv[i*3+2] >> 4; if (z<0) z=0; if (z>255) z=255;
    p[0] = (4u<<24) | (ot[z] & 0x00FFFFFF);
    ot[z] = (u32)(p - prims);
    p[1] = 0x20808080;
    p[2] = ((u32)(outv[i*3+1]&0xFFFF)<<16) | (u32)(outv[i*3]&0xFFFF);
    p[3] = ((u32)(outv[i*3+4]&0xFFFF)<<16) | (u32)(outv[i*3+3]&0xFFFF);
    p[4] = ((u32)(outv[i*3+7]&0xFFFF)<<16) | (u32)(outv[i*3+6]&0xFFFF);
    p += 5;
  }
  s32 h=0; for (int i=0;i<256;i++) h += (s32)ot[i];
  return h;
}

s32 bench_main(int which, int iters){
  static int inited=0; if(!inited){init(); inited=1;}
  s32 r=0;
  for (int k=0;k<iters;k++){
    if (which==0||which==9) r += transform();
    if (which==1||which==9) r += lzs();
    if (which==2||which==9) r += vm();
    if (which==3||which==9) r += build_ot();
  }
  return r;
}

#include <stdio.h>
#include <stdlib.h>
int main(int c,char**v){ printf("%d\n",bench_main(atoi(v[1]),atoi(v[2]))); }
