/* dc_gte.c - PS1 GTE (COP2) for the Dreamcast runtime.
 *
 * Self-contained implementation of the documented GTE (PSX-SPX "Geometry
 * Transformation Engine"). The state lives in cpu->gte_data / cpu->gte_ctrl
 * in hardware layout, because generated code reads and writes most registers
 * there directly and calls the accessors below only for the special ones.
 */
#include "dc_runtime.h"
#include "psx_gte_divide.h"

#define D(i) cpu->gte_data[i]
#define C(i) cpu->gte_ctrl[i]

/* FLAG bits */
#define F_MAC1P (1u << 30)
#define F_MAC2P (1u << 29)
#define F_MAC3P (1u << 28)
#define F_MAC1N (1u << 27)
#define F_MAC2N (1u << 26)
#define F_MAC3N (1u << 25)
#define F_IR1   (1u << 24)
#define F_IR2   (1u << 23)
#define F_IR3   (1u << 22)
#define F_R     (1u << 21)
#define F_G     (1u << 20)
#define F_B     (1u << 19)
#define F_SZ3   (1u << 18)
#define F_DIV   (1u << 17)
#define F_MAC0P (1u << 16)
#define F_MAC0N (1u << 15)
#define F_SX2   (1u << 14)
#define F_SY2   (1u << 13)
#define F_IR0   (1u << 12)

typedef struct {
    uint32_t flag;
    int sf, lm;
} Op;

static inline int16_t lo16(uint32_t v) { return (int16_t)(v & 0xFFFFu); }
static inline int16_t hi16(uint32_t v) { return (int16_t)(v >> 16); }

/* ---- matrices / vectors from the register file ---- */
static void mat(CPUState* cpu, int base, int16_t m[3][3])
{
    m[0][0] = lo16(C(base + 0)); m[0][1] = hi16(C(base + 0));
    m[0][2] = lo16(C(base + 1)); m[1][0] = hi16(C(base + 1));
    m[1][1] = lo16(C(base + 2)); m[1][2] = hi16(C(base + 2));
    m[2][0] = lo16(C(base + 3)); m[2][1] = hi16(C(base + 3));
    m[2][2] = lo16(C(base + 4));
}
static void vec(CPUState* cpu, int n, int16_t v[3])
{
    v[0] = lo16(D(n * 2)); v[1] = hi16(D(n * 2)); v[2] = lo16(D(n * 2 + 1));
}
static inline int32_t ir(CPUState* cpu, int i) { return (int16_t)D(8 + i); }  /* i = 1..3, 0 = IR0 */

/* ---- saturation / flags ---- */
static int64_t mac_chk(Op* o, int i, int64_t v)
{
    if (v > 0x7FFFFFFFFFFLL) o->flag |= (i == 1 ? F_MAC1P : i == 2 ? F_MAC2P : F_MAC3P);
    else if (v < -0x80000000000LL) o->flag |= (i == 1 ? F_MAC1N : i == 2 ? F_MAC2N : F_MAC3N);
    /* the accumulator wraps at 44 bits */
    return (int64_t)((uint64_t)v << 20) >> 20;
}
static void set_mac(CPUState* cpu, Op* o, int i, int64_t v)
{
    v = mac_chk(o, i, v);
    D(24 + i) = (uint32_t)(int32_t)(v >> o->sf);
}
static int32_t sat_ir(Op* o, int i, int32_t v, int lm)
{
    int32_t lo = lm ? 0 : -0x8000;
    uint32_t f = i == 1 ? F_IR1 : i == 2 ? F_IR2 : F_IR3;
    if (v < lo) { o->flag |= f; return lo; }
    if (v > 0x7FFF) { o->flag |= f; return 0x7FFF; }
    return v;
}
static void set_ir(CPUState* cpu, Op* o, int i, int32_t v, int lm)
{
    D(8 + i) = (uint32_t)(int32_t)sat_ir(o, i, v, lm);
}
static void macs_to_ir(CPUState* cpu, Op* o)
{
    for (int i = 1; i <= 3; i++) set_ir(cpu, o, i, (int32_t)D(24 + i), o->lm);
}
static void set_mac0(CPUState* cpu, Op* o, int64_t v)
{
    if (v > 0x7FFFFFFFLL) o->flag |= F_MAC0P;
    else if (v < -0x80000000LL) o->flag |= F_MAC0N;
    D(24) = (uint32_t)(int32_t)v;
}
static void set_ir0(CPUState* cpu, Op* o, int64_t v)
{
    if (v < 0) { o->flag |= F_IR0; v = 0; }
    else if (v > 0x1000) { o->flag |= F_IR0; v = 0x1000; }
    D(8) = (uint32_t)(int32_t)v;
}
static uint16_t sat_z(Op* o, int64_t v)
{
    if (v < 0) { o->flag |= F_SZ3; return 0; }
    if (v > 0xFFFF) { o->flag |= F_SZ3; return 0xFFFF; }
    return (uint16_t)v;
}
static void push_sz(CPUState* cpu, uint16_t z)
{
    D(16) = D(17); D(17) = D(18); D(18) = D(19); D(19) = z;
}
static void push_sxy(CPUState* cpu, Op* o, int64_t x, int64_t y)
{
    if (x < -0x400) { o->flag |= F_SX2; x = -0x400; } else if (x > 0x3FF) { o->flag |= F_SX2; x = 0x3FF; }
    if (y < -0x400) { o->flag |= F_SY2; y = -0x400; } else if (y > 0x3FF) { o->flag |= F_SY2; y = 0x3FF; }
    D(12) = D(13); D(13) = D(14);
    D(14) = ((uint32_t)(uint16_t)(int16_t)x) | ((uint32_t)(uint16_t)(int16_t)y << 16);
}
static uint8_t sat_col(Op* o, int i, int32_t v)
{
    uint32_t f = i == 0 ? F_R : i == 1 ? F_G : F_B;
    if (v < 0) { o->flag |= f; return 0; }
    if (v > 0xFF) { o->flag |= f; return 0xFF; }
    return (uint8_t)v;
}
/* Colour FIFO push from MAC1..3 (>> 4), code byte from RGBC. */
static void push_rgb(CPUState* cpu, Op* o)
{
    uint8_t r = sat_col(o, 0, (int32_t)D(25) >> 4);
    uint8_t g = sat_col(o, 1, (int32_t)D(26) >> 4);
    uint8_t b = sat_col(o, 2, (int32_t)D(27) >> 4);
    D(20) = D(21); D(21) = D(22);
    D(22) = r | ((uint32_t)g << 8) | ((uint32_t)b << 16) | (D(6) & 0xFF000000u);
}

/* MAC = T*0x1000 + M*V ; then IR from MAC. */
static void mul_mv(CPUState* cpu, Op* o, int16_t m[3][3], const int32_t v[3], const int64_t t[3])
{
    for (int i = 0; i < 3; i++) {
        int64_t a = mac_chk(o, i + 1, t[i] * 0x1000 + (int64_t)m[i][0] * v[0]);
        a = mac_chk(o, i + 1, a + (int64_t)m[i][1] * v[1]);
        a = mac_chk(o, i + 1, a + (int64_t)m[i][2] * v[2]);
        D(25 + i) = (uint32_t)(int32_t)(a >> o->sf);
    }
    macs_to_ir(cpu, o);
}

/* ---- perspective transform of one vertex ----
   The rotation matrix and translation are unpacked once per command (RTPT
   does three vertices). The 44-bit accumulator is checked once, after the
   last addition: the 16x16-bit products cannot overflow on their own. */
static inline int64_t mac_fast(Op* o, int i, int64_t v)
{
    if ((uint64_t)(v + 0x80000000000LL) >> 44) return mac_chk(o, i, v);
    return v;
}
static void rtp_m(CPUState* cpu, Op* o, const int16_t m[3][3], const int64_t t12[3], int small_t, int n, int last)
{
    const int32_t v0 = lo16(D(n * 2)), v1 = hi16(D(n * 2)), v2 = lo16(D(n * 2 + 1));
    int64_t mac[3];
    if (small_t && (uint32_t)(v0 + 0x2000) < 0x4000 && (uint32_t)(v1 + 0x2000) < 0x4000 && (uint32_t)(v2 + 0x2000) < 0x4000) {
        /* |T| < 2^17 and |V| < 2^13: the sum stays under 2^31, so 32-bit
           arithmetic is exact and no MAC overflow is possible */
        for (int i = 0; i < 3; i++) {
            int32_t a = (int32_t)t12[i] + m[i][0] * v0 + m[i][1] * v1 + m[i][2] * v2;
            mac[i] = a;
            D(25 + i) = (uint32_t)(a >> o->sf);
        }
    } else
    for (int i = 0; i < 3; i++) {
        int64_t a = t12[i] + (int64_t)(m[i][0] * v0) + (int64_t)(m[i][1] * v1) + (int64_t)(m[i][2] * v2);
        a = mac_fast(o, i + 1, a);
        mac[i] = a;
        D(25 + i) = (uint32_t)(int32_t)(a >> o->sf);
    }
    set_ir(cpu, o, 1, (int32_t)D(25), o->lm);
    set_ir(cpu, o, 2, (int32_t)D(26), o->lm);
    /* IR3 flag is computed from MAC3 >> 12 regardless of sf (hardware quirk) */
    {
        int32_t z12 = (int32_t)(mac[2] >> 12);
        int32_t v = (int32_t)D(27);
        int32_t lo = o->lm ? 0 : -0x8000;
        if (z12 < -0x8000 || z12 > 0x7FFF) o->flag |= F_IR3;
        D(11) = (uint32_t)(int32_t)(v < lo ? lo : v > 0x7FFF ? 0x7FFF : v);
    }
    uint16_t z = sat_z(o, mac[2] >> 12);
    push_sz(cpu, z);
    int overflow = 0;
    uint32_t q = (uint32_t)psx_gte_divide((uint16_t)C(26), z, &overflow);
    if (overflow) o->flag |= F_DIV;
    int64_t sx = (int64_t)q * ir(cpu, 1) + (int32_t)C(24);
    int64_t sy = (int64_t)q * ir(cpu, 2) + (int32_t)C(25);
    push_sxy(cpu, o, sx >> 16, sy >> 16);
    if (last) {
        int64_t dq = (int64_t)(int16_t)C(27) * q + (int32_t)C(28);
        set_mac0(cpu, o, dq);
        set_ir0(cpu, o, dq >> 12);
    }
}
static void rtp(CPUState* cpu, Op* o, int first, int count)
{
    int16_t m[3][3];
    mat(cpu, 0, m);
    const int32_t tx = (int32_t)C(5), ty = (int32_t)C(6), tz = (int32_t)C(7);
    const int64_t t12[3] = { (int64_t)tx << 12, (int64_t)ty << 12, (int64_t)tz << 12 };
    const int small_t = (uint32_t)(tx + 0x20000) < 0x40000 && (uint32_t)(ty + 0x20000) < 0x40000 &&
                        (uint32_t)(tz + 0x20000) < 0x40000;
    for (int k = 0; k < count; k++) rtp_m(cpu, o, m, t12, small_t, first + k, k == count - 1);
}

/* ---- lighting pipeline pieces ---- */
static void light(CPUState* cpu, Op* o, int n)          /* IR = LLM * V */
{
    int16_t m[3][3], v16[3]; mat(cpu, 8, m); vec(cpu, n, v16);
    int32_t v[3] = { v16[0], v16[1], v16[2] };
    int64_t t[3] = { 0, 0, 0 };
    mul_mv(cpu, o, m, v, t);
}
static void color(CPUState* cpu, Op* o)                  /* IR = BK + LCM * IR */
{
    int16_t m[3][3]; mat(cpu, 16, m);
    int32_t v[3] = { ir(cpu, 1), ir(cpu, 2), ir(cpu, 3) };
    int64_t t[3] = { (int32_t)C(13), (int32_t)C(14), (int32_t)C(15) };
    mul_mv(cpu, o, m, v, t);
}
static void rgb_ir(CPUState* cpu, int32_t out[3])        /* RGBC << 4 */
{
    out[0] = (int32_t)(D(6) & 0xFF) << 4;
    out[1] = (int32_t)((D(6) >> 8) & 0xFF) << 4;
    out[2] = (int32_t)((D(6) >> 16) & 0xFF) << 4;
}
/* MAC = base + (FC - base) * IR0, base in MAC units (already << 12 by caller's sf). */
static void depth_cue(CPUState* cpu, Op* o, const int64_t base[3])
{
    int64_t fc[3] = { (int64_t)(int32_t)C(21) << 12, (int64_t)(int32_t)C(22) << 12, (int64_t)(int32_t)C(23) << 12 };
    int32_t tmp[3];
    for (int i = 0; i < 3; i++) {
        int64_t d = mac_chk(o, i + 1, fc[i] - base[i]) >> o->sf;
        tmp[i] = sat_ir(o, i + 1, (int32_t)d, 0);
    }
    for (int i = 0; i < 3; i++) {
        int64_t a = mac_chk(o, i + 1, base[i] + (int64_t)ir(cpu, 0) * tmp[i]);
        D(25 + i) = (uint32_t)(int32_t)(a >> o->sf);
    }
    macs_to_ir(cpu, o);
}
static void finish_color(CPUState* cpu, Op* o, int mode)  /* mode 0: NCS, 1: NCC, 2: NCD */
{
    if (mode == 0) { macs_to_ir(cpu, o); push_rgb(cpu, o); return; }
    int32_t c[3]; rgb_ir(cpu, c);
    int64_t prod[3];
    for (int i = 0; i < 3; i++) prod[i] = (int64_t)c[i] * ir(cpu, i + 1);
    if (mode == 1) {
        for (int i = 0; i < 3; i++) set_mac(cpu, o, i + 1, prod[i]);
        macs_to_ir(cpu, o);
    } else {
        depth_cue(cpu, o, prod);
    }
    push_rgb(cpu, o);
}
static void nc(CPUState* cpu, Op* o, int n, int mode)
{
    light(cpu, o, n);
    color(cpu, o);
    finish_color(cpu, o, mode);
}

/* ---- command entry ---- */
void gte_execute_at(CPUState* cpu, uint32_t cmd, uint32_t pc)
{
    (void)pc;
    g_dc_stats.gte_ops++;
    Op o = { 0, (cmd & (1u << 19)) ? 12 : 0, (cmd & (1u << 10)) ? 1 : 0 };
    Op* op = &o;
    switch (cmd & 0x3F) {
    case 0x01: rtp(cpu, op, 0, 1); break;                                   /* RTPS */
    case 0x30: rtp(cpu, op, 0, 3); break;                                   /* RTPT */
    case 0x06: {                                                            /* NCLIP */
        int64_t x0 = lo16(D(12)), y0 = hi16(D(12)), x1 = lo16(D(13)), y1 = hi16(D(13));
        int64_t x2 = lo16(D(14)), y2 = hi16(D(14));
        set_mac0(cpu, op, x0 * y1 + x1 * y2 + x2 * y0 - x0 * y2 - x1 * y0 - x2 * y1);
        break;
    }
    case 0x0C: {                                                            /* OP */
        int32_t d1 = (int16_t)C(0), d2 = (int16_t)C(2), d3 = (int16_t)C(4);
        set_mac(cpu, op, 1, (int64_t)ir(cpu, 3) * d2 - (int64_t)ir(cpu, 2) * d3);
        set_mac(cpu, op, 2, (int64_t)ir(cpu, 1) * d3 - (int64_t)ir(cpu, 3) * d1);
        set_mac(cpu, op, 3, (int64_t)ir(cpu, 2) * d1 - (int64_t)ir(cpu, 1) * d2);
        macs_to_ir(cpu, op);
        break;
    }
    case 0x10: {                                                            /* DPCS */
        int32_t c[3]; rgb_ir(cpu, c);
        int64_t b[3] = { (int64_t)c[0] << 12, (int64_t)c[1] << 12, (int64_t)c[2] << 12 };
        depth_cue(cpu, op, b); push_rgb(cpu, op);
        break;
    }
    case 0x2A:                                                              /* DPCT */
        for (int k = 0; k < 3; k++) {
            uint32_t c0 = D(20);
            int64_t b[3] = { (int64_t)(c0 & 0xFF) << 16, (int64_t)((c0 >> 8) & 0xFF) << 16,
                             (int64_t)((c0 >> 16) & 0xFF) << 16 };
            depth_cue(cpu, op, b); push_rgb(cpu, op);
        }
        break;
    case 0x11: {                                                            /* INTPL */
        int64_t b[3] = { (int64_t)ir(cpu, 1) << 12, (int64_t)ir(cpu, 2) << 12, (int64_t)ir(cpu, 3) << 12 };
        depth_cue(cpu, op, b); push_rgb(cpu, op);
        break;
    }
    case 0x29: {                                                            /* DCPL */
        int32_t c[3]; rgb_ir(cpu, c);
        int64_t b[3];
        for (int i = 0; i < 3; i++) b[i] = (int64_t)c[i] * ir(cpu, i + 1);
        depth_cue(cpu, op, b); push_rgb(cpu, op);
        break;
    }
    case 0x12: {                                                            /* MVMVA */
        int mx = (cmd >> 17) & 3, vx = (cmd >> 15) & 3, tx = (cmd >> 13) & 3;
        int16_t m[3][3];
        if (mx == 0) mat(cpu, 0, m);
        else if (mx == 1) mat(cpu, 8, m);
        else if (mx == 2) mat(cpu, 16, m);
        else {  /* mx = 3: garbage matrix (hardware bug) */
            int16_t r = (int16_t)((D(6) & 0xFF) << 4), t13 = lo16(C(1)), t22 = lo16(C(2));
            m[0][0] = (int16_t)-r; m[0][1] = r; m[0][2] = (int16_t)ir(cpu, 0);
            m[1][0] = m[1][1] = m[1][2] = t13;
            m[2][0] = m[2][1] = m[2][2] = t22;
        }
        int32_t v[3];
        if (vx < 3) { int16_t v16[3]; vec(cpu, vx, v16); v[0] = v16[0]; v[1] = v16[1]; v[2] = v16[2]; }
        else { v[0] = ir(cpu, 1); v[1] = ir(cpu, 2); v[2] = ir(cpu, 3); }
        int64_t t[3] = { 0, 0, 0 };
        if (tx == 0) { t[0] = (int32_t)C(5); t[1] = (int32_t)C(6); t[2] = (int32_t)C(7); }
        else if (tx == 1) { t[0] = (int32_t)C(13); t[1] = (int32_t)C(14); t[2] = (int32_t)C(15); }
        else if (tx == 2) {
            /* far colour translation: only the last product is kept (hardware bug) */
            int64_t fc[3] = { (int32_t)C(21), (int32_t)C(22), (int32_t)C(23) };
            for (int i = 0; i < 3; i++) {
                int64_t a = mac_chk(op, i + 1, fc[i] * 0x1000 + (int64_t)m[i][0] * v[0]);
                sat_ir(op, i + 1, (int32_t)(a >> op->sf), 0);
                a = mac_chk(op, i + 1, (int64_t)m[i][1] * v[1]);
                a = mac_chk(op, i + 1, a + (int64_t)m[i][2] * v[2]);
                D(25 + i) = (uint32_t)(int32_t)(a >> op->sf);
            }
            macs_to_ir(cpu, op);
            break;
        }
        mul_mv(cpu, op, m, v, t);
        break;
    }
    case 0x13: nc(cpu, op, 0, 2); break;                                    /* NCDS */
    case 0x16: nc(cpu, op, 0, 2); nc(cpu, op, 1, 2); nc(cpu, op, 2, 2); break; /* NCDT */
    case 0x1B: nc(cpu, op, 0, 1); break;                                    /* NCCS */
    case 0x3F: nc(cpu, op, 0, 1); nc(cpu, op, 1, 1); nc(cpu, op, 2, 1); break; /* NCCT */
    case 0x1E: nc(cpu, op, 0, 0); break;                                    /* NCS */
    case 0x20: nc(cpu, op, 0, 0); nc(cpu, op, 1, 0); nc(cpu, op, 2, 0); break; /* NCT */
    case 0x14: color(cpu, op); finish_color(cpu, op, 2); break;             /* CDP */
    case 0x1C: color(cpu, op); finish_color(cpu, op, 1); break;             /* CC */
    case 0x28:                                                              /* SQR */
        for (int i = 1; i <= 3; i++) set_mac(cpu, op, i, (int64_t)ir(cpu, i) * ir(cpu, i));
        macs_to_ir(cpu, op);
        break;
    case 0x2D: {                                                            /* AVSZ3 */
        int64_t s = (int64_t)(int16_t)C(29) * ((D(17) & 0xFFFF) + (D(18) & 0xFFFF) + (D(19) & 0xFFFF));
        set_mac0(cpu, op, s); D(7) = sat_z(op, s >> 12);
        break;
    }
    case 0x2E: {                                                            /* AVSZ4 */
        int64_t s = (int64_t)(int16_t)C(30) *
                    ((D(16) & 0xFFFF) + (D(17) & 0xFFFF) + (D(18) & 0xFFFF) + (D(19) & 0xFFFF));
        set_mac0(cpu, op, s); D(7) = sat_z(op, s >> 12);
        break;
    }
    case 0x3D:                                                              /* GPF */
        for (int i = 1; i <= 3; i++) set_mac(cpu, op, i, (int64_t)ir(cpu, 0) * ir(cpu, i));
        macs_to_ir(cpu, op); push_rgb(cpu, op);
        break;
    case 0x3E:                                                              /* GPL */
        for (int i = 1; i <= 3; i++)
            set_mac(cpu, op, i, ((int64_t)(int32_t)D(24 + i) << op->sf) + (int64_t)ir(cpu, 0) * ir(cpu, i));
        macs_to_ir(cpu, op); push_rgb(cpu, op);
        break;
    default:
        break;
    }
    if (o.flag & 0x7F87E000u) o.flag |= 0x80000000u;
    C(31) = o.flag;
}

/* ---- special register accessors ---- */
static uint32_t orgb(CPUState* cpu)
{
    uint32_t c[3];
    for (int i = 0; i < 3; i++) {
        int32_t v = ir(cpu, i + 1) >> 7;
        c[i] = v < 0 ? 0 : v > 0x1F ? 0x1F : (uint32_t)v;
    }
    return c[0] | (c[1] << 5) | (c[2] << 10);
}

uint32_t gte_read_data(CPUState* cpu, uint8_t r)
{
    r &= 31;
    switch (r) {
    case 1: case 3: case 5: case 8: case 9: case 10: case 11: return (uint32_t)(int32_t)(int16_t)D(r);
    case 7: case 16: case 17: case 18: case 19: return D(r) & 0xFFFFu;
    case 15: return D(14);
    case 28: case 29: return orgb(cpu);
    default: return D(r);
    }
}

uint32_t gte_read_ctrl(CPUState* cpu, uint8_t r)
{
    r &= 31;
    switch (r) {
    case 4: case 12: case 20: case 26: case 27: case 29: case 30: return (uint32_t)(int32_t)(int16_t)C(r);
    default: return C(r);
    }
}

void gte_write_data(CPUState* cpu, uint8_t r, uint32_t v)
{
    r &= 31;
    switch (r) {
    case 15: D(12) = D(13); D(13) = D(14); D(14) = v; D(15) = v; break;
    case 28:
        D(28) = v & 0x7FFF;
        D(9) = (v & 0x1F) << 7; D(10) = ((v >> 5) & 0x1F) << 7; D(11) = ((v >> 10) & 0x1F) << 7;
        break;
    case 29: break;
    case 30: {
        D(30) = v;
        uint32_t x = (v & 0x80000000u) ? ~v : v;
        uint32_t n = 0;
        while (n < 32 && !(x & 0x80000000u)) { x <<= 1; n++; }
        D(31) = n;
        break;
    }
    case 31: break;
    default: D(r) = v; break;
    }
}

void gte_write_ctrl(CPUState* cpu, uint8_t r, uint32_t v)
{
    r &= 31;
    if (r == 31) {
        v &= 0x7FFFF000u;
        if (v & 0x7F87E000u) v |= 0x80000000u;
    }
    C(r) = v;
}
