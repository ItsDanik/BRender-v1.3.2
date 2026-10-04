/*
 * Rewrites of the hottest 8 bit z-buffered rasteriser loops as plain C.
 *
 * The originals (zb8p2unl.c, zb8p2lit.c, fti8pizp.c ...) are line by line
 * transliterations of the Pentium assembly: every "register" and most
 * interpolants are globals, which on ARM means memory traffic on every pixel.
 * These versions keep the same fixed point arithmetic, carries and pixel
 * order so the output is bit-identical (checked with PENTPRIM_VERIFY=1, see
 * verify.h), but keep all per-span state in locals.
 */
#include "fastprim.h"

#include "brender.h"
#include "common.h"
#include "fpwork.h"
#include "verify.h"
#include "work.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

static inline br_uint_32 ror16(br_uint_32 v) {
    return (v >> 16) | (v << 16);
}

/*
 * Z buffered, affine textured, unlit, power of 2 texture: one half (top or
 * bottom trapezium) of a triangle. Mirrors DRAW_ZT_I8_D16_POW2.
 *
 * z is kept "rotated": integer part in the low 16 bits, fraction in the high
 * 16 bits, the carry out of the fraction is added back into the integer.
 */
static inline __attribute__((always_inline)) void draw_zt_pow2(br_uint_32* minor_x, const br_uint_32* d_minor_x, br_int_32* half_count, const int dir, const int pow2) {
    const br_uint_32 u_mask = (1u << pow2) - 1;
    const br_uint_32 v_mask = u_mask << pow2;
    const int v_shift = 16 - pow2;
    br_int_32 count = *half_count;

    if (count < 0) {
        return;
    }

    br_uint_8* const colour = work.colour.base;
    br_uint_8* const depth = work.depth.base;
    const br_uint_8* const texture = work.texture.base;
    const br_int_32 colour_stride = work.colour.stride_b;
    const br_int_32 depth_stride = work.depth.stride_b;

    const br_uint_32 d_z_x = workspace.d_z_x;
    const br_uint_32 d_u_x = workspace.d_u_x;
    const br_uint_32 d_v_x = workspace.d_v_x;
    const br_uint_32 d_xm = workspace.d_xm;
    const br_uint_32 d_xm_f = workspace.d_xm_f;
    const br_uint_32 d_minor = *d_minor_x;

    br_uint_32 s_z = workspace.s_z;
    br_uint_32 s_u = workspace.s_u;
    br_uint_32 s_v = workspace.s_v;
    br_uint_32 xm = workspace.xm;
    br_uint_32 xm_f = workspace.xm_f;
    br_uint_32 minor = *minor_x;
    br_uint_32 scan = workspace.scanAddress;
    br_uint_32 zscan = workspace.depthAddress;

    do {
        const br_uint_32 x = minor >> 16;
        br_int_32 n = (br_int_32)((xm >> 16) - x);
        br_uint_8* const line = colour + (br_uint_32)(scan + x);
        br_uint_16* const zline = (br_uint_16*)(depth + (br_uint_32)(zscan + 2 * x));
        br_uint_32 z = ror16(s_z);
        br_uint_32 u = s_u;
        br_uint_32 v = s_v;

        if (dir == DRAW_LR ? n <= 0 : n >= 0) {
            for (;;) {
                if ((br_uint_16)z <= zline[n]) {
                    const br_uint_8 texel = texture[((v >> v_shift) & v_mask) | ((u >> 16) & u_mask)];
                    if (texel != 0) {
                        zline[n] = (br_uint_16)z;
                        line[n] = texel;
                    }
                }
                if (dir == DRAW_LR) {
                    z += d_z_x;
                    z += z < d_z_x;
                    u += d_u_x;
                    v += d_v_x;
                    if (++n > 0) {
                        break;
                    }
                } else {
                    const br_uint_32 borrow = z < d_z_x;
                    z -= d_z_x + borrow;
                    u -= d_u_x;
                    v -= d_v_x;
                    if (--n < 0) {
                        break;
                    }
                }
            }
        }

        // per line: the major edge fraction carry selects the "carry" deltas
        xm_f += d_xm_f;
        if (xm_f < d_xm_f) {
            s_u += workspace.d_u_y_1;
            s_v += workspace.d_v_y_1;
            s_z += workspace.d_z_y_1;
        } else {
            s_u += workspace.d_u_y_0;
            s_v += workspace.d_v_y_0;
            s_z += workspace.d_z_y_0;
        }
        scan += colour_stride;
        zscan += depth_stride;
        minor += d_minor;
        xm += d_xm;
    } while (--count >= 0);

    workspace.s_z = s_z;
    workspace.s_u = s_u;
    workspace.s_v = s_v;
    workspace.c_u = s_u;
    workspace.c_v = s_v;
    workspace.xm = xm;
    workspace.xm_f = xm_f;
    workspace.scanAddress = scan;
    workspace.depthAddress = zscan;
    *minor_x = minor;
    *half_count = count;
}

#define DRAW_ZT_POW2_CASE(p)                                                                                    \
    case p:                                                                                                     \
        if (right_to_left) {                                                                                    \
            draw_zt_pow2(&workspace.x1, &workspace.d_x1, &workspace.topCount, DRAW_RL, p);                     \
            draw_zt_pow2(&workspace.x2, &workspace.d_x2, &workspace.bottomCount, DRAW_RL, p);                  \
        } else {                                                                                                \
            draw_zt_pow2(&workspace.x1, &workspace.d_x1, &workspace.topCount, DRAW_LR, p);                     \
            draw_zt_pow2(&workspace.x2, &workspace.d_x2, &workspace.bottomCount, DRAW_LR, p);                  \
        }                                                                                                       \
        break;

void FastDraw_ZT_I8_D16_POW2(int right_to_left, int pow2) {
    switch (pow2) {
        DRAW_ZT_POW2_CASE(3)
        DRAW_ZT_POW2_CASE(4)
        DRAW_ZT_POW2_CASE(5)
        DRAW_ZT_POW2_CASE(6)
        DRAW_ZT_POW2_CASE(7)
        DRAW_ZT_POW2_CASE(8)
    default:
        // 1024 wide textures use a mask table the original lacks (low_bit_mask[10]); keep it generic
        if (right_to_left) {
            draw_zt_pow2(&workspace.x1, &workspace.d_x1, &workspace.topCount, DRAW_RL, pow2);
            draw_zt_pow2(&workspace.x2, &workspace.d_x2, &workspace.bottomCount, DRAW_RL, pow2);
        } else {
            draw_zt_pow2(&workspace.x1, &workspace.d_x1, &workspace.topCount, DRAW_LR, pow2);
            draw_zt_pow2(&workspace.x2, &workspace.d_x2, &workspace.bottomCount, DRAW_LR, pow2);
        }
        break;
    }
}

/*
 * Z buffered, affine textured, lit (shade table), power of 2 texture: one half
 * of a triangle. Mirrors DRAW_ZTI_I8_D16_POW2 (zb8p2lit.c), including its
 * scratch0/scratch1 bookkeeping in the workspace.
 */
static inline __attribute__((always_inline)) void draw_zti_pow2(br_uint_32* minor_x, const br_uint_32* d_minor_x, br_int_32* half_count, const int dir, const int pow2) {
    const br_uint_32 u_mask = (1u << pow2) - 1;
    const br_uint_32 v_mask = u_mask << pow2;
    const int v_shift = 16 - pow2;
    br_int_32 count = *half_count;

    if (count < 0) {
        return;
    }

    br_uint_8* const colour = work.colour.base;
    br_uint_8* const depth = work.depth.base;
    const br_uint_8* const texture = work.texture.base;
    const br_uint_8* const shade = work.shade_table;
    const br_int_32 colour_stride = work.colour.stride_b;
    const br_int_32 depth_stride = work.depth.stride_b;

    const br_uint_32 d_z_x = workspace.d_z_x;
    const br_uint_32 d_i_x = workspace.d_i_x;
    const br_uint_32 d_u_x = workspace.d_u_x;
    const br_uint_32 d_v_x = workspace.d_v_x;
    const br_uint_32 d_xm = workspace.d_xm;
    const br_uint_32 d_xm_f = workspace.d_xm_f;
    const br_uint_32 d_minor = *d_minor_x;

    br_uint_32 s_z = workspace.s_z;
    br_uint_32 s_i = workspace.s_i;
    br_uint_32 s_u = workspace.s_u;
    br_uint_32 s_v = workspace.s_v;
    br_uint_32 xm = workspace.xm;
    br_uint_32 xm_f = workspace.xm_f;
    br_uint_32 minor = *minor_x;
    br_uint_32 scan = workspace.scanAddress;
    br_uint_32 zscan = workspace.depthAddress;
    br_uint_32 scratch0 = workspace.scratch0;
    br_uint_32 scratch1 = workspace.scratch1;

    do {
        const br_uint_32 x = minor >> 16;
        br_int_32 n = (br_int_32)((xm >> 16) - x);

        scratch0 = scan + x;
        if (dir == DRAW_LR ? n <= 0 : n >= 0) {
            br_uint_8* const line = colour + scratch0;
            br_uint_16* zline;
            br_uint_32 z = ror16(s_z);
            br_uint_32 i = s_i;
            br_uint_32 u = s_u;
            br_uint_32 v = s_v;

            scratch1 = zscan + 2 * x;
            zline = (br_uint_16*)(depth + scratch1);
            for (;;) {
                if ((br_uint_16)z <= zline[n]) {
                    const br_uint_8 texel = texture[((v >> v_shift) & v_mask) | ((u >> 16) & u_mask)];
                    if (texel != 0) {
                        zline[n] = (br_uint_16)z;
                        line[n] = shade[(i & 0xff0000u) >> 8 | texel];
                    }
                }
                if (dir == DRAW_LR) {
                    z += d_z_x;
                    z += z < d_z_x;
                    i += d_i_x;
                    u += d_u_x;
                    v += d_v_x;
                    if (++n > 0) {
                        break;
                    }
                } else {
                    const br_uint_32 borrow = z < d_z_x;
                    z -= d_z_x + borrow;
                    i -= d_i_x;
                    u -= d_u_x;
                    v -= d_v_x;
                    if (--n < 0) {
                        break;
                    }
                }
            }
        }

        xm_f += d_xm_f;
        if (xm_f < d_xm_f) {
            s_i += workspace.d_i_y_1;
            s_z += workspace.d_z_y_1;
            s_u += workspace.d_u_y_1;
            s_v += workspace.d_v_y_1;
        } else {
            s_i += workspace.d_i_y_0;
            s_z += workspace.d_z_y_0;
            s_u += workspace.d_u_y_0;
            s_v += workspace.d_v_y_0;
        }
        zscan += depth_stride;
        scan += colour_stride;
        minor += d_minor;
        xm += d_xm;
    } while (--count >= 0);

    workspace.s_z = s_z;
    workspace.s_i = s_i;
    workspace.s_u = s_u;
    workspace.s_v = s_v;
    workspace.c_i = s_i;
    workspace.c_u = s_u;
    workspace.c_v = s_v;
    workspace.xm = xm;
    workspace.xm_f = xm_f;
    workspace.scanAddress = scan;
    workspace.depthAddress = zscan;
    workspace.scratch0 = scratch0;
    workspace.scratch1 = scratch1;
    *minor_x = minor;
    *half_count = count;
}

#define DRAW_ZTI_POW2_CASE(p)                                                                                   \
    case p:                                                                                                     \
        if (right_to_left) {                                                                                    \
            draw_zti_pow2(&workspace.x1, &workspace.d_x1, &workspace.topCount, DRAW_RL, p);                    \
            draw_zti_pow2(&workspace.x2, &workspace.d_x2, &workspace.bottomCount, DRAW_RL, p);                 \
        } else {                                                                                                \
            draw_zti_pow2(&workspace.x1, &workspace.d_x1, &workspace.topCount, DRAW_LR, p);                    \
            draw_zti_pow2(&workspace.x2, &workspace.d_x2, &workspace.bottomCount, DRAW_LR, p);                 \
        }                                                                                                       \
        break;

void FastDraw_ZTI_I8_D16_POW2(int right_to_left, int pow2) {
    switch (pow2) {
        DRAW_ZTI_POW2_CASE(3)
        DRAW_ZTI_POW2_CASE(4)
        DRAW_ZTI_POW2_CASE(5)
        DRAW_ZTI_POW2_CASE(6)
        DRAW_ZTI_POW2_CASE(7)
        DRAW_ZTI_POW2_CASE(8)
    default:
        if (right_to_left) {
            draw_zti_pow2(&workspace.x1, &workspace.d_x1, &workspace.topCount, DRAW_RL, pow2);
            draw_zti_pow2(&workspace.x2, &workspace.d_x2, &workspace.bottomCount, DRAW_RL, pow2);
        } else {
            draw_zti_pow2(&workspace.x1, &workspace.d_x1, &workspace.topCount, DRAW_LR, pow2);
            draw_zti_pow2(&workspace.x2, &workspace.d_x2, &workspace.bottomCount, DRAW_LR, pow2);
        }
        break;
    }
}

/*
 * Z buffered, perspective correct textured, unlit: TrapeziumRender_ZPT_I8_D16
 * and ScanlineRender_ZPT_I8_D16 in fti8pizp.c.
 *
 * Perspective is done without divisions: u and v are tracked as numerators
 * with an error term against the denominator q, and the texel offset
 * ("source") is stepped one texel at a time whenever an error term leaves
 * 0..q. The source keeps u in byte 0 and v in byte 1 so that both wrap
 * independently ("pre" shifts u to the top of its byte, "post" undoes it).
 */

typedef struct {
    int pre;
    br_uint_32 incu, decu, incv, decv;
    int post1;
    br_uint_32 post2;
} tZPT_size;

static const tZPT_size zpt_sizes[] = {
    { 3, 8, 8, 1, 1, 3, 0x03ff },     // 32x32
    { 2, 4, 4, 1, 1, 2, 0x0fff },     // 64x64
    { 1, 2, 2, 1, 1, 2, 0x3fff },     // 128x128
    { 0, 1, 1, 1, 1, 0, 0xffffffff }, // 256x256
};

enum { SCAN_I,
    SCAN_D,
    SCAN_B };

// add to byte 0 / byte 1 only, like the x86 "inc al" / "inc ah" the format relies on
static inline br_uint_32 lane0_add(br_uint_32 s, br_uint_32 k) {
    return (s & 0xffffff00u) | ((s + k) & 0xffu);
}

static inline br_uint_32 lane1_add(br_uint_32 s, br_uint_32 k) {
    return (s & 0xffff00ffu) | ((s + (k << 8)) & 0xff00u);
}


/*
 * Multi-step error correction. The original steps the texel one at a time
 * while the error term is out of range, often 2-3 times per pixel on
 * minified textures, with an unpredictable loop exit each time. For q > 0
 * the number of steps k follows from the error term, so guess k from the
 * previous pixel, apply k steps at once (one multiply) and fix the guess up.
 * k steps of 32-bit wrapping arithmetic equal one step of k times the size.
 * For q <= 0 the original loops are used, see the callers.
 */
static inline br_uint_32 lane_add(br_uint_32 s, br_uint_32 k, const int lane) {
    return lane == 0 ? lane0_add(s, k) : lane1_add(s, k);
}

// while ((int)num >= 0) { lane += inc; d -= dq; num -= q; }   requires (int)q > 0, (int)num >= 0
static inline __attribute__((always_inline)) void steps_down_to_negative(br_uint_32* num, br_uint_32* d, br_uint_32* source,
    const br_uint_32 q, const br_uint_32 dq, const br_uint_32 inc, const int lane, br_int_32* guess) {
    br_int_32 k = *guess;
    br_int_64 t = (br_int_64)(br_int_32)*num - (br_int_64)k * (br_int_32)q;
    while (t >= 0) {
        t -= (br_int_32)q;
        k++;
    }
    while (k > 1 && t + (br_int_32)q < 0) {
        t += (br_int_32)q;
        k--;
    }
    *num = (br_uint_32)t;
    *d -= (br_uint_32)k * dq;
    *source = lane_add(*source, (br_uint_32)k * inc, lane);
    *guess = k;
}

// while ((int)num < 0) { lane -= dec; d += dq; num += q; }   requires (int)q > 0, (int)num < 0
static inline __attribute__((always_inline)) void steps_up_to_nonnegative(br_uint_32* num, br_uint_32* d, br_uint_32* source,
    const br_uint_32 q, const br_uint_32 dq, const br_uint_32 dec, const int lane, br_int_32* guess) {
    br_int_32 k = *guess;
    br_int_64 t = (br_int_64)(br_int_32)*num + (br_int_64)k * (br_int_32)q;
    while (t < 0) {
        t += (br_int_32)q;
        k++;
    }
    while (k > 1 && t - (br_int_32)q >= 0) {
        t -= (br_int_32)q;
        k--;
    }
    *num = (br_uint_32)t;
    *d += (br_uint_32)k * dq;
    *source = lane_add(*source, -((br_uint_32)k * dec), lane);
    *guess = k;
}

// while ((int)num >= (int)q) { lane += inc; d -= dq; num -= q; }   requires (int)q > 0, (int)num >= (int)q
static inline __attribute__((always_inline)) void steps_down_below_q(br_uint_32* num, br_uint_32* d, br_uint_32* source,
    const br_uint_32 q, const br_uint_32 dq, const br_uint_32 inc, const int lane, br_int_32* guess) {
    br_int_32 k = *guess;
    br_int_64 t = (br_int_64)(br_int_32)*num - (br_int_64)k * (br_int_32)q;
    while (t >= (br_int_32)q) {
        t -= (br_int_32)q;
        k++;
    }
    while (k > 1 && t < 0) {
        t += (br_int_32)q;
        k--;
    }
    *num = (br_uint_32)t;
    *d -= (br_uint_32)k * dq;
    *source = lane_add(*source, (br_uint_32)k * inc, lane);
    *guess = k;
}

// interpolant state of the last scanline drawn (work.tsl in the original).
// The original only stores zdest after stepping past the first pixel, so a
// single pixel scanline leaves the previous value: zdest_set says whether to.
typedef struct {
    br_uint_8* dest;
    br_uint_16* zdest;
    int zdest_set;
    br_uint_32 z, q, u_num, du_num, v_num, dv_num;
} tZPT_scan_state;

static inline __attribute__((always_inline)) void zpt_scanline(const int predict, const int dir, const tZPT_size* sz, const int udir, const int vdir,
    br_uint_8* const start, br_uint_8* const end, br_uint_16* zdest, br_uint_32 source, br_uint_32 z, br_uint_32 q,
    br_uint_32 u_num, br_uint_32 du_num, br_uint_32 v_num, br_uint_32 dv_num, tZPT_scan_state* out) {

    const br_uint_8* const texture = work.texture.base;
    br_uint_8* dest = start;
    const br_uint_32 dz = work.tsl.dz;
    const br_uint_32 dq = work.tsl.ddenominator;

    br_int_32 u_guess = 1, v_guess = 1;

    out->zdest_set = 0;
    for (;;) {
        // depth first: no texture fetch (cache miss) for hidden pixels
        if ((br_uint_16)z <= *zdest) {
            const br_uint_8 texel = texture[source];
            if (texel != 0) {
                *zdest = (br_uint_16)z;
                *dest = texel;
            }
        }
        source <<= sz->pre;
        if (dir == DIR_F ? dest + 1 > end : dest - 1 < end) {
            break;
        }
        if (dir == DIR_F) {
            dest++;
            zdest++;
            z += dz;
            z += z < dz;
            q += dq;
            u_num += du_num;
        } else {
            br_uint_32 borrow = z < dz;
            dest--;
            zdest--;
            z -= dz + borrow;
            q -= dq;
            u_num -= du_num;
        }

        // step u until its error term is back in range
        if (udir == SCAN_B) {
            if ((br_int_32)u_num < 0) {
                if (predict && (br_int_32)q > 0) {
                    steps_up_to_nonnegative(&u_num, &du_num, &source, q, dq, sz->decu, 0, &u_guess);
                } else {
                    do {
                        source = lane0_add(source, -sz->decu);
                        du_num += dq;
                        u_num += q;
                    } while ((br_int_32)u_num < 0);
                }
            } else if ((br_int_32)u_num >= (br_int_32)q) {
                if (predict && (br_int_32)q > 0) {
                    steps_down_below_q(&u_num, &du_num, &source, q, dq, sz->incu, 0, &u_guess);
                } else {
                    do {
                        source = lane0_add(source, sz->incu);
                        du_num -= dq;
                        u_num -= q;
                    } while ((br_int_32)u_num >= (br_int_32)q);
                }
            }
        } else if (udir == SCAN_I) {
            if ((br_int_32)u_num >= 0) {
                if (predict && (br_int_32)q > 0) {
                    steps_down_to_negative(&u_num, &du_num, &source, q, dq, sz->incu, 0, &u_guess);
                } else {
                    do {
                        source = lane0_add(source, sz->incu);
                        du_num -= dq;
                        u_num -= q;
                    } while ((br_int_32)u_num >= 0);
                }
            }
        } else {
            if ((br_int_32)u_num < 0) {
                if (predict && (br_int_32)q > 0) {
                    // with q > 0 the carry out of u_num + q happens exactly when it turns non-negative
                    steps_up_to_nonnegative(&u_num, &du_num, &source, q, dq, sz->decu, 0, &u_guess);
                } else {
                    br_uint_32 carry;
                    do {
                        source = lane0_add(source, -sz->decu);
                        du_num += dq;
                        u_num += q;
                        carry = u_num < q;
                    } while (!carry);
                }
            }
        }

        if (dir == DIR_F) {
            v_num += dv_num;
        } else {
            v_num -= dv_num;
        }

        // and v
        if (vdir == SCAN_B) {
            if ((br_int_32)v_num < 0) {
                if (predict && (br_int_32)q > 0) {
                    steps_up_to_nonnegative(&v_num, &dv_num, &source, q, dq, sz->decv, 1, &v_guess);
                } else {
                    do {
                        source = lane1_add(source, -sz->decv);
                        dv_num += dq;
                        v_num += q;
                    } while ((br_int_32)v_num < 0);
                }
            } else if ((br_int_32)v_num >= (br_int_32)q) {
                if (predict && (br_int_32)q > 0) {
                    steps_down_below_q(&v_num, &dv_num, &source, q, dq, sz->incv, 1, &v_guess);
                } else {
                    do {
                        source = lane1_add(source, sz->incv);
                        dv_num -= dq;
                        v_num -= q;
                    } while ((br_int_32)v_num >= (br_int_32)q);
                }
            }
        } else if (vdir == SCAN_I) {
            if ((br_int_32)v_num >= 0) {
                if (predict && (br_int_32)q > 0) {
                    steps_down_to_negative(&v_num, &dv_num, &source, q, dq, sz->incv, 1, &v_guess);
                } else {
                    do {
                        source = lane1_add(source, sz->incv);
                        dv_num -= dq;
                        v_num -= q;
                    } while ((br_int_32)v_num >= 0);
                }
            }
        } else {
            if ((br_int_32)v_num < 0) {
                if (predict && (br_int_32)q > 0) {
                    steps_up_to_nonnegative(&v_num, &dv_num, &source, q, dq, sz->decv, 1, &v_guess);
                } else {
                    br_uint_32 carry;
                    do {
                        source = lane1_add(source, -sz->decv);
                        dv_num += dq;
                        v_num += q;
                        carry = v_num < q;
                    } while (!carry);
                }
            }
        }

        source >>= sz->post1;
        source &= sz->post2;
    }

    out->dest = dest;
    if (dest != start) {
        out->zdest = zdest;
        out->zdest_set = 1;
    }
    out->z = z;
    out->q = q;
    out->u_num = u_num;
    out->du_num = du_num;
    out->v_num = v_num;
    out->dv_num = dv_num;
}

static inline __attribute__((always_inline)) void zpt_trapezium(const int predict, const int dir, const tZPT_size* sz) {
    br_int_32 count = workspace.topCount;
    if (count < 0) {
        return;
    }

    br_uint_8* const colour = work.colour.base;
    br_uint_8* const depth = work.depth.base;
    const br_int_32 colour_stride = work.colour.stride_b;
    const br_int_32 depth_stride = work.depth.stride_b;

    const br_uint_32 main_d = workspace.d_xm;
    const br_uint_32 top_d = workspace.d_x1;
    const br_uint_32 q_grad = work.pq.grad_x;
    const br_uint_32 q_nocarry = work.pq.d_nocarry;
    const br_uint_32 q_carry = work.pq.d_carry;

    br_uint_32 scan = workspace.scanAddress;
    br_uint_32 zscan = workspace.depthAddress;
    br_uint_32 main_i = workspace.xm;
    br_uint_32 top_i = workspace.x1;
    br_uint_32 s_z = workspace.s_z;
    br_uint_32 q = work.pq.current;
    br_uint_32 u = work.pu.current, u_grad = work.pu.grad_x, u_nocarry = work.pu.d_nocarry;
    br_uint_32 v = work.pv.current, v_grad = work.pv.grad_x, v_nocarry = work.pv.d_nocarry;
    br_uint_32 source = work.tsl.source;

    int drawn = 0;
    br_uint_16* tsl_zdest = (br_uint_16*)work.tsl.zdest;
    br_uint_8 *start_ptr = NULL, *end_ptr = NULL;
    br_uint_16* zstart_ptr = NULL;
    tZPT_scan_state last;

    br_int_32 x_end = top_i >> 16;
    br_int_32 x_start = main_i & 0xffff;

    for (;;) {
        if (dir == DIR_F ? x_start <= x_end : x_start >= x_end) {
            end_ptr = colour + (br_uint_32)(scan + x_end);
            start_ptr = colour + (br_uint_32)(scan + x_start);
            zstart_ptr = (br_uint_16*)(depth + (br_uint_32)(zscan + 2 * x_start));

            // bring the u and v error terms of the scanline start into 0..q
            source <<= sz->pre;
            if ((br_int_32)u >= (br_int_32)q) {
                do {
                    source = lane0_add(source, sz->incu);
                    u_grad -= q_grad;
                    u_nocarry -= q_nocarry;
                    u -= q;
                } while ((br_int_32)u >= (br_int_32)q);
            } else if ((br_int_32)u < 0) {
                do {
                    source = lane0_add(source, -sz->decu);
                    u_grad += q_grad;
                    u_nocarry += q_nocarry;
                    u += q;
                } while ((br_int_32)u < 0);
            }
            if ((br_int_32)v >= (br_int_32)q) {
                do {
                    source = lane1_add(source, sz->incv);
                    v_grad -= q_grad;
                    v_nocarry -= q_nocarry;
                    v -= q;
                } while ((br_int_32)v >= (br_int_32)q);
            } else if ((br_int_32)v < 0) {
                do {
                    source = lane1_add(source, -sz->decv);
                    v_grad += q_grad;
                    v_nocarry += q_nocarry;
                    v += q;
                } while ((br_int_32)v < 0);
            }
            source >>= sz->post1;
            source &= sz->post2;

            // pick the scanline variant by the directions u, v and q move in
            {
                const br_int_32 ug = (br_int_32)u_grad, vg = (br_int_32)v_grad, qg = (br_int_32)q_grad;
                const br_uint_32 z0 = ror16(s_z);
                int ud, vd;
#define POS(a) (dir == DIR_F ? (a) > 0 : (a) < 0)
#define LE(a, b) (dir == DIR_F ? (a) <= (b) : (a) >= (b))
#define GT(a, b) (dir == DIR_F ? (a) > (b) : (a) < (b))
                if (!(dir == DIR_F ? qg < 0 : qg > 0)) {
                    ud = POS(ug) ? SCAN_I : SCAN_D;
                    vd = POS(vg) ? SCAN_I : SCAN_D;
                } else if (POS(ug)) {
                    if (POS(vg)) {
                        ud = SCAN_I, vd = SCAN_I;
                    } else if (LE(vg, qg)) {
                        ud = SCAN_I, vd = SCAN_D;
                    } else {
                        ud = SCAN_B, vd = SCAN_B;
                    }
                } else {
                    if (GT(ug, qg)) {
                        ud = SCAN_B, vd = SCAN_B;
                    } else if (POS(vg)) {
                        ud = SCAN_D, vd = SCAN_I;
                    } else if (LE(vg, qg)) {
                        ud = SCAN_D, vd = SCAN_D;
                    } else {
                        ud = SCAN_B, vd = SCAN_B;
                    }
                }
#undef POS
#undef LE
#undef GT

                // numerators start relative to q when stepping upwards
#define SCAN(UD, VD)                                                                                         \
    zpt_scanline(predict, dir, sz, UD, VD, start_ptr, end_ptr, zstart_ptr, source, z0, q,                            \
        UD == SCAN_I ? u - q : u, UD == SCAN_I ? u_grad - q_grad : u_grad,                                  \
        VD == SCAN_I ? v - q : v, VD == SCAN_I ? v_grad - q_grad : v_grad, &last)
                if (ud == SCAN_I) {
                    if (vd == SCAN_I) {
                        SCAN(SCAN_I, SCAN_I);
                    } else {
                        SCAN(SCAN_I, SCAN_D);
                    }
                } else if (ud == SCAN_D) {
                    if (vd == SCAN_I) {
                        SCAN(SCAN_D, SCAN_I);
                    } else {
                        SCAN(SCAN_D, SCAN_D);
                    }
                } else {
                    SCAN(SCAN_B, SCAN_B);
                }
#undef SCAN
            }
            drawn = 1;
            if (last.zdest_set) {
                tsl_zdest = last.zdest;
            }
        }

        // next scanline: the major edge fraction carry selects the deltas
        scan += colour_stride;
        zscan += depth_stride;
        {
            const br_uint_32 next = main_i + main_d;
            if (next < main_d) {
                main_i = next + 1;
                q += q_carry;
                s_z += workspace.d_z_y_1;
                // u and v have no carry delta of their own: nocarry + one step along x
                u += u_nocarry + u_grad;
                v += v_nocarry + v_grad;
            } else {
                main_i = next;
                q += q_nocarry;
                s_z += workspace.d_z_y_0;
                u += u_nocarry;
                v += v_nocarry;
            }
        }
        top_i += top_d;
        x_end = top_i >> 16;
        x_start = main_i & 0xffff;
        if (--count < 0) {
            break;
        }
    }

    workspace.scanAddress = scan;
    workspace.depthAddress = zscan;
    workspace.xm = main_i;
    workspace.x1 = top_i;
    workspace.s_z = s_z;
    workspace.topCount = count;
    work.pq.current = q;
    work.pu.current = u;
    work.pu.grad_x = u_grad;
    work.pu.d_nocarry = u_nocarry;
    work.pv.current = v;
    work.pv.grad_x = v_grad;
    work.pv.d_nocarry = v_nocarry;
    if (drawn) {
        work.tsl.source = source;
        work.tsl.start = (char*)start_ptr;
        work.tsl.end = (char*)end_ptr;
        work.tsl.zstart = (char*)zstart_ptr;
        work.tsl.dest = (char*)last.dest;
        work.tsl.zdest = (char*)tsl_zdest;
        work.tsl.z = last.z;
        work.tsl.denominator = last.q;
        work.tsl.u_numerator = last.u_num;
        work.tsl.du_numerator = last.du_num;
        work.tsl.v_numerator = last.v_num;
        work.tsl.dv_numerator = last.dv_num;
    }
}

static void zpt_trapezium_variant(const int predict, int dir, int size) {
    switch (size * 2 + (dir == DIR_B)) {
    case 0:
        zpt_trapezium(predict, DIR_F, &zpt_sizes[0]);
        break;
    case 1:
        zpt_trapezium(predict, DIR_B, &zpt_sizes[0]);
        break;
    case 2:
        zpt_trapezium(predict, DIR_F, &zpt_sizes[1]);
        break;
    case 3:
        zpt_trapezium(predict, DIR_B, &zpt_sizes[1]);
        break;
    case 4:
        zpt_trapezium(predict, DIR_F, &zpt_sizes[2]);
        break;
    case 5:
        zpt_trapezium(predict, DIR_B, &zpt_sizes[2]);
        break;
    case 6:
        zpt_trapezium(predict, DIR_F, &zpt_sizes[3]);
        break;
    case 7:
        zpt_trapezium(predict, DIR_B, &zpt_sizes[3]);
        break;
    }
}

/*
 * "Fast" renderer (gPentprim_fast): perspective texture mapping by
 * subdivision. NOT bit-identical to the original.
 *
 * The original steps an error term per pixel to get the exact texel of every
 * pixel. Here the exact texture coordinate is only worked out every
 * FAST_RUN pixels (one divide) and interpolated linearly in between, as in
 * most software renderers of the time. Edges, depth and the first and last
 * texel of every scanline are the same as in the exact version, the texels
 * in between can be off by a fraction of a texel on steep perspective.
 *
 * With `lit` this also replaces the original ZPTI (shade table) trapezium.
 */
#define FAST_RUN_SHIFT 4
#define FAST_RUN (1 << FAST_RUN_SHIFT)

// 1/n for the last, shorter run of a scanline
static const float fast_recip[FAST_RUN] = {
    0.0f, 1.0f / 1, 1.0f / 2, 1.0f / 3, 1.0f / 4, 1.0f / 5, 1.0f / 6, 1.0f / 7,
    1.0f / 8, 1.0f / 9, 1.0f / 10, 1.0f / 11, 1.0f / 12, 1.0f / 13, 1.0f / 14, 1.0f / 15
};

static inline br_int_32 fast_fix(float f) {
    if (f > 1e9f) {
        f = 1e9f;
    }
    if (f < -1e9f) {
        f = -1e9f;
    }
    return (br_int_32)f;
}

/*
 * One scanline of `count` pixels. u_num/q and v_num/q are the position inside
 * the texel `source`, the gradients are per pixel in the direction of travel.
 * u and v are kept as 16.16 texels; only the low `bits` of the integer part
 * are used, so they wrap like the original.
 */
static inline __attribute__((always_inline)) void zpt_fast_scanline(const int lit, const int dir, const int bits,
    br_uint_8* dest, br_uint_16* zdest, br_int_32 count, const br_uint_32 source, br_uint_32 z, br_uint_32 i,
    const br_int_32 q, const br_int_32 dq, const br_int_32 u_num, const br_int_32 du_num, const br_int_32 v_num, const br_int_32 dv_num) {

    const br_uint_8* const texture = work.texture.base;
    const br_uint_8* const shade = work.shade_table;
    const br_uint_32 dz = work.tsl.dz;
    const br_uint_32 di = work.tsl.di;
    const br_uint_32 mask = (1u << bits) - 1;
    const br_uint_32 vmask = mask << bits;
    const br_uint_32 u_base = (source & mask) << 16;
    const br_uint_32 v_base = ((source >> bits) & mask) << 16;
    const float fq = (float)q, fdq = (float)dq;
    const float fu = (float)u_num, fdu = (float)du_num;
    const float fv = (float)v_num, fdv = (float)dv_num;
    // q is positive inside a triangle. If it is not at either end of the
    // scanline (degenerate input), draw the scanline with its first texel.
    const int flat = q <= 0 || (br_int_64)q + (br_int_64)(count - 1) * dq <= 0;
    br_uint_32 u, v, u_end, v_end;
    br_int_32 du, dv;
    br_int_32 k = 0;

    if (flat) {
        u = u_base;
        v = v_base;
    } else {
        const float r = 65536.0f / fq;
        u = u_base + (br_uint_32)fast_fix(fu * r);
        v = v_base + (br_uint_32)fast_fix(fv * r);
    }

    while (count > 0) {
        br_int_32 n;

        // exact coordinates at the end of this run. The last run ends on the
        // last pixel so nothing is sampled outside the scanline.
        if (count > FAST_RUN) {
            n = FAST_RUN;
            k += FAST_RUN;
        } else {
            n = count;
            k += count - 1;
        }
        if (flat || count == 1) {
            u_end = u;
            v_end = v;
            du = 0;
            dv = 0;
        } else {
            const float fk = (float)k;
            const float r = 65536.0f / (fq + fk * fdq);
            u_end = u_base + (br_uint_32)fast_fix((fu + fk * fdu) * r);
            v_end = v_base + (br_uint_32)fast_fix((fv + fk * fdv) * r);
            if (count > FAST_RUN) {
                du = (br_int_32)(u_end - u) >> FAST_RUN_SHIFT;
                dv = (br_int_32)(v_end - v) >> FAST_RUN_SHIFT;
            } else {
                const float rn = fast_recip[count - 1];
                du = (br_int_32)((float)(br_int_32)(u_end - u) * rn);
                dv = (br_int_32)((float)(br_int_32)(v_end - v) * rn);
            }
        }
        count -= n;

        do {
            // depth first: no texture fetch (cache miss) for hidden pixels
            if ((br_uint_16)z <= *zdest) {
                const br_uint_8 texel = texture[((v >> (16 - bits)) & vmask) | ((u >> 16) & mask)];
                if (texel != 0) {
                    *zdest = (br_uint_16)z;
                    *dest = lit ? shade[((i >> 8) & 0xff00) | texel] : texel;
                }
            }
            u += du;
            v += dv;
            if (dir == DIR_F) {
                dest++;
                zdest++;
                z += dz;
                z += z < dz;
                i += di;
            } else {
                br_uint_32 borrow = z < dz;
                dest--;
                zdest--;
                z -= dz + borrow;
                i -= di;
            }
        } while (--n > 0);

        // restart every run from the exact value
        u = u_end;
        v = v_end;
    }
}

// The edge walk of zpt_trapezium with the fast scanline
static inline __attribute__((always_inline)) void zpt_fast_trapezium(const int lit, const int dir, const tZPT_size* sz, const int bits) {
    br_int_32 count = workspace.topCount;
    if (count < 0) {
        return;
    }

    br_uint_8* const colour = work.colour.base;
    br_uint_8* const depth = work.depth.base;
    const br_int_32 colour_stride = work.colour.stride_b;
    const br_int_32 depth_stride = work.depth.stride_b;

    const br_uint_32 main_d = workspace.d_xm;
    const br_uint_32 top_d = workspace.d_x1;
    const br_uint_32 q_grad = work.pq.grad_x;
    const br_uint_32 q_nocarry = work.pq.d_nocarry;
    const br_uint_32 q_carry = work.pq.d_carry;

    br_uint_32 scan = workspace.scanAddress;
    br_uint_32 zscan = workspace.depthAddress;
    br_uint_32 main_i = workspace.xm;
    br_uint_32 top_i = workspace.x1;
    br_uint_32 s_z = workspace.s_z;
    br_uint_32 s_i = workspace.s_i;
    br_uint_32 q = work.pq.current;
    br_uint_32 u = work.pu.current, u_grad = work.pu.grad_x, u_nocarry = work.pu.d_nocarry;
    br_uint_32 v = work.pv.current, v_grad = work.pv.grad_x, v_nocarry = work.pv.d_nocarry;
    br_uint_32 source = work.tsl.source;

    br_int_32 x_end = top_i >> 16;
    br_int_32 x_start = main_i & 0xffff;

    for (;;) {
        if (dir == DIR_F ? x_start <= x_end : x_start >= x_end) {
            // bring the u and v error terms of the scanline start into 0..q
            source <<= sz->pre;
            if ((br_int_32)u >= (br_int_32)q) {
                do {
                    source = lane0_add(source, sz->incu);
                    u_grad -= q_grad;
                    u_nocarry -= q_nocarry;
                    u -= q;
                } while ((br_int_32)u >= (br_int_32)q);
            } else if ((br_int_32)u < 0) {
                do {
                    source = lane0_add(source, -sz->decu);
                    u_grad += q_grad;
                    u_nocarry += q_nocarry;
                    u += q;
                } while ((br_int_32)u < 0);
            }
            if ((br_int_32)v >= (br_int_32)q) {
                do {
                    source = lane1_add(source, sz->incv);
                    v_grad -= q_grad;
                    v_nocarry -= q_nocarry;
                    v -= q;
                } while ((br_int_32)v >= (br_int_32)q);
            } else if ((br_int_32)v < 0) {
                do {
                    source = lane1_add(source, -sz->decv);
                    v_grad += q_grad;
                    v_nocarry += q_nocarry;
                    v += q;
                } while ((br_int_32)v < 0);
            }
            source >>= sz->post1;
            source &= sz->post2;

            if (dir == DIR_F) {
                zpt_fast_scanline(lit, dir, bits, colour + (br_uint_32)(scan + x_start), (br_uint_16*)(depth + (br_uint_32)(zscan + 2 * x_start)),
                    x_end - x_start + 1, source, ror16(s_z), s_i, q, q_grad, u, u_grad, v, v_grad);
            } else {
                zpt_fast_scanline(lit, dir, bits, colour + (br_uint_32)(scan + x_start), (br_uint_16*)(depth + (br_uint_32)(zscan + 2 * x_start)),
                    x_start - x_end + 1, source, ror16(s_z), s_i, q, -q_grad, u, -u_grad, v, -v_grad);
            }
        }

        // next scanline: the major edge fraction carry selects the deltas
        scan += colour_stride;
        zscan += depth_stride;
        {
            const br_uint_32 next = main_i + main_d;
            if (next < main_d) {
                main_i = next + 1;
                q += q_carry;
                s_z += workspace.d_z_y_1;
                s_i += workspace.d_i_y_1;
                u += u_nocarry + u_grad;
                v += v_nocarry + v_grad;
            } else {
                main_i = next;
                q += q_nocarry;
                s_z += workspace.d_z_y_0;
                s_i += workspace.d_i_y_0;
                u += u_nocarry;
                v += v_nocarry;
            }
        }
        top_i += top_d;
        x_end = top_i >> 16;
        x_start = main_i & 0xffff;
        if (--count < 0) {
            break;
        }
    }

    workspace.scanAddress = scan;
    workspace.depthAddress = zscan;
    workspace.xm = main_i;
    workspace.x1 = top_i;
    workspace.s_z = s_z;
    if (lit) {
        workspace.s_i = s_i;
    }
    workspace.topCount = count;
    work.pq.current = q;
    work.pu.current = u;
    work.pu.grad_x = u_grad;
    work.pu.d_nocarry = u_nocarry;
    work.pv.current = v;
    work.pv.grad_x = v_grad;
    work.pv.d_nocarry = v_nocarry;
    work.tsl.source = source;
}

static void zpt_fast_trapezium_variant(const int lit, int dir, int size) {
#define VARIANT(LIT)                                             \
    switch (size * 2 + (dir == DIR_B)) {                         \
    case 0:                                                      \
        zpt_fast_trapezium(LIT, DIR_F, &zpt_sizes[0], 5);        \
        break;                                                   \
    case 1:                                                      \
        zpt_fast_trapezium(LIT, DIR_B, &zpt_sizes[0], 5);        \
        break;                                                   \
    case 2:                                                      \
        zpt_fast_trapezium(LIT, DIR_F, &zpt_sizes[1], 6);        \
        break;                                                   \
    case 3:                                                      \
        zpt_fast_trapezium(LIT, DIR_B, &zpt_sizes[1], 6);        \
        break;                                                   \
    case 4:                                                      \
        zpt_fast_trapezium(LIT, DIR_F, &zpt_sizes[2], 7);        \
        break;                                                   \
    case 5:                                                      \
        zpt_fast_trapezium(LIT, DIR_B, &zpt_sizes[2], 7);        \
        break;                                                   \
    case 6:                                                      \
        zpt_fast_trapezium(LIT, DIR_F, &zpt_sizes[3], 8);        \
        break;                                                   \
    case 7:                                                      \
        zpt_fast_trapezium(LIT, DIR_B, &zpt_sizes[3], 8);        \
        break;                                                   \
    }
    if (lit) {
        VARIANT(1)
    } else {
        VARIANT(0)
    }
#undef VARIANT
}

void FastTrapezium_ZPTI_I8_D16(int dir, int size) {
    zpt_fast_trapezium_variant(1, dir, size);
}

/*
 * Development: PENTPRIM_AB_ZPT=1 alternates the step prediction on and off
 * call by call and reports the time of each at exit (same scenes for both).
 */
static int ab_zpt = -1;
static long ab_calls[2];
static long long ab_ns[2];

static void ab_zpt_summary(void) {
    fprintf(stderr, "ZPT trapezium A/B: plain %.0fns/call, predicted %.0fns/call (%ld calls)\n",
        ab_calls[0] ? (double)ab_ns[0] / ab_calls[0] : 0, ab_calls[1] ? (double)ab_ns[1] / ab_calls[1] : 0, ab_calls[0] + ab_calls[1]);
}

void FastTrapezium_ZPT_I8_D16(int dir, int size) {
    if (ab_zpt < 0) {
        const char* env = getenv("PENTPRIM_AB_ZPT");
        ab_zpt = env != NULL && env[0] == '1';
        if (ab_zpt) {
            atexit(ab_zpt_summary);
        }
    }
    if (ab_zpt) {
        struct timespec t0, t1;
        int predict = (ab_calls[0] + ab_calls[1]) & 1;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        if (predict) {
            zpt_trapezium_variant(1, dir, size);
        } else {
            zpt_trapezium_variant(0, dir, size);
        }
        clock_gettime(CLOCK_MONOTONIC, &t1);
        ab_ns[predict] += (t1.tv_sec - t0.tv_sec) * 1000000000LL + (t1.tv_nsec - t0.tv_nsec);
        ab_calls[predict]++;
        return;
    }
    if (gPentprim_fast && !gPentprim_nested) {
        zpt_fast_trapezium_variant(0, dir, size);
        return;
    }
    zpt_trapezium_variant(1, dir, size);
}

/*
 * Z buffered, flat coloured: one half of a triangle. Mirrors DRAW_Z_I8_D16
 * (zb8.c). The z carry of one pixel is added at the start of the next, and
 * the depth test compares all 32 bits against the previous z with the
 * buffered depth in its low half - both kept for identical results.
 */
static inline __attribute__((always_inline)) void draw_z(br_uint_32* minor_x, const br_uint_32* d_minor_x, br_int_32* half_count, const int dir) {
    br_int_32 count = *half_count;

    if (count < 0) {
        return;
    }

    br_uint_8* const colour = work.colour.base;
    br_uint_8* const depth = work.depth.base;
    const br_int_32 colour_stride = work.colour.stride_b;
    const br_int_32 depth_stride = work.depth.stride_b;
    const br_uint_8 pixel = workspace.colour & 0xff;

    const br_uint_32 d_z_x = workspace.d_z_x;
    const br_uint_32 d_xm = workspace.d_xm;
    const br_uint_32 d_xm_f = workspace.d_xm_f;
    const br_uint_32 d_minor = *d_minor_x;

    br_uint_32 s_z = workspace.s_z;
    br_uint_32 xm = workspace.xm;
    br_uint_32 xm_f = workspace.xm_f;
    br_uint_32 minor = *minor_x;
    br_uint_32 scan = workspace.scanAddress;
    br_uint_32 zscan = workspace.depthAddress;

    do {
        const br_uint_32 x = minor >> 16;
        br_int_32 n = (br_int_32)((xm >> 16) - x);

        if (dir == DRAW_LR ? n <= 0 : n >= 0) {
            br_uint_8* const line = colour + (br_uint_32)(scan + x);
            br_uint_16* const zline = (br_uint_16*)(depth + (br_uint_32)(zscan + 2 * x));
            br_uint_32 z = ror16(s_z);
            br_uint_32 carry = 0;

            for (;;) {
                const br_uint_32 prev = z;
                if (dir == DRAW_LR) {
                    z += carry;
                    carry = z < carry;
                } else {
                    const br_uint_32 borrow = z < carry;
                    z -= carry;
                    carry = borrow;
                }
                if (z <= ((prev & 0xffff0000u) | zline[n])) {
                    zline[n] = (br_uint_16)z;
                    line[n] = pixel;
                }
                if (dir == DRAW_LR) {
                    z += d_z_x;
                    carry = z < d_z_x;
                    if (++n > 0) {
                        break;
                    }
                } else {
                    carry = z < d_z_x;
                    z -= d_z_x;
                    if (--n < 0) {
                        break;
                    }
                }
            }
        }

        xm_f += d_xm_f;
        {
            const br_uint_32 dzy = xm_f < d_xm_f ? workspace.d_z_y_1 : workspace.d_z_y_0;
            s_z += dzy;
            s_z += s_z < dzy;
        }
        scan += colour_stride;
        zscan += depth_stride;
        minor += d_minor;
        xm += d_xm;
    } while (--count >= 0);

    workspace.s_z = s_z;
    workspace.xm = xm;
    workspace.xm_f = xm_f;
    workspace.scanAddress = scan;
    workspace.depthAddress = zscan;
    *minor_x = minor;
    *half_count = count;
}

void FastDraw_Z_I8_D16(int right_to_left) {
    if (right_to_left) {
        draw_z(&workspace.x1, &workspace.d_x1, &workspace.topCount, DRAW_RL);
        draw_z(&workspace.x2, &workspace.d_x2, &workspace.bottomCount, DRAW_RL);
    } else {
        draw_z(&workspace.x1, &workspace.d_x1, &workspace.topCount, DRAW_LR);
        draw_z(&workspace.x2, &workspace.d_x2, &workspace.bottomCount, DRAW_LR);
    }
}

/*
 * Z buffered, Gouraud shaded (index interpolation): one trapezium. Mirrors
 * TRAPEZIUM_ZI_I8_D16 (fti8_piz.c) including the x86 carry chain: the
 * intensity is pre-stepped back one pixel per scanline, and each pixel adds
 * the z carry left by the previous one.
 */
static inline __attribute__((always_inline)) void trapezium_zi(br_int_32* half_count, br_uint_32* half_i, const br_uint_32* half_d_i, const int dir) {
    br_int_32 count = *half_count;

    if (count < 0) {
        return;
    }

    br_uint_8* const colour = work.colour.base;
    br_uint_8* const depth = work.depth.base;
    const br_int_32 colour_stride = work.colour.stride_b;
    const br_int_32 depth_stride = work.depth.stride_b;

    const br_uint_32 d_i_x = workspace.d_i_x;
    const br_uint_32 d_z_x = workspace.d_z_x;
    const br_uint_32 main_d = workspace.d_xm;
    const br_uint_32 minor_d = *half_d_i;
    const br_uint_32 main_d_f = work.main.d_f;

    br_uint_32 s_i = workspace.s_i;
    br_uint_32 s_z = workspace.s_z;
    br_uint_32 main_i = workspace.xm;
    br_uint_32 minor = *half_i;
    br_uint_32 main_f = work.main.f;
    br_uint_32 scan = workspace.scanAddress;
    br_uint_32 zscan = workspace.depthAddress;

    br_uint_32 x_minor = minor >> 16;
    br_uint_32 x_main = main_i >> 16;

    for (;;) {
        br_int_32 n = (br_int_32)(x_main - x_minor);

        if (dir == DIR_F ? n <= 0 : n >= 0) {
            br_uint_8* const line = colour + (br_uint_32)(scan + x_minor);
            br_uint_16* const zline = (br_uint_16*)(depth + (br_uint_32)(zscan + 2 * x_minor));
            br_uint_32 i = ror16(s_i);
            br_uint_32 z = ror16(s_z);
            br_uint_32 carry, t;

            // step the intensity back one pixel ("sub/sbb"), the flags carry on into the loop
            if (dir == DIR_F) {
                carry = i < d_i_x;
                i -= d_i_x;
                t = i;
                i = t - carry;
                carry = t < carry;
            } else {
                i += d_i_x;
                carry = i < d_i_x;
                t = i + carry;
                carry = t < i;
                i = t;
            }

            for (;;) {
                const br_uint_32 prev = z;
                // z carry from the previous pixel
                if (dir == DIR_F) {
                    z += carry;
                } else {
                    z -= carry;
                }
                // intensity step
                if (dir == DIR_F) {
                    i += d_i_x;
                    carry = i < d_i_x;
                    i += carry;
                } else {
                    carry = i < d_i_x;
                    i -= d_i_x;
                    i -= carry;
                }
                if (z <= ((prev & 0xffff0000u) | zline[n])) {
                    zline[n] = (br_uint_16)z;
                    line[n] = (br_uint_8)i;
                }
                if (dir == DIR_F) {
                    z += d_z_x;
                    carry = z < d_z_x;
                    if (++n > 0) {
                        break;
                    }
                } else {
                    carry = z < d_z_x;
                    z -= d_z_x;
                    if (--n < 0) {
                        break;
                    }
                }
            }
        }

        scan += colour_stride;
        zscan += depth_stride;
        main_i += main_d;
        minor += minor_d;
        x_main = main_i >> 16;
        x_minor = minor >> 16;
        main_f += main_d_f;
        if (main_f < main_d_f) {
            s_i += workspace.d_i_y_1;
            s_z += workspace.d_z_y_1;
        } else {
            s_i += workspace.d_i_y_0;
            s_z += workspace.d_z_y_0;
        }
        if (--count < 0) {
            break;
        }
    }

    workspace.scanAddress = scan;
    workspace.depthAddress = zscan;
    workspace.xm = main_i;
    workspace.s_i = s_i;
    workspace.s_z = s_z;
    work.main.f = main_f;
    *half_i = minor;
    *half_count = count;
}

void FastTrapezium_ZI_I8_D16(int right_to_left) {
    if (right_to_left) {
        trapezium_zi(&workspace.topCount, &workspace.x1, &workspace.d_x1, DIR_B);
        trapezium_zi(&workspace.bottomCount, &workspace.x2, &workspace.d_x2, DIR_B);
    } else {
        trapezium_zi(&workspace.topCount, &workspace.x1, &workspace.d_x1, DIR_F);
        trapezium_zi(&workspace.bottomCount, &workspace.x2, &workspace.d_x2, DIR_F);
    }
}
