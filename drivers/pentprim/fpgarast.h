#ifndef PENTPRIM_FPGARAST_H
#define PENTPRIM_FPGARAST_H

/*
 * FPGA rasteriser offload (gPentprim_fpga, PENTPRIM_FPGA=1).
 *
 * Triangle setup stays on the CPU. Instead of running the pixel loops, the
 * rasteriser entry points in fastprim.c pack the prepared edge and
 * interpolant state into a command and submit it. The FPGA (or, without one,
 * the software model in fpgarast.c) walks the edges and fills colour and
 * depth.
 *
 * The software model executes commands from their words alone, with the same
 * integer arithmetic as the pixel loops in fastprim.c: it is the golden
 * reference for the RTL testbench, and PENTPRIM_VERIFY=1 checks it against
 * the original rasteriser like any other rewrite.
 *
 * Addresses
 *
 * Commands address memory with 32 bit byte offsets into the rasteriser's
 * region of the shared DDR3 memory:
 *   0x000000  command ring (hardware backend)
 *   0x100000  8 buffer windows of 1MB: colour and depth buffers. A window is
 *             bound to host memory on first use, with the first pointer seen
 *             at offset 0x40000 so sub-pixelmaps and shifted origins of the
 *             same buffer fall into the same window
 *   0x900000  textures and shade tables, allocated on first use
 *
 * Commands
 *
 * A command is a sequence of little endian 32 bit words. Word 0 is the head:
 *   [7:0] opcode, [15:8] length in words including the head, [31:16] flags
 *
 * FR_OP_TARGET  render target geometry for the following triangles
 *   w1 colour stride in bytes, w2 depth stride in bytes
 *
 * FR_OP_TRI     affine triangle (Z, ZI, ZT, ZTI of fastprim.c): two trapezia
 *               sharing the major edge xm, minor edges x1 (top) and x2
 *   flags  FR_TRI_RL, FR_TRI_I (intensity), FR_TRI_T (textured),
 *          [7:4] log2 of the texture size
 *   w1  colour address of the scan origin    w2  depth address
 *   w3  xm      w4  d_xm    w5  xm_f    w6  d_xm_f
 *   w7  x1      w8  d_x1    w9  x2      w10 d_x2
 *   w11 [15:0] topCount, [31:16] bottomCount (signed)
 *   w12 s_z     w13 d_z_x   w14 d_z_y_0 w15 d_z_y_1
 *   then, if neither FR_TRI_I nor FR_TRI_T:  colour
 *         if FR_TRI_I:  s_i, d_i_x, d_i_y_0, d_i_y_1
 *         if FR_TRI_T:  s_u, d_u_x, d_u_y_0, d_u_y_1, s_v, d_v_x, d_v_y_0,
 *                       d_v_y_1, texture address
 *         if both:      shade table address
 *
 * FR_OP_PTRI    perspective correct textured triangle (ZPT, ZPTI)
 *   flags  FR_PTRI_B (scan right to left), FR_PTRI_I (lit),
 *          [7:4] log2 of the texture size
 *   w1  colour address of the scan origin    w2  depth address
 *   w3  main edge (fraction.integer)  w4 its delta
 *   w5  x1      w6  d_x1    w7  x2      w8  d_x2
 *   w9  [15:0] topCount, [31:16] bottomCount (signed)
 *   w10 s_z     w11 d_z_y_0 w12 d_z_y_1 w13 dz per pixel (fraction.integer)
 *   w14 q       w15 q_grad  w16 q_nocarry  w17 q_carry
 *   w18 u       w19 u_grad  w20 u_nocarry
 *   w21 v       w22 v_grad  w23 v_nocarry
 *   w24 texel offset of the first scanline (v << size | u)
 *   w25 texture address
 *   then, if FR_PTRI_I: s_i, d_i_y_0, d_i_y_1, di per pixel, shade table address
 *
 * FR_OP_ATRI    affine textured triangle, texture of any width (zb8awtm.c):
 *               the edges of FR_OP_TRI, plain 16.16 depth, u as 16.16
 *               texels wrapping at the width, v as a byte offset of the
 *               texture row plus a fraction, wrapping at the texture size
 *   flags  FR_TRI_RL
 *   w1..w11 as FR_OP_TRI
 *   w12 s_z     w13 d_z_x   w14 d_z_y_0 w15 d_z_y_1
 *   w16 su      w17 dux     w18 duy0    w19 duy1
 *   w20 sv      w21 svf     w22 dvx     w23 dvxc    w24 dvxf
 *   w25 dvy0    w26 dvy0c   w27 dvy1    w28 dvy1c   w29 dvy0f   w30 dvy1f
 *   w31 u wrap (16.16)      w32 v wrap (bytes)      w33 texture size (bytes)
 *   w34 texture address
 *
 * FR_OP_FOG     depth cue or fog pass over a rectangle (DoDepthByShadeTable
 *               in the game): every pixel with a depth other than 0xFFFF and
 *               (depth - start) < near is replaced by
 *               table[((depth - start) shifted) & 0xFF00 | colour]
 *               (16 bit arithmetic). Sets the strides like FR_OP_TARGET.
 *   w1 colour address of the first row    w2 depth address of the first row
 *   w3 [15:0] pixels per row (1..1024), [31:16] rows
 *   w4 colour stride   w5 depth stride
 *   w6 [15:0] start, [31:16] near
 *   w7 shift: left if positive, right if negative
 *   w8 table address
 *
 * FR_OP_FLUSH   texture or table memory changed: forget cached copies
 *
 * FR_OP_FILL    fill a rectangle with a 16 bit pattern (depth clear, or a
 *               colour with both bytes the same): even addresses get the low
 *               byte, odd ones the high byte
 *   w1 address of the first row   w2 bytes per row (1..FR_FILL_MAX)   w3 rows
 *   w4 stride in bytes            w5 [15:0] pattern
 */

#include "brender.h"

#define FR_OP_NOP 0
#define FR_OP_TARGET 1
#define FR_OP_TRI 2
#define FR_OP_PTRI 3
#define FR_OP_FLUSH 4
#define FR_OP_FILL 5
#define FR_OP_ATRI 6
#define FR_OP_FOG 7

#define FR_HEAD(op, words, flags) ((br_uint_32)(op) | ((br_uint_32)(words) << 8) | ((br_uint_32)(flags) << 16))
#define FR_HEAD_OP(h) ((h) & 0xff)
#define FR_HEAD_WORDS(h) (((h) >> 8) & 0xff)
#define FR_HEAD_FLAGS(h) ((h) >> 16)

#define FR_TRI_RL 0x0001
#define FR_TRI_I 0x0002
#define FR_TRI_T 0x0004
#define FR_PTRI_B 0x0001
#define FR_PTRI_I 0x0002
#define FR_FLAGS_POW2(f) (((f) >> 4) & 15)

#define FR_MAX_WORDS 40

/* bytes per row of FR_OP_FILL: with any alignment within the hardware's line buffer */
#define FR_FILL_MAX 2048

#define FR_WINDOW_BASE 0x100000u
#define FR_WINDOW_SIZE 0x100000u
#define FR_WINDOW_ORIGIN 0x40000u
#define FR_WINDOWS 8
#define FR_HEAP_BASE 0x900000u
#define FR_HEAP_SIZE 0x700000u

extern int gPentprim_fpga;

/*
 * The hardware: set by the platform when the FPGA is there. Without it the
 * software model executes the commands.
 */
typedef struct {
    /* queue a command */
    void (*submit)(const br_uint_32* w, int words);
    /* return when everything submitted is drawn and in memory */
    void (*wait)(void);
    /* copy between host memory and the rasteriser's memory (nothing pending) */
    void (*upload)(br_uint_32 addr, const void* host, br_uint_32 size);
    void (*download)(br_uint_32 addr, void* host, br_uint_32 size);
} tFpgaRast_backend;

void FpgaRast_SetBackend(const tFpgaRast_backend* backend);

/*
 * Buffer ownership. With the hardware, the FPGA draws into its own copy of
 * the colour and depth buffers. Code that reads or writes those buffers
 * directly says so first: the host's copy is brought up to date, and if the
 * host may change it, the FPGA's is refreshed before it draws again.
 */
#define FR_SYNC_COLOUR 1      /* colour buffers, to read and write */
#define FR_SYNC_DEPTH_READ 2  /* depth buffers, to read */
#define FR_SYNC_DEPTH_WRITE 4 /* depth buffers, to read and write */
#define FR_SYNC_SCENE_END 8   /* (statistics) */
void FpgaRast_Sync(int flags);

/* Only read a part of a depth buffer: just that part is fetched */
void FpgaRast_SyncDepthRead(const void* first, br_uint_32 bytes);
/* ... and it was written to as well: the part goes back */
void FpgaRast_DepthWritten(const void* first, br_uint_32 bytes);

/* End of a scene (primitive library flush): the colour buffers go back to
 * the host, unless the caller holds them for more scenes of the same frame:
 * FpgaRast_Hold(1) ... scenes ... FpgaRast_Hold(0). While held, only
 * FpgaRast_Sync brings them back. */
void FpgaRast_SceneEnd(void);
void FpgaRast_Hold(int hold);

/* Fill rows of a buffer through the FPGA instead of writing them (depth
 * clear). pattern: 16 bits. Returns 0 if the caller has to do it. */
int FpgaRast_Fill(void* pixels, br_uint_32 row_bytes, br_uint_32 rows, br_uint_32 stride, br_uint_32 pattern, int depth);

/* The fog pass through the FPGA. colour, depth: first pixel of the first
 * row; table_size in bytes. Returns 0 if the caller has to do it. */
int FpgaRast_Fog(void* colour, void* depth, int width, int rows, br_uint_32 colour_stride, br_uint_32 depth_stride,
    br_uint_32 start, br_uint_32 too_near, int shift, const void* table, br_uint_32 table_size);

/* The texture or table at this address changed or was freed */
void FpgaRast_Invalidate(const void* base);

/* Emit the triangle prepared in workspace/work (called from fastprim.c) */
void FpgaRast_Tri(int flags, int pow2);
/* The arbitrary width textured triangle prepared in workspace/workspaceA */
void FpgaRast_ATri(int right_to_left);
/* Called for both trapezia of a perspective triangle, emits on the first */
void FpgaRast_PTri(int backwards, int lit, int bits);

/* Software model: execute one command */
void FpgaModel_Execute(const br_uint_32* w);

#endif
