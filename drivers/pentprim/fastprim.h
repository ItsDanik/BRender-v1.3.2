#ifndef PENTPRIM_FASTPRIM_H
#define PENTPRIM_FASTPRIM_H

/* Plain C rewrites of the hot rasteriser loops, see fastprim.c */

/* Both halves of a triangle prepared by the ZT setup (zb8p2unl.c) */
void FastDraw_ZT_I8_D16_POW2(int right_to_left, int pow2);

/* Both halves of a triangle prepared by the ZTI setup (zb8p2lit.c) */
void FastDraw_ZTI_I8_D16_POW2(int right_to_left, int pow2);

/* One trapezium (top or bottom half) of a ZPT triangle, see fti8pizp.c.
 * size: 0 = 32x32, 1 = 64x64, 2 = 128x128, 3 = 256x256 texture */
void FastTrapezium_ZPT_I8_D16(int dir, int size);

/* Both halves of a flat triangle prepared by the Z setup (zb8.c) */
void FastDraw_Z_I8_D16(int right_to_left);

/* Both trapezia of a Gouraud triangle prepared by the ZI setup (fti8_piz.c) */
void FastTrapezium_ZI_I8_D16(int right_to_left);

#endif
