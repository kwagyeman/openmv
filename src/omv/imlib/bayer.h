/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2024 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2024 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Bilinear debayering functions.
 */
#include "imlib.h"
#include "simd.h"

// Row vectors are loaded into memory and processed in little-endian order.
// row_0 stores MSB [G1, R1, G0, R0] LSB pixels where each pixel is 8-bits.
// row_1 stores MSB [B1, G3, B0, G2] LSB pixels where each pixel is 8-bits.
// row_2 stores MSB [G5, R3, G4, R2] LSB pixels where each pixel is 8-bits.
// In the case of vectors larger than 32-bits the pattern is repeated for every 32-bits.
//
// vdebayer_bggr produces two output pixels per 32-bits:
// r_pixels = MSB [0, R@G3, 0, R@B0] LSB pixels where each pixel is 8-bits.
// g_pixels = MSB [0, G@G3, 0, G@B0] LSB pixels where each pixel is 8-bits.
// b_pixels = MSB [0, B@G3, 0, B@B0] LSB pixels where each pixel is 8-bits.
static inline void vdebayer_bggr(v128_t row_0,
                                 v128_t row_1,
                                 v128_t row_2,
                                 v128_t *r_pixels,
                                 v128_t *g_pixels,
                                 v128_t *b_pixels) {
    v128_t row_02 = vhadd_u8(row_0, row_2);
    // row_02 = [(G1+G5)/2, (R1+R3)/2, (G0+G4)/2, (R0+R2)/2]
    v128_t row_11 = vhadd_u8(row_1, vpkhtb(row_1, row_1));
    // row_11 = [(B1+B1)/2, (G3+G3)/2, (B0+B1)/2, (G2+G3)/2]
    // row_11 = [B1, G3, (B0+B1)/2, (G2+G3)/2]
    v128_t t_r_pixels = vhadd_u8(row_02, vpkhtb(row_02, row_02));
    // t_r_pixels = [(G1+G5+G1+G5)/4, (R1+R3+R1+R3)/4, (G1+G5+G0+G4)/4, (R1+R3+R0+R2)/4]
    // t_r_pixels = [(G1+G5)/2, R@G3, G@G3, R@B0]
    *r_pixels = vuxtb16(t_r_pixels);
    // r_pixels = [0, R@G3, 0, R@B0]
    v128_t t_g_pixels = vhadd_u8(row_11, vpkhtb_8(row_11, row_02));
    // t_g_pixels = [(B1+B1)/2, (G3+G3)/2, (B0+B1+R1+R3)/4, (G2+G3+G0+G4)/4]
    // t_g_pixels = [B@B1, G@G3, (B0+B1+R1+R3)/4, G@B0]
    *g_pixels = vuxtb16(t_g_pixels);
    // g_pixels = [0, G@G3, 0, G@B0]
    v128_t t_b_pixels = vhadd_u8(row_1, vpkhbt(row_1, row_1));
    // t_b_pixels = [(B1+B0)/2, (G3+G2)/2, (B0+B0)/2, (G2+G2)/2]
    // t_b_pixels = [B@G3, (G3+G2)/2, B@B0, G@G2]
    *b_pixels = vuxtb16_8(t_b_pixels);
    // b_pixels = [0, B@G3, 0, B@B0]
}

// Row vectors are loaded into memory and processed in little-endian order.
// row_0 stores MSB [R1, G1, R0, G0] LSB pixels where each pixel is 8-bits.
// row_1 stores MSB [G3, B1, G2, B0] LSB pixels where each pixel is 8-bits.
// row_2 stores MSB [R3, G5, R2, G4] LSB pixels where each pixel is 8-bits.
// In the case of vectors larger than 32-bits the pattern is repeated for every 32-bits.
//
// vdebayer_gbrg produces two output pixels per 32-bits:
// r_pixels = MSB [0, R@B1, 0, R@G2] LSB pixels where each pixel is 8-bits.
// g_pixels = MSB [0, G@B1, 0, G@G2] LSB pixels where each pixel is 8-bits.
// b_pixels = MSB [0, B@B1, 0, B@G2] LSB pixels where each pixel is 8-bits.
static inline void vdebayer_gbrg(v128_t row_0,
                                 v128_t row_1,
                                 v128_t row_2,
                                 v128_t *r_pixels,
                                 v128_t *g_pixels,
                                 v128_t *b_pixels) {
    v128_t row_02 = vhadd_u8(row_0, row_2);
    // row_02 = [(R1+R3)/2, (G1+G5)/2, (R0+R2)/2, (G0+G4)/2]
    v128_t row_11 = vhadd_u8(row_1, vpkhbt(row_1, row_1));
    // row_11 = [(G3+G2)/2, (B1+B0)/2, (G2+G2)/2, (B0+B0)/2]
    // row_11 = [(G3+G2)/2, B@G2, G@G2, B@B0]
    v128_t t_r_pixels = vhadd_u8(row_02, vpkhbt(row_02, row_02));
    // t_r_pixels = [(R1+R3+R0+R2)/4, (G1+G5+G0+G4)/4, (R0+R2+R0+R2)/4, (G0+G4+G0+G4)/4]
    // t_r_pixels = [R@B1, (G1+G5+G0+G4)/4, R@G2, (G0+G4)/2]
    *r_pixels = vuxtb16_8(t_r_pixels);
    // r_pixels = [0, R@B1, 0, R@G2]
    v128_t t_g_pixels = vhadd_u8(row_11, vpkhbt_8(row_11, row_02));
    // t_g_pixels = [(G3+G2+G1+G5)/4, (B1+B0+R0+R2)/4, (G2+G2+G2+G2)/4, (B0+B0+B0+B0)/2]
    // t_g_pixels = [G@B1, (B1+B0+R0+R2)/4, G@G2, B@B0]
    *g_pixels = vuxtb16_8(t_g_pixels);
    // g_pixels = [0, G@B1, 0, G@G2]
    v128_t t_b_pixels = vhadd_u8(row_1, vpkhtb(row_1, row_1));
    // t_b_pixels = [(G3+G3)/2, (B1+B1)/2, (G2+G3)/2, (B0+B1)/2]
    // t_b_pixels = [(G3+G3)/2, B@B1, (G2+G3)/2, B@G2]
    *b_pixels = vuxtb16(t_b_pixels);
    // b_pixels = [0, B@B1, 0, B@G2]
}

// Row vectors are loaded into memory and processed in little-endian order.
// row_0 stores MSB [B1, G1, B0, G0] LSB pixels where each pixel is 8-bits.
// row_1 stores MSB [G3, R3, G2, R0] LSB pixels where each pixel is 8-bits.
// row_2 stores MSB [B3, G5, B2, G4] LSB pixels where each pixel is 8-bits.
// In the case of vectors larger than 32-bits the pattern is repeated for every 32-bits.
//
// vdebayer_grbg produces two output pixels per 32-bits:
// r_pixels = MSB [0, R@R3, 0, R@G2] LSB pixels where each pixel is 8-bits.
// g_pixels = MSB [0, G@R3, 0, G@G2] LSB pixels where each pixel is 8-bits.
// b_pixels = MSB [0, B@R3, 0, B@G2] LSB pixels where each pixel is 8-bits.
static inline void vdebayer_grbg(v128_t row_0,
                                 v128_t row_1,
                                 v128_t row_2,
                                 v128_t *r_pixels,
                                 v128_t *g_pixels,
                                 v128_t *b_pixels) {
    v128_t row_02 = vhadd_u8(row_0, row_2);
    // row_02 = [(B1+B3)/2, (G1+G5)/2, (B0+B2)/2, (G0+G4)/2]
    v128_t row_11 = vhadd_u8(row_1, vpkhbt(row_1, row_1));
    // row_11 = [(G3+G2)/2, (R3+R0)/2, (G2+G2)/2, (R0+R0)/2]
    // row_11 = [(G3+G2)/2, R@G2, G@G2, R@R0]
    v128_t t_r_pixels = vhadd_u8(row_1, vpkhtb(row_1, row_1));
    // t_r_pixels = [(G3+G3)/2, (R3+R3)/2, (G2+G3)/2, (R0+R3)/2]
    // t_r_pixels = [(G3+G3)/2, R@R3, (G2+G3)/2, R@G2]
    *r_pixels = vuxtb16(t_r_pixels);
    // r_pixels = [0, R@R3, 0, R@G2]
    v128_t t_g_pixels = vhadd_u8(row_11, vpkhbt_8(row_11, row_02));
    // t_g_pixels = [(G3+G2+G1+G5)/4, (R3+R0+B0+B2)/4, (G2+G2+G2+G2)/4, (R0+R0+R0+R0)/2]
    // t_g_pixels = [G@R3, (R3+R0+B0+B2)/4, G@G2, R@R0]
    *g_pixels = vuxtb16_8(t_g_pixels);
    // g_pixels = [0, G@R3, 0, G@G2]
    v128_t t_b_pixels = vhadd_u8(row_02, vpkhbt(row_02, row_02));
    // t_b_pixels = [(B1+B3+B0+B2)/4, (G1+G5+G0+G4)/4, (B0+B2+B0+B2)/4, (G0+G4+G0+G4)/2]
    // t_b_pixels = [B@R3, (G1+G5+G0+G4)/4, B@G2, (G0+G4)/2]
    *b_pixels = vuxtb16_8(t_b_pixels);
    // b_pixels = [0, B@R3, 0, B@G2]
}

// Row vectors are loaded into memory and processed in little-endian order.
// row_0 stores MSB [G1, B1, G0, B0] LSB pixels where each pixel is 8-bits.
// row_1 stores MSB [R1, G3, R0, G2] LSB pixels where each pixel is 8-bits.
// row_2 stores MSB [G5, B3, G4, B2] LSB pixels where each pixel is 8-bits.
// In the case of vectors larger than 32-bits the pattern is repeated for every 32-bits.
//
// vdebayer_rggb produces two output pixels per 32-bits:
// r_pixels = MSB [0, R@G3, 0, R@R0] LSB pixels where each pixel is 8-bits.
// g_pixels = MSB [0, G@G3, 0, G@R0] LSB pixels where each pixel is 8-bits.
// b_pixels = MSB [0, B@G3, 0, B@R0] LSB pixels where each pixel is 8-bits.
static inline void vdebayer_rggb(v128_t row_0,
                                 v128_t row_1,
                                 v128_t row_2,
                                 v128_t *r_pixels,
                                 v128_t *g_pixels,
                                 v128_t *b_pixels) {
    v128_t row_02 = vhadd_u8(row_0, row_2);
    // row_02 = [(G1+G5)/2, (B1+B3)/2, (G0+G4)/2, (B0+B2)/2]
    v128_t row_11 = vhadd_u8(row_1, vpkhtb(row_1, row_1));
    // row_11 = [(R1+R1)/2, (G3+G3)/2, (R0+R1)/2, (G2+G3)/2]
    // row_11 = [R@R1, G@G3, R@G3, (G2+G3)/2]
    v128_t t_r_pixels = vhadd_u8(row_1, vpkhbt(row_1, row_1));
    // t_r_pixels = [(R1+R0)/2, (G3+G2)/2, (R0+R0)/2, (G2+G2)/2]
    // t_r_pixels = [R@G3, (G3+G2)/2, R@R0, G@G2]
    *r_pixels = vuxtb16_8(t_r_pixels);
    // r_pixels = [0, R@G3, 0, R@R0]
    v128_t t_g_pixels = vhadd_u8(row_11, vpkhtb_8(row_11, row_02));
    // t_g_pixels = [(R1+R1+R1+R1)/4, (G3+G3+G3+G3)/4, (R0+R1+B1+B3)/4, (G2+G3+G0+G4)/4]
    // t_g_pixels = [R@R1, G@G3, (R0+R1+B1+B3)/4, G@R0]
    *g_pixels = vuxtb16(t_g_pixels);
    // g_pixels = [0, G@G3, 0, G@R0]
    v128_t t_b_pixels = vhadd_u8(row_02, vpkhtb(row_02, row_02));
    // t_b_pixels = [(G1+G5+G1+G5)/4, (B1+B3+B1+B3)/4, (G0+G4+G1+G5)/4, (B0+B2+B1+B3)/4]
    // t_b_pixels = [(G1+G5)/2, B@G3, (G0+G4+G1+G5)/4, B@R0]
    *b_pixels = vuxtb16(t_b_pixels);
    // b_pixels = [0, B@G3, 0, B@R0]
}

// Note that the loaded pointers are shifted to up by 1 to account for the offset
// created by debayering the image.
static inline void vdebayer_rowptrs_init(image_t *src, uint32_t y, uint8_t **rowptrs) {
    // keep row pointers in bounds
    if (y == 0) {
        rowptrs[1] = src->data;
        rowptrs[2] = rowptrs[1] + ((src->h >= 2) ? src->w : 0);
        rowptrs[3] = rowptrs[1] + ((src->h >= 3) ? (src->w * 2) : 0);
        rowptrs[0] = rowptrs[2];
    } else if (y == (src->h - 2)) {
        rowptrs[0] = src->data + ((y - 1) * src->w);
        rowptrs[1] = rowptrs[0] + src->w;
        rowptrs[2] = rowptrs[1] + src->w;
        rowptrs[3] = rowptrs[1];
    } else if (y == (src->h - 1)) {
        rowptrs[0] = src->data + ((y - 1) * src->w);
        rowptrs[1] = rowptrs[0] + src->w;
        rowptrs[2] = rowptrs[0];
        rowptrs[3] = rowptrs[1];
    } else {
        // get 4 neighboring rows
        rowptrs[0] = src->data + ((y - 1) * src->w);
        rowptrs[1] = rowptrs[0] + src->w;
        rowptrs[2] = rowptrs[1] + src->w;
        rowptrs[3] = rowptrs[2] + src->w;
    }
}

// vdebayer_load_rows_inner() handles boundary conditions and doesn't need to be fast.
extern void vdebayer_load_rows_inner(uint8_t **rowptrs, uint32_t x, uint32_t n, v128_t *row);
static inline void vdebayer_load_rows(uint8_t **rowptrs, uint32_t x, uint32_t n, v128_t *row) {
    // For the vast majority of cases we load vector size pixels at a time and exit quickly.
    if ((x != 0) && (n == UINT8_VECTOR_SIZE)) {
        // Start loading 1 pixel behind the x position and load 1 extra pixel.
        row[0] = vldr_u8(rowptrs[0] + x - 1, UINT8_VECTOR_SIZE);
        row[1] = vldr_u8(rowptrs[1] + x - 1, UINT8_VECTOR_SIZE);
        row[2] = vldr_u8(rowptrs[2] + x - 1, UINT8_VECTOR_SIZE);
        row[3] = vldr_u8(rowptrs[3] + x - 1, UINT8_VECTOR_SIZE);
    } else {
        vdebayer_load_rows_inner(rowptrs, x, n, row);
    }
}

// In the case of vectors larger than 32-bits the pattern is repeated for every 32-bits.
//
// r_pixels = MSB [0, R1, 0, R0] LSB pixels where each pixel is 8-bits.
// g_pixels = MSB [0, G1, 0, G0] LSB pixels where each pixel is 8-bits.
// b_pixels = MSB [0, B1, 0, B0] LSB pixels where each pixel is 8-bits.
//
// Returns 2x uint8_t Grayscale (MSB [garbage, G1, garbage, G0] LSB) pixels for every 32-bits.
static inline v128_t vdebayer_to_grayscale(v128_t r_pixels,
                                           v128_t g_pixels,
                                           v128_t b_pixels) {
    r_pixels = vmul_u32(r_pixels, vdup_u32(38));
    r_pixels = vmla_u32(g_pixels, vdup_u32(75), r_pixels);
    r_pixels = vmla_u32(b_pixels, vdup_u32(15), r_pixels);
    return vlsr_u32(r_pixels, 7);
}

// In the case of vectors larger than 32-bits the pattern is repeated for every 32-bits.
//
// r_pixels = MSB [0, R1, 0, R0] LSB pixels where each pixel is 8-bits.
// g_pixels = MSB [0, G1, 0, G0] LSB pixels where each pixel is 8-bits.
// b_pixels = MSB [0, B1, 0, B0] LSB pixels where each pixel is 8-bits.
//
// Returns 2x uint16_t RGB565 (MSB [RGB1, RGB0] LSB) pixels for every 32-bits.
static inline v128_t vdebayer_to_rgb565(v128_t r_pixels,
                                        v128_t g_pixels,
                                        v128_t b_pixels) {
    r_pixels = vand_u32(vlsl_u32(r_pixels, 8), vdup_u16(0xf800));
    g_pixels = vand_u32(vlsl_u32(g_pixels, 3), vdup_u16(0x07e0));
    b_pixels = vand_u32(vlsr_u32(b_pixels, 3), vdup_u16(0x001f));
    return vorr_u32(r_pixels, vorr_u32(g_pixels, b_pixels));
}

// In the case of vectors larger than 32-bits the pattern is repeated for every 32-bits.
//
// r_pixels = MSB [0, R1, 0, R0] LSB pixels where each pixel is 8-bits.
// g_pixels = MSB [0, G1, 0, G0] LSB pixels where each pixel is 8-bits.
// b_pixels = MSB [0, B1, 0, B0] LSB pixels where each pixel is 8-bits.
//
// Stores 2x binary pixels for every 32-bits.
static inline void vdebayer_store_binary(uint32_t *p,
                                         uint32_t x,
                                         uint32_t n,
                                         v128_t r_pixels,
                                         v128_t g_pixels,
                                         v128_t b_pixels) {
    v128_t grayscale = vdebayer_to_grayscale(r_pixels, g_pixels, b_pixels);

    for (size_t i = 0; i < n; i++) {
        IMAGE_PUT_BINARY_PIXEL_FAST(p, x + i, (vget_u8(grayscale, i * 2) >> 7));
    }
}
