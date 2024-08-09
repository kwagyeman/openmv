/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2024 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2024 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Debayering Functions
 */
#include "bayer.h"

pixformat_t imlib_bayer_shift(pixformat_t pixfmt, int x, int y, bool transpose) {
    bool shift_right = x % 2;
    bool shift_down = y % 2;

    switch (pixfmt) {
        case PIXFORMAT_BAYER_BGGR: {
            if (shift_right && shift_down) {
                return PIXFORMAT_BAYER_RGGB;
            } else if (shift_right) {
                return transpose ? PIXFORMAT_BAYER_GRBG : PIXFORMAT_BAYER_GBRG;
            } else if (shift_down) {
                return transpose ? PIXFORMAT_BAYER_GBRG : PIXFORMAT_BAYER_GRBG;
            }
        }
        case PIXFORMAT_BAYER_GBRG: {
            if (shift_right && shift_down) {
                return transpose ? PIXFORMAT_BAYER_GBRG : PIXFORMAT_BAYER_GRBG;
            } else if (shift_right) {
                return PIXFORMAT_BAYER_BGGR;
            } else if (shift_down) {
                return PIXFORMAT_BAYER_RGGB;
            }
        }
        case PIXFORMAT_BAYER_GRBG: {
            if (shift_right && shift_down) {
                return transpose ? PIXFORMAT_BAYER_GRBG : PIXFORMAT_BAYER_GBRG;
            } else if (shift_right) {
                return PIXFORMAT_BAYER_RGGB;
            } else if (shift_down) {
                return PIXFORMAT_BAYER_BGGR;
            }
        }
        case PIXFORMAT_BAYER_RGGB: {
            if (shift_right && shift_down) {
                return PIXFORMAT_BAYER_BGGR;
            } else if (shift_right) {
                return transpose ? PIXFORMAT_BAYER_GBRG : PIXFORMAT_BAYER_GRBG;
            } else if (shift_down) {
                return transpose ? PIXFORMAT_BAYER_GRBG : PIXFORMAT_BAYER_GBRG;
            }
        }
        default: {
            return pixfmt;
        }
    }
}

void imlib_debayer_line(int x_start, int x_end, int y_row, void *dst_row_ptr, pixformat_t pixfmt, image_t *src) {
    int src_w = src->w, w_limit = src_w - 1, w_limit_m_1 = w_limit - 1;
    int src_h = src->h, h_limit = src_h - 1, h_limit_m_1 = h_limit - 1;

    int y_row_odd = y_row & 1;
    int y = (y_row / 2) * 2;
    uint8_t *rowptr_grgr_0, *rowptr_bgbg_1, *rowptr_grgr_2, *rowptr_bgbg_3;

    // keep row pointers in bounds
    if (y == 0) {
        rowptr_bgbg_1 = src->data;
        rowptr_grgr_2 = rowptr_bgbg_1 + ((src_h >= 2) ? src_w : 0);
        rowptr_bgbg_3 = rowptr_bgbg_1 + ((src_h >= 3) ? (src_w * 2) : 0);
        rowptr_grgr_0 = rowptr_grgr_2;
    } else if (y == h_limit_m_1) {
        rowptr_grgr_0 = src->data + ((y - 1) * src_w);
        rowptr_bgbg_1 = rowptr_grgr_0 + src_w;
        rowptr_grgr_2 = rowptr_bgbg_1 + src_w;
        rowptr_bgbg_3 = rowptr_bgbg_1;
    } else if (y >= h_limit) {
        rowptr_grgr_0 = src->data + ((y - 1) * src_w);
        rowptr_bgbg_1 = rowptr_grgr_0 + src_w;
        rowptr_grgr_2 = rowptr_grgr_0;
        rowptr_bgbg_3 = rowptr_bgbg_1;
    } else {
        // get 4 neighboring rows
        rowptr_grgr_0 = src->data + ((y - 1) * src_w);
        rowptr_bgbg_1 = rowptr_grgr_0 + src_w;
        rowptr_grgr_2 = rowptr_bgbg_1 + src_w;
        rowptr_bgbg_3 = rowptr_grgr_2 + src_w;
    }

    // If the image is an odd width this will go for the last loop and we drop the last column.
    if (!y_row_odd) {
        // even
        for (int x = x_start, i = 0; x < x_end; x += 2, i += 2) {
            uint32_t row_grgr_0, row_bgbg_1, row_grgr_2;

            // keep pixels in bounds
            if (x == 0) {
                if (src_w >= 4) {
                    row_grgr_0 = *((uint32_t *) rowptr_grgr_0);
                    row_bgbg_1 = *((uint32_t *) rowptr_bgbg_1);
                    row_grgr_2 = *((uint32_t *) rowptr_grgr_2);
                } else if (src_w >= 3) {
                    row_grgr_0 = *((uint16_t *) rowptr_grgr_0) | (*(rowptr_grgr_0 + 2) << 16);
                    row_bgbg_1 = *((uint16_t *) rowptr_bgbg_1) | (*(rowptr_bgbg_1 + 2) << 16);
                    row_grgr_2 = *((uint16_t *) rowptr_grgr_2) | (*(rowptr_grgr_2 + 2) << 16);
                } else if (src_w >= 2) {
                    row_grgr_0 = *((uint16_t *) rowptr_grgr_0);
                    row_grgr_0 = (row_grgr_0 << 16) | row_grgr_0;
                    row_bgbg_1 = *((uint16_t *) rowptr_bgbg_1);
                    row_bgbg_1 = (row_bgbg_1 << 16) | row_bgbg_1;
                    row_grgr_2 = *((uint16_t *) rowptr_grgr_2);
                    row_grgr_2 = (row_grgr_2 << 16) | row_grgr_2;
                } else {
                    row_grgr_0 = *(rowptr_grgr_0) * 0x01010101;
                    row_bgbg_1 = *(rowptr_bgbg_1) * 0x01010101;
                    row_grgr_2 = *(rowptr_grgr_2) * 0x01010101;
                }
                // The starting point needs to be offset by 1. The below patterns are actually
                // rgrg, gbgb, rgrg, and gbgb. So, shift left and backfill the missing border pixel.
                row_grgr_0 = (row_grgr_0 << 8) | __UXTB_RORn(row_grgr_0, 8);
                row_bgbg_1 = (row_bgbg_1 << 8) | __UXTB_RORn(row_bgbg_1, 8);
                row_grgr_2 = (row_grgr_2 << 8) | __UXTB_RORn(row_grgr_2, 8);
            } else if (x == w_limit_m_1) {
                row_grgr_0 = *((uint32_t *) (rowptr_grgr_0 + x - 2));
                row_grgr_0 = (row_grgr_0 >> 8) | ((row_grgr_0 << 8) & 0xff000000);
                row_bgbg_1 = *((uint32_t *) (rowptr_bgbg_1 + x - 2));
                row_bgbg_1 = (row_bgbg_1 >> 8) | ((row_bgbg_1 << 8) & 0xff000000);
                row_grgr_2 = *((uint32_t *) (rowptr_grgr_2 + x - 2));
                row_grgr_2 = (row_grgr_2 >> 8) | ((row_grgr_2 << 8) & 0xff000000);
            } else if (x >= w_limit) {
                row_grgr_0 = *((uint16_t *) (rowptr_grgr_0 + x - 1));
                row_grgr_0 = (row_grgr_0 << 16) | row_grgr_0;
                row_bgbg_1 = *((uint16_t *) (rowptr_bgbg_1 + x - 1));
                row_bgbg_1 = (row_bgbg_1 << 16) | row_bgbg_1;
                row_grgr_2 = *((uint16_t *) (rowptr_grgr_2 + x - 1));
                row_grgr_2 = (row_grgr_2 << 16) | row_grgr_2;
            } else {
                // get 4 neighboring rows
                row_grgr_0 = *((uint32_t *) (rowptr_grgr_0 + x - 1));
                row_bgbg_1 = *((uint32_t *) (rowptr_bgbg_1 + x - 1));
                row_grgr_2 = *((uint32_t *) (rowptr_grgr_2 + x - 1));
            }

            int r_pixels_0, g_pixels_0, b_pixels_0;

            switch (src->pixfmt) {
                case PIXFORMAT_BAYER_BGGR: {
                    #if defined(ARM_MATH_DSP)
                    int row_02 = __UHADD8(row_grgr_0, row_grgr_2);
                    int row_1g = __UHADD8(row_bgbg_1, __PKHTB(row_bgbg_1, row_bgbg_1, 16));

                    r_pixels_0 = __UXTB16(__UHADD8(row_02, __PKHTB(row_02, row_02, 16)));
                    g_pixels_0 = __UXTB16(__UHADD8(row_1g, __PKHTB(row_1g, row_02, 8)));
                    b_pixels_0 = __UXTB16_RORn(__UHADD8(row_bgbg_1, __PKHBT(row_bgbg_1, row_bgbg_1, 16)), 8);
                    #else

                    int r0 = ((row_grgr_0 & 0xFF) + (row_grgr_2 & 0xFF)) >> 1;
                    int r2 = (((row_grgr_0 >> 16) & 0xFF) + ((row_grgr_2 >> 16) & 0xFF)) >> 1;
                    r_pixels_0 = (r2 << 16) | ((r0 + r2) >> 1);

                    int g0 = (row_grgr_0 >> 8) & 0xFF;
                    int g1 = (((row_bgbg_1 >> 16) & 0xFF) + (row_bgbg_1 & 0xFF)) >> 1;
                    int g2 = (row_grgr_2 >> 8) & 0xFF;
                    g_pixels_0 = (row_bgbg_1 & 0xFF0000) | ((((g0 + g2) >> 1) + g1) >> 1);

                    int b1 = (((row_bgbg_1 >> 24) & 0xFF) + ((row_bgbg_1 >> 8) & 0xFF)) >> 1;
                    b_pixels_0 = (b1 << 16) | ((row_bgbg_1 >> 8) & 0xFF);

                    #endif
                    break;
                }
                case PIXFORMAT_BAYER_GBRG: {
                    #if defined(ARM_MATH_DSP)
                    int row_02 = __UHADD8(row_grgr_0, row_grgr_2);
                    int row_1g = __UHADD8(row_bgbg_1, __PKHBT(row_bgbg_1, row_bgbg_1, 16));

                    r_pixels_0 = __UXTB16_RORn(__UHADD8(row_02, __PKHBT(row_02, row_02, 16)), 8);
                    g_pixels_0 = __UXTB16_RORn(__UHADD8(row_1g, __PKHBT(row_1g, row_02, 8)), 8);
                    b_pixels_0 = __UXTB16(__UHADD8(row_bgbg_1, __PKHTB(row_bgbg_1, row_bgbg_1, 16)));
                    #else

                    int r0 = (((row_grgr_0 >> 8) & 0xFF) + ((row_grgr_2 >> 8) & 0xFF)) >> 1;
                    int r2 = (((row_grgr_0 >> 24) & 0xFF) + ((row_grgr_2 >> 24) & 0xFF)) >> 1;
                    r_pixels_0 = r0 | (((r0 + r2) >> 1) << 16);

                    int g0 = (row_grgr_0 >> 16) & 0xFF;
                    int g1 = (((row_bgbg_1 >> 24) & 0xFF) + ((row_bgbg_1 >> 8) & 0xFF)) >> 1;
                    int g2 = (row_grgr_2 >> 16) & 0xFF;
                    g_pixels_0 = ((row_bgbg_1 >> 8) & 0xFF) | (((((g0 + g2) >> 1) + g1) >> 1) << 16);

                    int b1 = (((row_bgbg_1 >> 16) & 0xFF) + (row_bgbg_1 & 0xFF)) >> 1;
                    b_pixels_0 = b1 | (row_bgbg_1 & 0xFF0000);

                    #endif
                    break;
                }
                case PIXFORMAT_BAYER_GRBG: {
                    #if defined(ARM_MATH_DSP)
                    int row_02 = __UHADD8(row_grgr_0, row_grgr_2);
                    int row_1g = __UHADD8(row_bgbg_1, __PKHBT(row_bgbg_1, row_bgbg_1, 16));

                    r_pixels_0 = __UXTB16(__UHADD8(row_bgbg_1, __PKHTB(row_bgbg_1, row_bgbg_1, 16)));
                    g_pixels_0 = __UXTB16_RORn(__UHADD8(row_1g, __PKHBT(row_1g, row_02, 8)), 8);
                    b_pixels_0 = __UXTB16_RORn(__UHADD8(row_02, __PKHBT(row_02, row_02, 16)), 8);
                    #else

                    int r1 = (((row_bgbg_1 >> 16) & 0xFF) + (row_bgbg_1 & 0xFF)) >> 1;
                    r_pixels_0 = r1 | (row_bgbg_1 & 0xFF0000);

                    int g0 = (row_grgr_0 >> 16) & 0xFF;
                    int g1 = (((row_bgbg_1 >> 24) & 0xFF) + ((row_bgbg_1 >> 8) & 0xFF)) >> 1;
                    int g2 = (row_grgr_2 >> 16) & 0xFF;
                    g_pixels_0 = ((row_bgbg_1 >> 8) & 0xFF) | (((((g0 + g2) >> 1) + g1) >> 1) << 16);

                    int b0 = (((row_grgr_0 >> 8) & 0xFF) + ((row_grgr_2 >> 8) & 0xFF)) >> 1;
                    int b2 = (((row_grgr_0 >> 24) & 0xFF) + ((row_grgr_2 >> 24) & 0xFF)) >> 1;
                    b_pixels_0 = b0 | (((b0 + b2) >> 1) << 16);

                    #endif
                    break;
                }
                case PIXFORMAT_BAYER_RGGB: {
                    #if defined(ARM_MATH_DSP)
                    int row_02 = __UHADD8(row_grgr_0, row_grgr_2);
                    int row_1g = __UHADD8(row_bgbg_1, __PKHTB(row_bgbg_1, row_bgbg_1, 16));

                    r_pixels_0 = __UXTB16_RORn(__UHADD8(row_bgbg_1, __PKHBT(row_bgbg_1, row_bgbg_1, 16)), 8);
                    g_pixels_0 = __UXTB16(__UHADD8(row_1g, __PKHTB(row_1g, row_02, 8)));
                    b_pixels_0 = __UXTB16(__UHADD8(row_02, __PKHTB(row_02, row_02, 16)));
                    #else

                    int r1 = (((row_bgbg_1 >> 24) & 0xFF) + ((row_bgbg_1 >> 8) & 0xFF)) >> 1;
                    r_pixels_0 = (r1 << 16) | ((row_bgbg_1 >> 8) & 0xFF);

                    int g0 = (row_grgr_0 >> 8) & 0xFF;
                    int g1 = (((row_bgbg_1 >> 16) & 0xFF) + (row_bgbg_1 & 0xFF)) >> 1;
                    int g2 = (row_grgr_2 >> 8) & 0xFF;
                    g_pixels_0 = (row_bgbg_1 & 0xFF0000) | ((((g0 + g2) >> 1) + g1) >> 1);

                    int b0 = ((row_grgr_0 & 0xFF) + (row_grgr_2 & 0xFF)) >> 1;
                    int b2 = (((row_grgr_0 >> 16) & 0xFF) + ((row_grgr_2 >> 16) & 0xFF)) >> 1;
                    b_pixels_0 = (b2 << 16) | ((b0 + b2) >> 1);

                    #endif
                    break;
                }
                default: {
                    r_pixels_0 = 0;
                    g_pixels_0 = 0;
                    b_pixels_0 = 0;
                    break;
                }
            }

            switch (pixfmt) {
                case PIXFORMAT_BINARY: {
                    uint32_t *dst_row_ptr_32 = (uint32_t *) dst_row_ptr;
                    int y0 = ((r_pixels_0 * 38) + (g_pixels_0 * 75) + (b_pixels_0 * 15)) >> 7;
                    IMAGE_PUT_BINARY_PIXEL_FAST(dst_row_ptr_32, i, (y0 >> 7));

                    if (x != w_limit) {
                        IMAGE_PUT_BINARY_PIXEL_FAST(dst_row_ptr_32, i + 1, (y0 >> 23));
                    }

                    break;
                }
                case PIXFORMAT_GRAYSCALE: {
                    uint8_t *dst_row_ptr_8 = (uint8_t *) dst_row_ptr;
                    int y0 = ((r_pixels_0 * 38) + (g_pixels_0 * 75) + (b_pixels_0 * 15)) >> 7;
                    IMAGE_PUT_GRAYSCALE_PIXEL_FAST(dst_row_ptr_8, i, y0);

                    if (x != w_limit) {
                        IMAGE_PUT_GRAYSCALE_PIXEL_FAST(dst_row_ptr_8, i + 1, y0 >> 16);
                    }

                    break;
                }
                case PIXFORMAT_RGB565: {
                    uint16_t *dst_row_ptr_16 = (uint16_t *) dst_row_ptr;
                    int rgb565_0 = ((r_pixels_0 << 8) & 0xf800f800) |
                                   ((g_pixels_0 << 3) & 0x07e007e0) |
                                   ((b_pixels_0 >> 3) & 0x001f001f);

                    if (x == w_limit) {
                        // just put bottom
                        IMAGE_PUT_RGB565_PIXEL_FAST(dst_row_ptr_16, i, rgb565_0);
                    } else {
                        // put both
                        *((uint32_t *) (dst_row_ptr_16 + i)) = rgb565_0;
                    }

                    break;
                }
                default: {
                    break;
                }
            }
        }
    } else {
        // odd
        for (int x = x_start, i = 0; x < x_end; x += 2, i += 2) {
            uint32_t row_bgbg_1, row_grgr_2, row_bgbg_3;

            // keep pixels in bounds
            if (x == 0) {
                if (src_w >= 4) {
                    row_bgbg_1 = *((uint32_t *) rowptr_bgbg_1);
                    row_grgr_2 = *((uint32_t *) rowptr_grgr_2);
                    row_bgbg_3 = *((uint32_t *) rowptr_bgbg_3);
                } else if (src_w >= 3) {
                    row_bgbg_1 = *((uint16_t *) rowptr_bgbg_1) | (*(rowptr_bgbg_1 + 2) << 16);
                    row_grgr_2 = *((uint16_t *) rowptr_grgr_2) | (*(rowptr_grgr_2 + 2) << 16);
                    row_bgbg_3 = *((uint16_t *) rowptr_bgbg_3) | (*(rowptr_bgbg_3 + 2) << 16);
                } else if (src_w >= 2) {
                    row_bgbg_1 = *((uint16_t *) rowptr_bgbg_1);
                    row_bgbg_1 = (row_bgbg_1 << 16) | row_bgbg_1;
                    row_grgr_2 = *((uint16_t *) rowptr_grgr_2);
                    row_grgr_2 = (row_grgr_2 << 16) | row_grgr_2;
                    row_bgbg_3 = *((uint16_t *) rowptr_bgbg_3);
                    row_bgbg_3 = (row_bgbg_3 << 16) | row_bgbg_3;
                } else {
                    row_bgbg_1 = *(rowptr_bgbg_1) * 0x01010101;
                    row_grgr_2 = *(rowptr_grgr_2) * 0x01010101;
                    row_bgbg_3 = *(rowptr_bgbg_3) * 0x01010101;
                }
                // The starting point needs to be offset by 1. The below patterns are actually
                // rgrg, gbgb, rgrg, and gbgb. So, shift left and backfill the missing border pixel.
                row_bgbg_1 = (row_bgbg_1 << 8) | __UXTB_RORn(row_bgbg_1, 8);
                row_grgr_2 = (row_grgr_2 << 8) | __UXTB_RORn(row_grgr_2, 8);
                row_bgbg_3 = (row_bgbg_3 << 8) | __UXTB_RORn(row_bgbg_3, 8);
            } else if (x == w_limit_m_1) {
                row_bgbg_1 = *((uint32_t *) (rowptr_bgbg_1 + x - 2));
                row_bgbg_1 = (row_bgbg_1 >> 8) | ((row_bgbg_1 << 8) & 0xff000000);
                row_grgr_2 = *((uint32_t *) (rowptr_grgr_2 + x - 2));
                row_grgr_2 = (row_grgr_2 >> 8) | ((row_grgr_2 << 8) & 0xff000000);
                row_bgbg_3 = *((uint32_t *) (rowptr_bgbg_3 + x - 2));
                row_bgbg_3 = (row_bgbg_3 >> 8) | ((row_bgbg_1 << 8) & 0xff000000);
            } else if (x >= w_limit) {
                row_bgbg_1 = *((uint16_t *) (rowptr_bgbg_1 + x - 1));
                row_bgbg_1 = (row_bgbg_1 << 16) | row_bgbg_1;
                row_grgr_2 = *((uint16_t *) (rowptr_grgr_2 + x - 1));
                row_grgr_2 = (row_grgr_2 << 16) | row_grgr_2;
                row_bgbg_3 = *((uint16_t *) (rowptr_bgbg_3 + x - 1));
                row_bgbg_3 = (row_bgbg_3 << 16) | row_bgbg_3;
            } else {
                // get 4 neighboring rows
                row_bgbg_1 = *((uint32_t *) (rowptr_bgbg_1 + x - 1));
                row_grgr_2 = *((uint32_t *) (rowptr_grgr_2 + x - 1));
                row_bgbg_3 = *((uint32_t *) (rowptr_bgbg_3 + x - 1));
            }

            int r_pixels_1, g_pixels_1, b_pixels_1;

            switch (src->pixfmt) {
                case PIXFORMAT_BAYER_BGGR: {
                    #if defined(ARM_MATH_DSP)
                    int row_13 = __UHADD8(row_bgbg_1, row_bgbg_3);
                    int row_2g = __UHADD8(row_grgr_2, __PKHBT(row_grgr_2, row_grgr_2, 16));

                    r_pixels_1 = __UXTB16(__UHADD8(row_grgr_2, __PKHTB(row_grgr_2, row_grgr_2, 16)));
                    g_pixels_1 = __UXTB16_RORn(__UHADD8(row_2g, __PKHBT(row_2g, row_13, 8)), 8);
                    b_pixels_1 = __UXTB16_RORn(__UHADD8(row_13, __PKHBT(row_13, row_13, 16)), 8);
                    #else

                    int r2 = (((row_grgr_2 >> 16) & 0xFF) + (row_grgr_2 & 0xFF)) >> 1;
                    r_pixels_1 = (row_grgr_2 & 0xFF0000) | r2;

                    int g1 = (row_bgbg_1 >> 16) & 0xFF;
                    int g2 = (((row_grgr_2 >> 24) & 0xFF) + ((row_grgr_2 >> 8) & 0xFF)) >> 1;
                    int g3 = (row_bgbg_3 >> 16) & 0xFF;
                    g_pixels_1 = (((((g1 + g3) >> 1) + g2) >> 1) << 16) | ((row_grgr_2 >> 8) & 0xFF);

                    int b1 = (((row_bgbg_1 >> 8) & 0xFF) + ((row_bgbg_3 >> 8) & 0xFF)) >> 1;
                    int b3 = (((row_bgbg_1 >> 24) & 0xFF) + ((row_bgbg_3 >> 24) & 0xFF)) >> 1;
                    b_pixels_1 = (((b1 + b3) >> 1) << 16) | b1;

                    #endif
                    break;
                }
                case PIXFORMAT_BAYER_GBRG: {
                    #if defined(ARM_MATH_DSP)
                    int row_13 = __UHADD8(row_bgbg_1, row_bgbg_3);
                    int row_2g = __UHADD8(row_grgr_2, __PKHTB(row_grgr_2, row_grgr_2, 16));

                    r_pixels_1 = __UXTB16_RORn(__UHADD8(row_grgr_2, __PKHBT(row_grgr_2, row_grgr_2, 16)), 8);
                    g_pixels_1 = __UXTB16(__UHADD8(row_2g, __PKHTB(row_2g, row_13, 8)));
                    b_pixels_1 = __UXTB16(__UHADD8(row_13, __PKHTB(row_13, row_13, 16)));
                    #else

                    int r2 = (((row_grgr_2 >> 24) & 0xFF) + ((row_grgr_2 >> 8) & 0xFF)) >> 1;
                    r_pixels_1 = ((row_grgr_2 >> 8) & 0xFF) | (r2 << 16);

                    int g1 = (row_bgbg_1 >> 8) & 0xFF;
                    int g2 = (((row_grgr_2 >> 16) & 0xFF) + (row_grgr_2 & 0xFF)) >> 1;
                    int g3 = (row_bgbg_3 >> 8) & 0xFF;
                    g_pixels_1 = ((((g1 + g3) >> 1) + g2) >> 1) | (row_grgr_2 & 0xFF0000);

                    int b1 = ((row_bgbg_1 & 0xFF) + (row_bgbg_3 & 0xFF)) >> 1;
                    int b3 = (((row_bgbg_1 >> 16) & 0xFF) + ((row_bgbg_3 >> 16) & 0xFF)) >> 1;
                    b_pixels_1 = ((b1 + b3) >> 1) | (b3 << 16);

                    #endif
                    break;
                }
                case PIXFORMAT_BAYER_GRBG: {
                    #if defined(ARM_MATH_DSP)
                    int row_13 = __UHADD8(row_bgbg_1, row_bgbg_3);
                    int row_2g = __UHADD8(row_grgr_2, __PKHTB(row_grgr_2, row_grgr_2, 16));

                    r_pixels_1 = __UXTB16(__UHADD8(row_13, __PKHTB(row_13, row_13, 16)));
                    g_pixels_1 = __UXTB16(__UHADD8(row_2g, __PKHTB(row_2g, row_13, 8)));
                    b_pixels_1 = __UXTB16_RORn(__UHADD8(row_grgr_2, __PKHBT(row_grgr_2, row_grgr_2, 16)), 8);
                    #else

                    int r1 = ((row_bgbg_1 & 0xFF) + (row_bgbg_3 & 0xFF)) >> 1;
                    int r3 = (((row_bgbg_1 >> 16) & 0xFF) + ((row_bgbg_3 >> 16) & 0xFF)) >> 1;
                    r_pixels_1 = ((r1 + r3) >> 1) | (r3 << 16);

                    int g1 = (row_bgbg_1 >> 8) & 0xFF;
                    int g2 = (((row_grgr_2 >> 16) & 0xFF) + (row_grgr_2 & 0xFF)) >> 1;
                    int g3 = (row_bgbg_3 >> 8) & 0xFF;
                    g_pixels_1 = ((((g1 + g3) >> 1) + g2) >> 1) | (row_grgr_2 & 0xFF0000);

                    int b2 = (((row_grgr_2 >> 24) & 0xFF) + ((row_grgr_2 >> 8) & 0xFF)) >> 1;
                    b_pixels_1 = ((row_grgr_2 >> 8) & 0xFF) | (b2 << 16);

                    #endif
                    break;
                }
                case PIXFORMAT_BAYER_RGGB: {
                    #if defined(ARM_MATH_DSP)
                    int row_13 = __UHADD8(row_bgbg_1, row_bgbg_3);
                    int row_2g = __UHADD8(row_grgr_2, __PKHBT(row_grgr_2, row_grgr_2, 16));

                    r_pixels_1 = __UXTB16_RORn(__UHADD8(row_13, __PKHBT(row_13, row_13, 16)), 8);
                    g_pixels_1 = __UXTB16_RORn(__UHADD8(row_2g, __PKHBT(row_2g, row_13, 8)), 8);
                    b_pixels_1 = __UXTB16(__UHADD8(row_grgr_2, __PKHTB(row_grgr_2, row_grgr_2, 16)));
                    #else

                    int r1 = (((row_bgbg_1 >> 8) & 0xFF) + ((row_bgbg_3 >> 8) & 0xFF)) >> 1;
                    int r3 = (((row_bgbg_1 >> 24) & 0xFF) + ((row_bgbg_3 >> 24) & 0xFF)) >> 1;
                    r_pixels_1 = (((r1 + r3) >> 1) << 16) | r1;

                    int g1 = (row_bgbg_1 >> 16) & 0xFF;
                    int g2 = (((row_grgr_2 >> 24) & 0xFF) + ((row_grgr_2 >> 8) & 0xFF)) >> 1;
                    int g3 = (row_bgbg_3 >> 16) & 0xFF;
                    g_pixels_1 = (((((g1 + g3) >> 1) + g2) >> 1) << 16) | ((row_grgr_2 >> 8) & 0xFF);

                    int b2 = (((row_grgr_2 >> 16) & 0xFF) + (row_grgr_2 & 0xFF)) >> 1;
                    b_pixels_1 = (row_grgr_2 & 0xFF0000) | b2;

                    #endif
                    break;
                }
                default: {
                    r_pixels_1 = 0;
                    g_pixels_1 = 0;
                    b_pixels_1 = 0;
                    break;
                }
            }

            switch (pixfmt) {
                case PIXFORMAT_BINARY: {
                    uint32_t *dst_row_ptr_32 = (uint32_t *) dst_row_ptr;
                    int y1 = ((r_pixels_1 * 38) + (g_pixels_1 * 75) + (b_pixels_1 * 15)) >> 7;
                    IMAGE_PUT_BINARY_PIXEL_FAST(dst_row_ptr_32, i, (y1 >> 7));

                    if (x != w_limit) {
                        IMAGE_PUT_BINARY_PIXEL_FAST(dst_row_ptr_32, i + 1, (y1 >> 23));
                    }

                    break;
                }
                case PIXFORMAT_GRAYSCALE: {
                    uint8_t *dst_row_ptr_8 = (uint8_t *) dst_row_ptr;
                    int y1 = ((r_pixels_1 * 38) + (g_pixels_1 * 75) + (b_pixels_1 * 15)) >> 7;
                    IMAGE_PUT_GRAYSCALE_PIXEL_FAST(dst_row_ptr_8, i, y1);

                    if (x != w_limit) {
                        IMAGE_PUT_GRAYSCALE_PIXEL_FAST(dst_row_ptr_8, i + 1, y1 >> 16);
                    }

                    break;
                }
                case PIXFORMAT_RGB565: {
                    uint16_t *dst_row_ptr_16 = (uint16_t *) dst_row_ptr;
                    int rgb565_1 = ((r_pixels_1 << 8) & 0xf800f800) |
                                   ((g_pixels_1 << 3) & 0x07e007e0) |
                                   ((b_pixels_1 >> 3) & 0x001f001f);

                    if (x == w_limit) {
                        // just put bottom
                        IMAGE_PUT_RGB565_PIXEL_FAST(dst_row_ptr_16, i, rgb565_1);
                    } else {
                        // put both
                        *((uint32_t *) (dst_row_ptr_16 + i)) = rgb565_1;
                    }

                    break;
                }
                default: {
                    break;
                }
            }
        }
    }
}

// Loads pixels from the image into the 4 row vectors and handles the boundary conditions.
// Note that the loaded pixels are shifted to the right by 1 to account for the offset
// created by debayering the image.
void vdebayer_load_rows_inner(uint8_t **rowptrs, uint32_t x, uint32_t n, v128_t *row) {
    if (x != 0) {
        // Start loading 1 pixel behind the x position and load 1 extra pixel.
        row[0] = vldr_u8(rowptrs[0] + x - 1, n + 1);
        row[1] = vldr_u8(rowptrs[1] + x - 1, n + 1);
        row[2] = vldr_u8(rowptrs[2] + x - 1, n + 1);
        row[3] = vldr_u8(rowptrs[3] + x - 1, n + 1);
        // Handle boundary conditions.
        if ((n % 4) == ((UINT8_VECTOR_SIZE - 1) % 4)) {
            // MSB [0, G1, R0, G0] LSB -> MSB [R0, G1, R0, G0] LSB
            row[0] = vset_u8(row[0], n, vget_u8(row[0], n - 2));
            row[1] = vset_u8(row[1], n, vget_u8(row[1], n - 2));
            row[2] = vset_u8(row[2], n, vget_u8(row[2], n - 2));
            row[3] = vset_u8(row[3], n, vget_u8(row[3], n - 2));
        } else if ((n % 4) == ((UINT8_VECTOR_SIZE - 2) % 4)) {
            // MSB [0, 0, R0, G0] LSB -> MSB [R0, G0, R0, G0] LSB
            row[0] = vset_u16(row[0], n / 2, vget_u16(row[0], (n / 2) - 1));
            row[1] = vset_u16(row[1], n / 2, vget_u16(row[1], (n / 2) - 1));
            row[2] = vset_u16(row[2], n / 2, vget_u16(row[2], (n / 2) - 1));
            row[3] = vset_u16(row[3], n / 2, vget_u16(row[3], (n / 2) - 1));
        }
    } else {
        row[0] = vldr_u8(rowptrs[0], n);
        row[1] = vldr_u8(rowptrs[1], n);
        row[2] = vldr_u8(rowptrs[2], n);
        row[3] = vldr_u8(rowptrs[3], n);
        // Handle boundary conditions.
        if ((n % 4) == ((UINT8_VECTOR_SIZE - 1) % 4)) {
            // MSB [0, G1, R0, G0] LSB -> MSB [R0, G1, R0, G0] LSB
            row[0] = vset_u8(row[0], n, vget_u8(row[0], n - 2));
            row[1] = vset_u8(row[1], n, vget_u8(row[1], n - 2));
            row[2] = vset_u8(row[2], n, vget_u8(row[2], n - 2));
            row[3] = vset_u8(row[3], n, vget_u8(row[3], n - 2));
        } else if ((n % 4) == ((UINT8_VECTOR_SIZE - 2) % 4)) {
            // MSB [0, 0, R0, G0] LSB -> MSB [R0, G0, R0, G0] LSB
            row[0] = vset_u16(row[0], n / 2, vget_u16(row[0], (n / 2) - 1));
            row[1] = vset_u16(row[1], n / 2, vget_u16(row[1], (n / 2) - 1));
            row[2] = vset_u16(row[2], n / 2, vget_u16(row[2], (n / 2) - 1));
            row[3] = vset_u16(row[3], n / 2, vget_u16(row[3], (n / 2) - 1));
        } else if ((n % 4) == ((UINT8_VECTOR_SIZE - 3) % 4)) {
            // MSB [0, 0, 0, G0] LSB -> MSB [G0, G0, G0, G0] LSB
            row[0] = vset_u32(row[0], n / 4, vget_u8(row[0], n - 1) * 0x01010101);
            row[1] = vset_u32(row[1], n / 4, vget_u8(row[1], n - 1) * 0x01010101);
            row[2] = vset_u32(row[2], n / 4, vget_u8(row[2], n - 1) * 0x01010101);
            row[3] = vset_u32(row[3], n / 4, vget_u8(row[3], n - 1) * 0x01010101);
        }
        // Shift loaded pixels left by 1 as if we started loading at x - 1.
        row[0] = vshlc(row[0], 8);
        row[1] = vshlc(row[1], 8);
        row[2] = vshlc(row[2], 8);
        row[3] = vshlc(row[3], 8);
        // MSB [G1, R0, G0, 0] LSB -> MSB [G1, R0, G0, R0] LSB
        row[0] = vset_u8(row[0], 0, vget_u8(row[0], 2));
        row[1] = vset_u8(row[1], 0, vget_u8(row[1], 2));
        row[2] = vset_u8(row[2], 0, vget_u8(row[2], 2));
        row[3] = vset_u8(row[3], 0, vget_u8(row[3], 2));
    }
}

void vdebayer_bggr_to_binary(image_t *dst, image_t *src) {
    for (uint32_t y = 0; y < src->h; y += 2) {
        uint8_t *rowptrs[4];
        vdebayer_rowptrs_init(src, y, rowptrs);
        uint32_t *p0 = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(dst, y);
        uint32_t *p1 = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(dst, y + 1);

        for (uint32_t x = 0; x < src->w; x += (UINT8_VECTOR_SIZE - 2)) {
            uint32_t n = IM_MIN(((uint32_t) (src->w - x)), UINT8_VECTOR_SIZE);
            v128_t rows[4], r_pixels, g_pixels, b_pixels;
            vdebayer_load_rows(rowptrs, x, n, rows);
            n = IM_MIN(n, (UINT8_VECTOR_SIZE - 2));

            vdebayer_bggr(rows[0], rows[1], rows[2], &r_pixels, &g_pixels, &b_pixels);
            vdebayer_store_binary(p0, x, n, r_pixels, g_pixels, b_pixels);

            if (y == (src->h - 1)) {
                continue;
            }

            // PIXFORMAT_BAYER_BGGR shifted down by 1 becomes PIXFORMAT_BAYER_GRBG
            vdebayer_grbg(rows[1], rows[2], rows[3], &r_pixels, &g_pixels, &b_pixels);
            vdebayer_store_binary(p1, x, n, r_pixels, g_pixels, b_pixels);
        }
    }   
}

void vdebayer_gbrg_to_binary(image_t *dst, image_t *src) {
    for (uint32_t y = 0; y < src->h; y += 2) {
        uint8_t *rowptrs[4];
        vdebayer_rowptrs_init(src, y, rowptrs);
        uint32_t *p0 = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(dst, y);
        uint32_t *p1 = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(dst, y + 1);

        for (uint32_t x = 0; x < src->w; x += (UINT8_VECTOR_SIZE - 2)) {
            uint32_t n = IM_MIN(((uint32_t) (src->w - x)), UINT8_VECTOR_SIZE);
            v128_t rows[4], r_pixels, g_pixels, b_pixels;
            vdebayer_load_rows(rowptrs, x, n, rows);
            n = IM_MIN(n, (UINT8_VECTOR_SIZE - 2));

            vdebayer_gbrg(rows[0], rows[1], rows[2], &r_pixels, &g_pixels, &b_pixels);
            vdebayer_store_binary(p0, x, n, r_pixels, g_pixels, b_pixels);

            if (y == (src->h - 1)) {
                continue;
            }

            // PIXFORMAT_BAYER_GBRG shifted down by 1 becomes PIXFORMAT_BAYER_RGGB
            vdebayer_rggb(rows[1], rows[2], rows[3], &r_pixels, &g_pixels, &b_pixels);
            vdebayer_store_binary(p1, x, n, r_pixels, g_pixels, b_pixels);
        }
    }   
}

void vdebayer_grbg_to_binary(image_t *dst, image_t *src) {
    for (uint32_t y = 0; y < src->h; y += 2) {
        uint8_t *rowptrs[4];
        vdebayer_rowptrs_init(src, y, rowptrs);
        uint32_t *p0 = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(dst, y);
        uint32_t *p1 = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(dst, y + 1);

        for (uint32_t x = 0; x < src->w; x += (UINT8_VECTOR_SIZE - 2)) {
            uint32_t n = IM_MIN(((uint32_t) (src->w - x)), UINT8_VECTOR_SIZE);
            v128_t rows[4], r_pixels, g_pixels, b_pixels;
            vdebayer_load_rows(rowptrs, x, n, rows);
            n = IM_MIN(n, (UINT8_VECTOR_SIZE - 2));

            vdebayer_grbg(rows[0], rows[1], rows[2], &r_pixels, &g_pixels, &b_pixels);
            vdebayer_store_binary(p0, x, n, r_pixels, g_pixels, b_pixels);

            if (y == (src->h - 1)) {
                continue;
            }

            // PIXFORMAT_BAYER_GRBG shifted down by 1 becomes PIXFORMAT_BAYER_BGGR
            vdebayer_bggr(rows[1], rows[2], rows[3], &r_pixels, &g_pixels, &b_pixels);
            vdebayer_store_binary(p1, x, n, r_pixels, g_pixels, b_pixels);
        }
    }   
}

void vdebayer_rggb_to_binary(image_t *dst, image_t *src) {
    for (uint32_t y = 0; y < src->h; y += 2) {
        uint8_t *rowptrs[4];
        vdebayer_rowptrs_init(src, y, rowptrs);
        uint32_t *p0 = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(dst, y);
        uint32_t *p1 = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(dst, y + 1);

        for (uint32_t x = 0; x < src->w; x += (UINT8_VECTOR_SIZE - 2)) {
            uint32_t n = IM_MIN(((uint32_t) (src->w - x)), UINT8_VECTOR_SIZE);
            v128_t rows[4], r_pixels, g_pixels, b_pixels;
            vdebayer_load_rows(rowptrs, x, n, rows);
            n = IM_MIN(n, (UINT8_VECTOR_SIZE - 2));

            vdebayer_rggb(rows[0], rows[1], rows[2], &r_pixels, &g_pixels, &b_pixels);
            vdebayer_store_binary(p0, x, n, r_pixels, g_pixels, b_pixels);

            if (y == (src->h - 1)) {
                continue;
            }

            // PIXFORMAT_BAYER_RGGB shifted down by 1 becomes PIXFORMAT_BAYER_GBRG
            vdebayer_gbrg(rows[1], rows[2], rows[3], &r_pixels, &g_pixels, &b_pixels);
            vdebayer_store_binary(p1, x, n, r_pixels, g_pixels, b_pixels);
        }
    }   
}

void vdebayer_bggr_to_grayscale(image_t *dst, image_t *src) {
    for (uint32_t y = 0; y < src->h; y += 2) {
        uint8_t *rowptrs[4];
        vdebayer_rowptrs_init(src, y, rowptrs);
        uint8_t *p0 = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(dst, y);
        uint8_t *p1 = p0 + dst->w;

        for (uint32_t x = 0; x < src->w; x += (UINT8_VECTOR_SIZE - 2)) {
            uint32_t n = IM_MIN(((uint32_t) (src->w - x)), UINT8_VECTOR_SIZE);
            v128_t rows[4], r_pixels, g_pixels, b_pixels;
            vdebayer_load_rows(rowptrs, x, n, rows);
            n = IM_MIN(n, (UINT8_VECTOR_SIZE - 2));

            vdebayer_bggr(rows[0], rows[1], rows[2], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16_narrow_u8(p0 + x, vdebayer_to_grayscale(r_pixels, g_pixels, b_pixels), n);

            if (y == (src->h - 1)) {
                continue;
            }

            // PIXFORMAT_BAYER_BGGR shifted down by 1 becomes PIXFORMAT_BAYER_GRBG
            vdebayer_grbg(rows[1], rows[2], rows[3], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16_narrow_u8(p1 + x, vdebayer_to_grayscale(r_pixels, g_pixels, b_pixels), n);
        }
    }   
}

void vdebayer_gbrg_to_grayscale(image_t *dst, image_t *src) {
    for (uint32_t y = 0; y < src->h; y += 2) {
        uint8_t *rowptrs[4];
        vdebayer_rowptrs_init(src, y, rowptrs);
        uint8_t *p0 = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(dst, y);
        uint8_t *p1 = p0 + dst->w;

        for (uint32_t x = 0; x < src->w; x += (UINT8_VECTOR_SIZE - 2)) {
            uint32_t n = IM_MIN(((uint32_t) (src->w - x)), UINT8_VECTOR_SIZE);
            v128_t rows[4], r_pixels, g_pixels, b_pixels;
            vdebayer_load_rows(rowptrs, x, n, rows);
            n = IM_MIN(n, (UINT8_VECTOR_SIZE - 2));

            vdebayer_gbrg(rows[0], rows[1], rows[2], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16_narrow_u8(p0 + x, vdebayer_to_grayscale(r_pixels, g_pixels, b_pixels), n);

            if (y == (src->h - 1)) {
                continue;
            }

            // PIXFORMAT_BAYER_GBRG shifted down by 1 becomes PIXFORMAT_BAYER_RGGB
            vdebayer_rggb(rows[1], rows[2], rows[3], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16_narrow_u8(p1 + x, vdebayer_to_grayscale(r_pixels, g_pixels, b_pixels), n);
        }
    }   
}

void vdebayer_grbg_to_grayscale(image_t *dst, image_t *src) {
    for (uint32_t y = 0; y < src->h; y += 2) {
        uint8_t *rowptrs[4];
        vdebayer_rowptrs_init(src, y, rowptrs);
        uint8_t *p0 = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(dst, y);
        uint8_t *p1 = p0 + dst->w;

        for (uint32_t x = 0; x < src->w; x += (UINT8_VECTOR_SIZE - 2)) {
            uint32_t n = IM_MIN(((uint32_t) (src->w - x)), UINT8_VECTOR_SIZE);
            v128_t rows[4], r_pixels, g_pixels, b_pixels;
            vdebayer_load_rows(rowptrs, x, n, rows);
            n = IM_MIN(n, (UINT8_VECTOR_SIZE - 2));

            vdebayer_grbg(rows[0], rows[1], rows[2], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16_narrow_u8(p0 + x, vdebayer_to_grayscale(r_pixels, g_pixels, b_pixels), n);

            if (y == (src->h - 1)) {
                continue;
            }

            // PIXFORMAT_BAYER_GRBG shifted down by 1 becomes PIXFORMAT_BAYER_BGGR
            vdebayer_bggr(rows[1], rows[2], rows[3], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16_narrow_u8(p1 + x, vdebayer_to_grayscale(r_pixels, g_pixels, b_pixels), n);
        }
    }   
}

void vdebayer_rggb_to_grayscale(image_t *dst, image_t *src) {
    for (uint32_t y = 0; y < src->h; y += 2) {
        uint8_t *rowptrs[4];
        vdebayer_rowptrs_init(src, y, rowptrs);
        uint8_t *p0 = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(dst, y);
        uint8_t *p1 = p0 + dst->w;

        for (uint32_t x = 0; x < src->w; x += (UINT8_VECTOR_SIZE - 2)) {
            uint32_t n = IM_MIN(((uint32_t) (src->w - x)), UINT8_VECTOR_SIZE);
            v128_t rows[4], r_pixels, g_pixels, b_pixels;
            vdebayer_load_rows(rowptrs, x, n, rows);
            n = IM_MIN(n, (UINT8_VECTOR_SIZE - 2));

            vdebayer_rggb(rows[0], rows[1], rows[2], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16_narrow_u8(p0 + x, vdebayer_to_grayscale(r_pixels, g_pixels, b_pixels), n);

            if (y == (src->h - 1)) {
                continue;
            }

            // PIXFORMAT_BAYER_RGGB shifted down by 1 becomes PIXFORMAT_BAYER_GBRG
            vdebayer_gbrg(rows[1], rows[2], rows[3], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16_narrow_u8(p1 + x, vdebayer_to_grayscale(r_pixels, g_pixels, b_pixels), n);
        }
    }   
}

void vdebayer_bggr_to_rgb565(image_t *dst, image_t *src) {
    for (uint32_t y = 0; y < src->h; y += 2) {
        uint8_t *rowptrs[4];
        vdebayer_rowptrs_init(src, y, rowptrs);
        uint16_t *p0 = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(dst, y);
        uint16_t *p1 = p0 + dst->w;

        for (uint32_t x = 0; x < src->w; x += (UINT8_VECTOR_SIZE - 2)) {
            uint32_t n = IM_MIN(((uint32_t) (src->w - x)), UINT8_VECTOR_SIZE);
            v128_t rows[4], r_pixels, g_pixels, b_pixels;
            vdebayer_load_rows(rowptrs, x, n, rows);
            n = IM_MIN(n, (UINT8_VECTOR_SIZE - 2));

            vdebayer_bggr(rows[0], rows[1], rows[2], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16(p0 + x, vdebayer_to_rgb565(r_pixels, g_pixels, b_pixels), n);

            if (y == (src->h - 1)) {
                continue;
            }

            // PIXFORMAT_BAYER_BGGR shifted down by 1 becomes PIXFORMAT_BAYER_GRBG
            vdebayer_grbg(rows[1], rows[2], rows[3], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16(p1 + x, vdebayer_to_rgb565(r_pixels, g_pixels, b_pixels), n);
        }
    }   
}

void vdebayer_gbrg_to_rgb565(image_t *dst, image_t *src) {
    for (uint32_t y = 0; y < src->h; y += 2) {
        uint8_t *rowptrs[4];
        vdebayer_rowptrs_init(src, y, rowptrs);
        uint16_t *p0 = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(dst, y);
        uint16_t *p1 = p0 + dst->w;

        for (uint32_t x = 0; x < src->w; x += (UINT8_VECTOR_SIZE - 2)) {
            uint32_t n = IM_MIN(((uint32_t) (src->w - x)), UINT8_VECTOR_SIZE);
            v128_t rows[4], r_pixels, g_pixels, b_pixels;
            vdebayer_load_rows(rowptrs, x, n, rows);
            n = IM_MIN(n, (UINT8_VECTOR_SIZE - 2));

            vdebayer_gbrg(rows[0], rows[1], rows[2], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16(p0 + x, vdebayer_to_rgb565(r_pixels, g_pixels, b_pixels), n);

            if (y == (src->h - 1)) {
                continue;
            }

            // PIXFORMAT_BAYER_GBRG shifted down by 1 becomes PIXFORMAT_BAYER_RGGB
            vdebayer_rggb(rows[1], rows[2], rows[3], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16(p1 + x, vdebayer_to_rgb565(r_pixels, g_pixels, b_pixels), n);
        }
    }   
}

void vdebayer_grbg_to_rgb565(image_t *dst, image_t *src) {
    for (uint32_t y = 0; y < src->h; y += 2) {
        uint8_t *rowptrs[4];
        vdebayer_rowptrs_init(src, y, rowptrs);
        uint16_t *p0 = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(dst, y);
        uint16_t *p1 = p0 + dst->w;

        for (uint32_t x = 0; x < src->w; x += (UINT8_VECTOR_SIZE - 2)) {
            uint32_t n = IM_MIN(((uint32_t) (src->w - x)), UINT8_VECTOR_SIZE);
            v128_t rows[4], r_pixels, g_pixels, b_pixels;
            vdebayer_load_rows(rowptrs, x, n, rows);
            n = IM_MIN(n, (UINT8_VECTOR_SIZE - 2));

            vdebayer_grbg(rows[0], rows[1], rows[2], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16(p0 + x, vdebayer_to_rgb565(r_pixels, g_pixels, b_pixels), n);

            if (y == (src->h - 1)) {
                continue;
            }

            // PIXFORMAT_BAYER_GRBG shifted down by 1 becomes PIXFORMAT_BAYER_BGGR
            vdebayer_bggr(rows[1], rows[2], rows[3], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16(p1 + x, vdebayer_to_rgb565(r_pixels, g_pixels, b_pixels), n);
        }
    }   
}

void vdebayer_rggb_to_rgb565(image_t *dst, image_t *src) {
    for (uint32_t y = 0; y < src->h; y += 2) {
        uint8_t *rowptrs[4];
        vdebayer_rowptrs_init(src, y, rowptrs);
        uint16_t *p0 = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(dst, y);
        uint16_t *p1 = p0 + dst->w;

        for (uint32_t x = 0; x < src->w; x += (UINT8_VECTOR_SIZE - 2)) {
            uint32_t n = IM_MIN(((uint32_t) (src->w - x)), UINT8_VECTOR_SIZE);
            v128_t rows[4], r_pixels, g_pixels, b_pixels;
            vdebayer_load_rows(rowptrs, x, n, rows);
            n = IM_MIN(n, (UINT8_VECTOR_SIZE - 2));

            vdebayer_rggb(rows[0], rows[1], rows[2], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16(p0 + x, vdebayer_to_rgb565(r_pixels, g_pixels, b_pixels), n);

            if (y == (src->h - 1)) {
                continue;
            }

            // PIXFORMAT_BAYER_RGGB shifted down by 1 becomes PIXFORMAT_BAYER_GBRG
            vdebayer_gbrg(rows[1], rows[2], rows[3], &r_pixels, &g_pixels, &b_pixels);
            vstr_u16(p1 + x, vdebayer_to_rgb565(r_pixels, g_pixels, b_pixels), n);
        }
    }   
}

// Does no bounds checking on the destination. Destination must be mutable.
void imlib_debayer_image(image_t *dst, image_t *src) {
    switch (src->pixfmt) {
        case PIXFORMAT_BAYER_BGGR: {
            switch (dst->pixfmt) {
                case PIXFORMAT_BINARY: {
                    vdebayer_bggr_to_binary(dst, src);
                    break;
                }
                case PIXFORMAT_GRAYSCALE: {
                    vdebayer_bggr_to_grayscale(dst, src);
                    break;
                }
                case PIXFORMAT_RGB565: {
                    vdebayer_bggr_to_rgb565(dst, src);
                    break;
                }
                default: {
                    break;
                }
            }
            break;
        }
        case PIXFORMAT_BAYER_GBRG: {
            switch (dst->pixfmt) {
                case PIXFORMAT_BINARY: {
                    vdebayer_gbrg_to_binary(dst, src);
                    break;
                }
                case PIXFORMAT_GRAYSCALE: {
                    vdebayer_gbrg_to_grayscale(dst, src);
                    break;
                }
                case PIXFORMAT_RGB565: {
                    vdebayer_gbrg_to_rgb565(dst, src);
                    break;
                }
                default: {
                    break;
                }
            }
            break;
        }
        case PIXFORMAT_BAYER_GRBG: {
            switch (dst->pixfmt) {
                case PIXFORMAT_BINARY: {
                    vdebayer_grbg_to_binary(dst, src);
                    break;
                }
                case PIXFORMAT_GRAYSCALE: {
                    vdebayer_grbg_to_grayscale(dst, src);
                    break;
                }
                case PIXFORMAT_RGB565: {
                    vdebayer_grbg_to_rgb565(dst, src);
                    break;
                }
                default: {
                    break;
                }
            }
            break;
        }
        case PIXFORMAT_BAYER_RGGB: {
            switch (dst->pixfmt) {
                case PIXFORMAT_BINARY: {
                    vdebayer_rggb_to_binary(dst, src);
                    break;
                }
                case PIXFORMAT_GRAYSCALE: {
                    vdebayer_rggb_to_grayscale(dst, src);
                    break;
                }
                case PIXFORMAT_RGB565: {
                    vdebayer_rggb_to_rgb565(dst, src);
                    break;
                }
                default: {
                    break;
                }
            }
            break;
        }
        default: {
            break;
        }
    }
}
