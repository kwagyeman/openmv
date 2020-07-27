/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2019 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2019 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Basic drawing functions.
 */
#include "font.h"
#include "imlib.h"

void* imlib_compute_row_ptr(const image_t *img, int y) {
    switch(img->bpp) {
        case IMAGE_BPP_BINARY: {
            return IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(img, y);
        }
        case IMAGE_BPP_GRAYSCALE: {
            return IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(img, y);
        }
        case IMAGE_BPP_RGB565: {
            return IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(img, y);
        }
        default: {
            // This shouldn't happen, at least we return a valid memory block
            return img->data;
        }
    }
}

inline int imlib_get_pixel_fast(int img_bpp, const void *row_ptr, int x)
{
    switch(img_bpp) {
        case IMAGE_BPP_BINARY: {
            return IMAGE_GET_BINARY_PIXEL_FAST((uint32_t*)row_ptr, x);
        }
        case IMAGE_BPP_GRAYSCALE: {
            return IMAGE_GET_GRAYSCALE_PIXEL_FAST((uint8_t*)row_ptr, x);
        }
        case IMAGE_BPP_RGB565: {
            return IMAGE_GET_RGB565_PIXEL_FAST((uint16_t*)row_ptr, x);
        }
        default: {
            return -1;
        }
    }
}


// Set pixel (handles boundary check and image type check).
void imlib_set_pixel(image_t *img, int x, int y, int p)
{
    if ((0 <= x) && (x < img->w) && (0 <= y) && (y < img->h)) {
        switch(img->bpp) {
            case IMAGE_BPP_BINARY: {
                IMAGE_PUT_BINARY_PIXEL(img, x, y, p);
                break;
            }
            case IMAGE_BPP_GRAYSCALE: {
                IMAGE_PUT_GRAYSCALE_PIXEL(img, x, y, p);
                break;
            }
            case IMAGE_BPP_RGB565: {
                IMAGE_PUT_RGB565_PIXEL(img, x, y, p);
                break;
            }
            default: {
                break;
            }
        }
    }
}

// https://stackoverflow.com/questions/1201200/fast-algorithm-for-drawing-filled-circles
static void point_fill(image_t *img, int cx, int cy, int r0, int r1, int c)
{
    for (int y = r0; y <= r1; y++) {
        for (int x = r0; x <= r1; x++) {
            if (((x * x) + (y * y)) <= (r0 * r0)) {
                imlib_set_pixel(img, cx + x, cy + y, c);
            }
        }
    }
}

// https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C
void imlib_draw_line(image_t *img, int x0, int y0, int x1, int y1, int c, int thickness)
{
    if (thickness > 0) {
        int thickness0 = (thickness - 0) / 2;
        int thickness1 = (thickness - 1) / 2;
        int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
        int dy = abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
        int err = ((dx > dy) ? dx : -dy) / 2;

        for (;;) {
            point_fill(img, x0, y0, -thickness0, thickness1, c);
            if ((x0 == x1) && (y0 == y1)) break;
            int e2 = err;
            if (e2 > -dx) { err -= dy; x0 += sx; }
            if (e2 <  dy) { err += dx; y0 += sy; }
        }
    }
}

static void xLine(image_t *img, int x1, int x2, int y, int c)
{
    while (x1 <= x2) imlib_set_pixel(img, x1++, y, c);
}

static void yLine(image_t *img, int x, int y1, int y2, int c)
{
    while (y1 <= y2) imlib_set_pixel(img, x, y1++, c);
}

void imlib_draw_rectangle(image_t *img, int rx, int ry, int rw, int rh, int c, int thickness, bool fill)
{
    if (fill) {

        for (int y = ry, yy = ry + rh; y < yy; y++) {
            for (int x = rx, xx = rx + rw; x < xx; x++) {
                imlib_set_pixel(img, x, y, c);
            }
        }

    } else if (thickness > 0) {
        int thickness0 = (thickness - 0) / 2;
        int thickness1 = (thickness - 1) / 2;

        for (int i = rx - thickness0, j = rx + rw + thickness1, k = ry + rh - 1; i < j; i++) {
            yLine(img, i, ry - thickness0, ry + thickness1, c);
            yLine(img, i, k - thickness0, k + thickness1, c);
        }

        for (int i = ry - thickness0, j = ry + rh + thickness1, k = rx + rw - 1; i < j; i++) {
            xLine(img, rx - thickness0, rx + thickness1, i, c);
            xLine(img, k - thickness0, k + thickness1, i, c);
        }
    }
}

// https://stackoverflow.com/questions/27755514/circle-with-thickness-drawing-algorithm
void imlib_draw_circle(image_t *img, int cx, int cy, int r, int c, int thickness, bool fill)
{
    if (fill) {
        point_fill(img, cx, cy, -r, r, c);
    } else if (thickness > 0) {
        int thickness0 = (thickness - 0) / 2;
        int thickness1 = (thickness - 1) / 2;

        int xo = r + thickness0;
        int xi = IM_MAX(r - thickness1, 0);
        int xi_tmp = xi;
        int y = 0;
        int erro = 1 - xo;
        int erri = 1 - xi;

        while(xo >= y) {
            xLine(img, cx + xi, cx + xo, cy + y,  c);
            yLine(img, cx + y,  cy + xi, cy + xo, c);
            xLine(img, cx - xo, cx - xi, cy + y,  c);
            yLine(img, cx - y,  cy + xi, cy + xo, c);
            xLine(img, cx - xo, cx - xi, cy - y,  c);
            yLine(img, cx - y,  cy - xo, cy - xi, c);
            xLine(img, cx + xi, cx + xo, cy - y,  c);
            yLine(img, cx + y,  cy - xo, cy - xi, c);

            y++;

            if (erro < 0) {
                erro += 2 * y + 1;
            } else {
                xo--;
                erro += 2 * (y - xo + 1);
            }

            if (y > xi_tmp) {
                xi = y;
            } else {
                if (erri < 0) {
                    erri += 2 * y + 1;
                } else {
                    xi--;
                    erri += 2 * (y - xi + 1);
                }
            }
        }
    }
}

// https://scratch.mit.edu/projects/50039326/
static void scratch_draw_pixel(image_t *img, int x0, int y0, int dx, int dy, float shear_dx, float shear_dy, int r0, int r1, int c)
{
    point_fill(img, x0 + dx, y0 + dy + fast_floorf((dx * shear_dy) / shear_dx), r0, r1, c);
}

// https://scratch.mit.edu/projects/50039326/
static void scratch_draw_line(image_t *img, int x0, int y0, int dx, int dy0, int dy1, float shear_dx, float shear_dy, int c)
{
    int y = y0 + fast_floorf((dx * shear_dy) / shear_dx);
    yLine(img, x0 + dx, y + dy0, y + dy1, c);
}

// https://scratch.mit.edu/projects/50039326/
static void scratch_draw_sheared_ellipse(image_t *img, int x0, int y0, int width, int height, bool filled, float shear_dx, float shear_dy, int c, int thickness)
{
    int thickness0 = (thickness - 0) / 2;
    int thickness1 = (thickness - 1) / 2;
    if (((thickness > 0) || filled) && (shear_dx != 0)) {
        int a_squared = width * width;
        int four_a_squared = a_squared * 4;
        int b_squared = height * height;
        int four_b_squared = b_squared * 4;

        int x = 0;
        int y = height;
        int sigma = (2 * b_squared) + (a_squared * (1 - (2 * height)));

        while ((b_squared * x) <= (a_squared * y)) {
            if (filled) {
                scratch_draw_line(img, x0, y0, x, -y, y, shear_dx, shear_dy, c);
                scratch_draw_line(img, x0, y0, -x, -y, y, shear_dx, shear_dy, c);
            } else {
                scratch_draw_pixel(img, x0, y0, x, y, shear_dx, shear_dy, -thickness0, thickness1, c);
                scratch_draw_pixel(img, x0, y0, -x, y, shear_dx, shear_dy, -thickness0, thickness1, c);
                scratch_draw_pixel(img, x0, y0, x, -y, shear_dx, shear_dy, -thickness0, thickness1, c);
                scratch_draw_pixel(img, x0, y0, -x, -y, shear_dx, shear_dy, -thickness0, thickness1, c);
            }

            if (sigma >= 0) {
                sigma += four_a_squared * (1 - y);
                y -= 1;
            }

            sigma += b_squared * ((4 * x) + 6);
            x += 1;
        }

        x = width;
        y = 0;
        sigma = (2 * a_squared) + (b_squared * (1 - (2 * width)));

        while ((a_squared * y) <= (b_squared * x)) {
            if (filled) {
                scratch_draw_line(img, x0, y0, x, -y, y, shear_dx, shear_dy, c);
                scratch_draw_line(img, x0, y0, -x, -y, y, shear_dx, shear_dy, c);
            } else {
                scratch_draw_pixel(img, x0, y0, x, y, shear_dx, shear_dy, -thickness0, thickness1, c);
                scratch_draw_pixel(img, x0, y0, -x, y, shear_dx, shear_dy, -thickness0, thickness1, c);
                scratch_draw_pixel(img, x0, y0, x, -y, shear_dx, shear_dy, -thickness0, thickness1, c);
                scratch_draw_pixel(img, x0, y0, -x, -y, shear_dx, shear_dy, -thickness0, thickness1, c);
            }

            if (sigma >= 0) {
                sigma += four_b_squared * (1 - x);
                x -= 1;
            }

            sigma += a_squared * ((4 * y) + 6);
            y += 1;
        }
    }
}

// https://scratch.mit.edu/projects/50039326/
static void scratch_draw_rotated_ellipse(image_t *img, int x, int y, int x_axis, int y_axis, int rotation, bool filled, int c, int thickness)
{
    if ((x_axis > 0) && (y_axis > 0)) {
        if ((x_axis == y_axis) || (rotation == 0)) {
            scratch_draw_sheared_ellipse(img, x, y, x_axis / 2, y_axis / 2, filled, 1, 0, c, thickness);
        } else if (rotation == 90) {
            scratch_draw_sheared_ellipse(img, x, y, y_axis / 2, x_axis / 2, filled, 1, 0, c, thickness);
        } else {

            // Avoid rotations above 90.
            if (rotation > 90) {
                rotation -= 90;
                int temp = x_axis;
                x_axis = y_axis;
                y_axis = temp;
            }

            // Avoid rotations above 45.
            if (rotation > 45) {
                rotation -= 90;
                int temp = x_axis;
                x_axis = y_axis;
                y_axis = temp;
            }

            float theta = fast_atanf(IM_DIV(y_axis, x_axis) * (-tanf(IM_DEG2RAD(rotation))));
            float shear_dx = (x_axis * cosf(theta) * cosf(IM_DEG2RAD(rotation))) - (y_axis * sinf(theta) * sinf(IM_DEG2RAD(rotation)));
            float shear_dy = (x_axis * cosf(theta) * sinf(IM_DEG2RAD(rotation))) + (y_axis * sinf(theta) * cosf(IM_DEG2RAD(rotation)));
            float shear_x_axis = fast_fabsf(shear_dx);
            float shear_y_axis = IM_DIV((y_axis * x_axis), shear_x_axis);
            scratch_draw_sheared_ellipse(img, x, y, fast_floorf(shear_x_axis / 2), fast_floorf(shear_y_axis / 2), filled, shear_dx, shear_dy, c, thickness);
        }
    }
}

void imlib_draw_ellipse(image_t *img, int cx, int cy, int rx, int ry, int rotation, int c, int thickness, bool fill)
{
    int r = rotation % 180;
    if (r < 0) r += 180;

    scratch_draw_rotated_ellipse(img, cx, cy, rx * 2, ry * 2, r, fill, c, thickness);
}

// char rotation == 0, 90, 180, 360, etc.
// string rotation == 0, 90, 180, 360, etc.
void imlib_draw_string(image_t *img, int x_off, int y_off, const char *str, int c, float scale, int x_spacing, int y_spacing, bool mono_space,
                       int char_rotation, bool char_hmirror, bool char_vflip, int string_rotation, bool string_hmirror, bool string_vflip)
{
    char_rotation %= 360;
    if (char_rotation < 0) char_rotation += 360;
    char_rotation = (char_rotation / 90) * 90;

    string_rotation %= 360;
    if (string_rotation < 0) string_rotation += 360;
    string_rotation = (string_rotation / 90) * 90;

    bool char_swap_w_h = (char_rotation == 90) || (char_rotation == 270);
    bool char_upsidedown = (char_rotation == 180) || (char_rotation == 270);

    if (string_hmirror) x_off -= fast_floorf(font[0].w * scale) - 1;
    if (string_vflip) y_off -= fast_floorf(font[0].h * scale) - 1;

    int org_x_off = x_off;
    int org_y_off = y_off;
    const int anchor = x_off;

    for(char ch, last = '\0'; (ch = *str); str++, last = ch) {

        if ((last == '\r') && (ch == '\n')) { // handle "\r\n" strings
            continue;
        }

        if ((ch == '\n') || (ch == '\r')) { // handle '\n' or '\r' strings
            x_off = anchor;
            y_off += (string_vflip ? -1 : +1) * (fast_floorf((char_swap_w_h ? font[0].w : font[0].h) * scale) + y_spacing); // newline height == space height
            continue;
        }

        if ((ch < ' ') || (ch > '~')) { // handle unknown characters
            continue;
        }

        const glyph_t *g = &font[ch - ' '];

        if (!mono_space) {
            // Find the first pixel set and offset to that.
            bool exit = false;

            if (!char_swap_w_h) {
                for (int x = 0, xx = g->w; x < xx; x++) {
                    for (int y = 0, yy = g->h; y < yy; y++) {
                        if (g->data[(char_upsidedown ^ char_vflip) ? (g->h - 1 - y) : y] &
                            (1 << ((char_upsidedown ^ char_hmirror ^ string_hmirror) ? x : (g->w - 1 - x)))) {
                            x_off += (string_hmirror ? +1 : -1) * fast_floorf(x * scale);
                            exit = true;
                            break;
                        }
                    }

                    if (exit) break;
                }
            } else {
                for (int y = g->h - 1; y >= 0; y--) {
                    for (int x = 0, xx = g->w; x < xx; x++) {
                        if (g->data[(char_upsidedown ^ char_vflip) ? (g->h - 1 - y) : y] &
                            (1 << ((char_upsidedown ^ char_hmirror ^ string_hmirror) ? x : (g->w - 1 - x)))) {
                            x_off += (string_hmirror ? +1 : -1) * fast_floorf((g->h - 1 - y) * scale);
                            exit = true;
                            break;
                        }
                    }

                    if (exit) break;
                }
            }
        }

        for (int y = 0, yy = fast_floorf(g->h * scale); y < yy; y++) {
            for (int x = 0, xx = fast_floorf(g->w * scale); x < xx; x++) {
                if (g->data[fast_floorf(y / scale)] & (1 << (g->w - 1 - fast_floorf(x / scale)))) {
                    int16_t x_tmp = x_off + (char_hmirror ? (xx - x - 1) : x), y_tmp = y_off + (char_vflip ? (yy - y - 1) : y);
                    point_rotate(x_tmp, y_tmp, IM_DEG2RAD(char_rotation), x_off + (xx / 2), y_off + (yy / 2), &x_tmp, &y_tmp);
                    point_rotate(x_tmp, y_tmp, IM_DEG2RAD(string_rotation), org_x_off, org_y_off, &x_tmp, &y_tmp);
                    imlib_set_pixel(img, x_tmp, y_tmp, c);
                }
            }
        }

        if (mono_space) {
            x_off += (string_hmirror ? -1 : +1) * (fast_floorf((char_swap_w_h ? g->h : g->w) * scale) + x_spacing);
        } else {
            // Find the last pixel set and offset to that.
            bool exit = false;

            if (!char_swap_w_h) {
                for (int x = g->w - 1; x >= 0; x--) {
                    for (int y = g->h - 1; y >= 0; y--) {
                        if (g->data[(char_upsidedown ^ char_vflip) ? (g->h - 1 - y) : y] &
                            (1 << ((char_upsidedown ^ char_hmirror ^ string_hmirror) ? x : (g->w - 1 - x)))) {
                            x_off += (string_hmirror ? -1 : +1) * (fast_floorf((x + 2) * scale) + x_spacing);
                            exit = true;
                            break;
                        }
                    }

                    if (exit) break;
                }
            } else {
                for (int y = 0, yy = g->h; y < yy; y++) {
                    for (int x = g->w - 1; x >= 0; x--) {
                        if (g->data[(char_upsidedown ^ char_vflip) ? (g->h - 1 - y) : y] &
                            (1 << ((char_upsidedown ^ char_hmirror ^ string_hmirror) ? x : (g->w - 1 - x)))) {
                            x_off += (string_hmirror ? -1 : +1) * (fast_floorf(((g->h - 1 - y) + 2) * scale) + x_spacing);
                            exit = true;
                            break;
                        }
                    }

                    if (exit) break;
                }
            }

            if (!exit) x_off += (string_hmirror ? -1 : +1) * fast_floorf(scale * 3); // space char
        }
    }
}

#ifdef NOT_USED

static int safe_map_pixel(int dst_bpp, int src_bpp, int pixel)
{
    switch (dst_bpp) {
        case IMAGE_BPP_BINARY: {
            switch (src_bpp) {
                case IMAGE_BPP_BINARY: {
                    return pixel;
                }
                case IMAGE_BPP_GRAYSCALE: {
                    return COLOR_GRAYSCALE_TO_BINARY(pixel);
                }
                case IMAGE_BPP_RGB565: {
                    return COLOR_RGB565_TO_BINARY(pixel);
                }
                default: {
                    return 0;
                }
            }
        }
        case IMAGE_BPP_GRAYSCALE: {
            switch (src_bpp) {
                case IMAGE_BPP_BINARY: {
                    return COLOR_BINARY_TO_GRAYSCALE(pixel);
                }
                case IMAGE_BPP_GRAYSCALE: {
                    return pixel;
                }
                case IMAGE_BPP_RGB565: {
                    return COLOR_RGB565_TO_GRAYSCALE(pixel);
                }
                default: {
                    return 0;
                }
            }
        }
        case IMAGE_BPP_RGB565: {
            switch (src_bpp) {
                case IMAGE_BPP_BINARY: {
                    return COLOR_BINARY_TO_RGB565(pixel);
                }
                case IMAGE_BPP_GRAYSCALE: {
                    return COLOR_GRAYSCALE_TO_RGB565(pixel);
                }
                case IMAGE_BPP_RGB565: {
                    return pixel;
                }
                default: {
                    return 0;
                }
            }
        }
        default: {
            return 0;
        }
    }
}
#endif // NOT_USED
/**
 * Blend two RGB888 format pixels using alpha.
 * NOTE:
 *   Interpolating RGB is not a good way of blending colors as it can generate colors that aren't in the original image.
 *   It's better to blend by transforming to another color space then interpolate, but that may slow things down.
 *   We could implement a better blend at a later date using a hint like image.BLEND_USING_HSV.
 *
 * @param background_pixel Background pixel value in RGB888
 * @param foreground_pixel Foreground pixel value in RGB888
 * @param alpha Foreground alpha 0->128
 * @param alpha_complement 128 minues Foreground alpha
 * @return Blended pixel in RGB888 format
 */
uint32_t draw_blendop_rgb888(uint32_t background_pixel, uint32_t foreground_pixel, uint32_t alpha, uint32_t alpha_complement)
{
    // rrrrrrrrggggggggbbbbbbbb
    uint32_t frb = foreground_pixel & 0xFF00FF;
    // rrrrrrrr........bbbbbbbb
    uint32_t fg = (foreground_pixel >> 8) & 255;
    // ................gggggggg
    uint32_t brb = background_pixel & 0xFF00FF;
    // rrrrrrrr........bbbbbbbb
    uint32_t bg = (background_pixel >> 8) & 255;
    // ................gggggggg

    uint32_t rb = (frb * alpha + brb * alpha_complement) >> 7;
    uint32_t g = (fg * alpha + bg * alpha_complement) >> 7;

    return (rb & 0xFF00FF) + (g << 8);
}

/**
 * Scale an RGB565 format pixel returning an RGB888 result.
 *
 * @param pixel RGB565 pixel to scale.
 * @param scale Amount to scale 0->128
 * @return Scaled pixel as RGB888
 */
uint32_t draw_scaleop_RGB565_to_RGB888(uint32_t pixel, uint32_t scale)
{
    uint32_t vr = COLOR_RGB565_TO_R8(pixel);
    uint32_t vg = COLOR_RGB565_TO_G8(pixel);
    uint32_t vb = COLOR_RGB565_TO_B8(pixel);

    // Scale is 0->128 so we shift right 7
    uint32_t r = (vr * scale) >> 7;
    uint32_t g = (vg * scale) >> 7;
    uint32_t b = (vb * scale) >> 7;
    
    return (r << 16) + (g << 8) + b;
}

/**
 * Convert a pixel to binary.
 * Used by interpolation cache line methods to convert mask to bitmap.
 *
 * @param bpp Bits per pixel of pixel.
 * @param pixel Pixel value.
 * @return pixel in binary format.
 */
inline bool pixel_to_binary(int bpp, uint32_t pixel) {
    switch (bpp) {
        case IMAGE_BPP_BINARY: {
            return pixel;
        }
        case IMAGE_BPP_GRAYSCALE: {
            return COLOR_GRAYSCALE_TO_BINARY(pixel);
        }
        case IMAGE_BPP_RGB565: {
            return COLOR_RGB565_TO_BINARY(pixel);
        }
        default: {
            return false;
        }
    }
}

//
// Combine 2 image lines using an alpha value (0 = keep destination, 256 = use 100% of source)
// the source and destination bpp must be the same
//
void imlib_combine_alpha(int alpha, const uint8_t *alpha_palette, uint8_t *src, int dest_y, int x_start, int x_end, int src_bpp, image_t *dest_img)
{
uint32_t packed_alpha = (alpha << 16) | (256 - alpha);
int dest_bpp = dest_img->bpp;
    
    if (alpha == 0 || (dest_img->bpp == IMAGE_BPP_BINARY && alpha < 128)) // nothing to do
        return;
    
    if (alpha == 256) { // fully opaque, just copy
        if (src_bpp != dest_img->bpp) { // need to convert the source format to the dest
            switch (src_bpp) {
                case IMAGE_BPP_BINARY:
                    if (dest_img->bpp == IMAGE_BPP_RGB565) {
                        uint16_t *dest = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(dest_img, dest_y);
                        for (int x=x_start; x<x_end; x++) {
                            uint32_t pixel = IMAGE_GET_BINARY_PIXEL_FAST((uint32_t *)src, x);
                            uint16_t u16Pixel = (pixel) ? 0xffff : 0x0000;
                            IMAGE_PUT_RGB565_PIXEL_FAST(dest, x, u16Pixel);
                        }
                    } else { // must be grayscale
                        uint8_t *dest = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(dest_img, dest_y);
                        for (int x=x_start; x<x_end; x++) {
                            uint32_t pixel = IMAGE_GET_BINARY_PIXEL_FAST((uint32_t *)src, x);
                            uint8_t u8Pixel = (pixel) ? 0xff : 0x00;
                            IMAGE_PUT_GRAYSCALE_PIXEL_FAST(dest, x, u8Pixel);
                        }
                    }
                    break;
                case IMAGE_BPP_GRAYSCALE:
                    if (dest_img->bpp == IMAGE_BPP_RGB565) {
                        uint16_t *dest = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(dest_img, dest_y);
                        for (int x=x_start; x<x_end; x++) {
                            uint8_t pixel = IMAGE_GET_GRAYSCALE_PIXEL_FAST(src, x);
                            uint16_t u16Pixel = COLOR_GRAYSCALE_TO_RGB565(pixel);
                            IMAGE_PUT_RGB565_PIXEL_FAST(dest, x, u16Pixel);
                        }
                    } else { // must be binary
                        uint32_t *dest = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(dest_img, dest_y);
                        for (int x=x_start; x<x_end; x++) {
                            uint8_t pixel = IMAGE_GET_GRAYSCALE_PIXEL_FAST(src, x);
                            uint32_t u32Pixel = COLOR_GRAYSCALE_TO_BINARY(pixel);
                            IMAGE_PUT_BINARY_PIXEL_FAST(dest, x, u32Pixel);
                        }
                    }
                    break;
                case IMAGE_BPP_RGB565:
                    if (dest_img->bpp == IMAGE_BPP_GRAYSCALE) {
                        uint8_t *dest = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(dest_img, dest_y);
                        for (int x=x_start; x<x_end; x++) {
                            uint16_t pixel = IMAGE_GET_RGB565_PIXEL_FAST((uint16_t*)src, x);
                            uint8_t u8Pixel = RGB565_TO_Y_FAST(pixel);
                            IMAGE_PUT_GRAYSCALE_PIXEL_FAST(dest, x, u8Pixel);
                        }
                    } else { // must be binary
                        uint32_t *dest = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(dest_img, dest_y);
                        for (int x=x_start; x<x_end; x++) {
                            uint16_t pixel = IMAGE_GET_RGB565_PIXEL_FAST((uint16_t *)src, x);
                            uint32_t u32Pixel = COLOR_RGB565_TO_BINARY(pixel);
                            IMAGE_PUT_BINARY_PIXEL_FAST(dest, x, u32Pixel);
                        }
                    }
                    break;
            }
        } else { // no need to convert the pixel format, just copy them
            switch (src_bpp) {
                case IMAGE_BPP_BINARY:
                {
                    uint32_t *dest = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(dest_img, dest_y);
                    for (int x=x_start; x<x_end; x++) {
                        uint32_t pixel = IMAGE_GET_BINARY_PIXEL_FAST((uint32_t*)src, x);
                        IMAGE_PUT_BINARY_PIXEL_FAST(dest, x, pixel);
                    }
                }
                    break;
                case IMAGE_BPP_GRAYSCALE:
                {
                    uint8_t *dest = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(dest_img, dest_y);
                    memcpy(&dest[x_start], &src[x_start], (x_end - x_start));
                }
                    break;
                case IMAGE_BPP_RGB565:
                {
                    uint16_t *dest = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(dest_img, dest_y);
                    uint16_t *s = (uint16_t *)src;
                    memcpy(&dest[x_start], &s[x_start], (x_end - x_start) * 2);
                }
                    break;
            }
        }
        return;
    }
// non-opaque pixels need to be alpha-blended
    switch (src_bpp) {
        case IMAGE_BPP_BINARY: // opaque = copy (no real blending occurring)
        {
            uint32_t pixel, *dest = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(dest_img, dest_y);
            for (int x = x_start; x < x_end; x++) {
                pixel = IMAGE_GET_BINARY_PIXEL_FAST((uint32_t *)src, x);
                IMAGE_PUT_BINARY_PIXEL_FAST(dest, x, pixel);
            }
        }
        break; // binary

        case IMAGE_BPP_GRAYSCALE:
        {
            uint8_t *dest_gray = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(dest_img, dest_y);
            uint16_t *dest_rgb565 = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(dest_img, dest_y);
            uint32_t *dest_binary = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(dest_img, dest_y);
            for (int x = x_start; x < x_end; x++) {
                uint32_t pixel = ((uint32_t)src[x] << 16) | dest_gray[x];
                uint8_t u8Pixel = __SMUAD(packed_alpha, pixel) >> 8;
                if (dest_bpp == IMAGE_BPP_GRAYSCALE)
                    IMAGE_PUT_GRAYSCALE_PIXEL_FAST(dest_gray, x, u8Pixel);
                else if (dest_bpp == IMAGE_BPP_RGB565)
                    IMAGE_PUT_RGB565_PIXEL_FAST(dest_rgb565, x, COLOR_GRAYSCALE_TO_RGB565(u8Pixel));
                else
                    IMAGE_PUT_BINARY_PIXEL_FAST(dest_binary, x, COLOR_GRAYSCALE_TO_BINARY(u8Pixel));
            } // for x
        }
        break; // grayscale

        case IMAGE_BPP_RGB565:
        {
            uint32_t not_alpha, rb_src, rb_dest, g_src, g_dest, rb_mask; 
            uint16_t *s = (uint16_t *)src;
            uint16_t dest_pixel, src_pixel;
            uint8_t *dest_gray = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(dest_img, dest_y);
            uint16_t *dest_rgb565 = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(dest_img, dest_y);
            uint32_t *dest_binary = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(dest_img, dest_y);
            uint16_t u16Pixel;
            alpha >>= 3; // use a 5-bit alpha
            not_alpha = 32-alpha;
            rb_mask = 0xf81f; // split RGB565 into R_B and _G_
            for (int x = x_start; x < x_end; x++) {
                src_pixel = s[x];
                dest_pixel = dest_rgb565[x];
                src_pixel = __builtin_bswap16(src_pixel); // swap byte order
                dest_pixel = __builtin_bswap16(dest_pixel);
                rb_src = src_pixel & rb_mask;
                rb_dest = dest_pixel & rb_mask;
                g_src = src_pixel & ~rb_mask;
                g_dest = dest_pixel & ~rb_mask;
                rb_src = ((rb_src * alpha) + (rb_dest * not_alpha)) >> 5;
                g_src = ((g_src * alpha) + (g_dest * not_alpha)) >> 5;
                rb_src &= rb_mask;
                g_src &= ~rb_mask;
                u16Pixel = __builtin_bswap16(rb_src | g_src);
                if (dest_bpp == IMAGE_BPP_RGB565)
                    IMAGE_PUT_RGB565_PIXEL_FAST(dest_rgb565, x, u16Pixel);
                else if (dest_bpp == IMAGE_BPP_GRAYSCALE)
                    IMAGE_PUT_GRAYSCALE_PIXEL_FAST(dest_gray, x, COLOR_RGB565_TO_GRAYSCALE(u16Pixel));
                else
                    IMAGE_PUT_BINARY_PIXEL_FAST(dest_binary, x, COLOR_RGB565_TO_BINARY(u16Pixel));
            } // for x
        }
        break; // RGB565    
    } // switch on bpp
} /* imlib_combine_alpha() */
//
// Convert the source image through the given color palette to RGB565
// If the source is already RGB565, treat it as grayscale in RGB565 format
//
void draw_palette_rgb565(uint8_t *pDest, image_t *src_img, int y, const uint16_t *color_palette)
{
    uint16_t *d = (uint16_t *)pDest;

    // clip source y to within image bounds
    if (y < 0) y = 0;
    else if (y > src_img->h-1) y = src_img->h-1;

    switch (src_img->bpp) {
        // Since the source image only has 2 possible values, pre-load the
        // palette entries for color 0 and 255
        case IMAGE_BPP_BINARY:
            {
                uint16_t pal0, pal255, *d;
                uint32_t *row = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(src_img, y);
                pal0 = color_palette[0]; pal255 = color_palette[255]; // only 2 values
                for (int x=0; x<src_img->w; x++) {
                    *d++ = (IMAGE_GET_BINARY_PIXEL_FAST(row, x)) ? pal255 : pal0;
                } // for x
            } 
            break;
        // A grayscale source image is assumed to be the most probable case.
        // Each pixel is simply translated through the given palette
        case IMAGE_BPP_GRAYSCALE:
            {
                uint8_t *row = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(src_img, y);
                for (int x=0; x<src_img->w; x++) {
                    *d++ = color_palette[IMAGE_GET_GRAYSCALE_PIXEL_FAST(row, x)];
                } // for x
            }   
            break;
        // If the user provided a color palette and a RGB565 source image
        // we must assume that the desired behavior is to treat the RGB565 as
        // grayscale and translate it through the custom palette
        case IMAGE_BPP_RGB565:
            {
                uint16_t *row = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(src_img, y);
                for (int x=0; x<src_img->w; x++) {
                    uint8_t pixel = RGB565_TO_Y_FAST(IMAGE_GET_RGB565_PIXEL_FAST(row, x));
                    *d++ = color_palette[pixel];
                } // for x
            }
            break;
    } // switch on bytes per pixel
} /* draw_palette_rgb565() */

/**
 * Draw an image onto another image converting format if necessary.
 * 
 * @param img The image to draw onto.
 * @param other The image to draw.
 * @param x_off X offset in destination.
 * @param y_off Y offset in destination.
 * @param x_scale X scale.
 * @param y_scale Y scale.
 * @param alpha Alpha, between 0 and 256 inclusive.
 * @param mask Mask image, if interpolating must be the same size as the other image.
 * @param color_palette Color palette for transforming grayscale images to RGB565.
 * @param alpha_palette Alpha palette for masking grayscale images.
 * @param hint Rendering hint.  e.g. IMAGE_HINT_BILINEAR, IMAGE_HINT_CENTER
 */
void imlib_draw_image(image_t *dest_img, image_t *src_img, int dest_x_start, int dest_y_start, float x_scale, float y_scale, int alpha, image_t *mask, const uint16_t *color_palette, const uint8_t *alpha_palette, image_hint_t hint)
{
    int src_x_start, src_y_start, dest_x_end, dest_y_end, dest_x, dest_y;
    int bFlipX, bFlipY, delta_x, delta_y, downscale_x, downscale_y;
    uint32_t x_accum, y_accum; // fixed point fraction vars
    // For all of the scaling (nearest neighbor, bilinear and bicubic) we use a
    // 16-bit fraction instead of a floating point value. Below, we calculate an
    // increment which fits in 16-bits. We can then add this value successively as
    // we loop over the destination pixels and then shift this sum right by 16
    // to get the corresponding source pixel
    // top 16-bits = whole part, bottom 16-bits = fractional part
    bFlipX = bFlipY = 0;
    delta_x = delta_y = 1; // positive direction
    if (x_scale < 0.0f) { // flip X
        bFlipX = 1;
        delta_x = -1;
        x_scale = -x_scale;
    }
    if (y_scale < 0.0f) { // flip Y
        bFlipY = 1;
        delta_y = -1;
        y_scale = -y_scale;
    }
    uint32_t x_frac = (uint32_t)(65536.0f / x_scale); // source increment
    uint32_t y_frac = (uint32_t)(65536.0f / y_scale);
    int bytes_per_img_line = dest_img->w * sizeof(uint8_t) * 2; // (1 byte grayscale + 1 byte alpha) = * 2
    uint8_t *cache_line_1, *cache_line_2;
    uint8_t *cache_line_top;//, *cache_line_bottom;
    uint8_t *cache_convert_1, *cache_convert_2; // for using the color palette
    uint8_t *cache_convert_3, *cache_convert_4;
    int bpp = (color_palette) ? IMAGE_BPP_RGB565 : src_img->bpp;

    // Scaled src size
    int src_width_scaled = fast_floorf(x_scale * src_img->w);
    int src_height_scaled = fast_floorf(y_scale * src_img->h);

    // If alpha is 0 then nothing changes
    if (alpha == 0) return;

    if (hint & IMAGE_HINT_BILINEAR) {
        // Cannot interpolate a 1x1 pixel.
        if (src_img->w <= 1 || src_img->h <= 1) hint &= ~IMAGE_HINT_BILINEAR;
    }

    src_x_start = src_y_start = 0;

    // Center src if hint is set
    if (hint & IMAGE_HINT_CENTER) {
        dest_x_start -= src_width_scaled >> 1;
        dest_y_start -= src_height_scaled >> 1;
    }
    if (dest_x_start < 0) {
        src_x_start = (-dest_x_start * x_scale);
        dest_x_start = 0;
    }
    if (dest_y_start < 0) {
        src_y_start = (-dest_y_start * y_scale);
        dest_y_start = 0;
    }
    dest_x_end = dest_x_start + (int)((src_img->w - src_x_start) * x_scale);
    if (dest_x_end > dest_img->w)
        dest_x_end = dest_img->w;
    dest_y_end = dest_y_start + (int)((src_img->h - src_y_start) * y_scale);
    if (dest_y_end > dest_img->h)
        dest_y_end = dest_img->h;
    // prep temporary buffers
    fb_alloc_mark();
    cache_line_1 = fb_alloc(bytes_per_img_line, FB_ALLOC_NO_HINT);
    cache_line_2 = fb_alloc(bytes_per_img_line, FB_ALLOC_NO_HINT);
    cache_convert_1 = fb_alloc(bytes_per_img_line, FB_ALLOC_NO_HINT);
    cache_convert_2 = fb_alloc(bytes_per_img_line, FB_ALLOC_NO_HINT);
    cache_convert_3 = fb_alloc(bytes_per_img_line, FB_ALLOC_NO_HINT);
    cache_convert_4 = fb_alloc(bytes_per_img_line, FB_ALLOC_NO_HINT);
    y_accum = src_y_start << 16;
    dest_y = dest_y_start;
    if (bFlipY) {
        dest_y = dest_y_end-1; // start from bottom
    }
    //
    // For shrinking an image, we use the 'area' method of averaging the pixels
    // Right now this only supports integer scaling options of exactly 1/2, 1/3, etc
    //
    if (hint & IMAGE_HINT_AREA)
    {
        int acc, tx, ty, cx, cy, xsrc, ysrc;
        
        downscale_x = (int)(1/x_scale);
        downscale_y = (int)(1/y_scale);
        ysrc = src_y_start;
        for (int y = dest_y_start; y < dest_y_end; y++, dest_y += delta_y) {
            if (y & 1) {
                cache_line_top = cache_line_2;
            } else {
                cache_line_top = cache_line_1;
            }
            switch (bpp) {
                case IMAGE_BPP_BINARY:
                    {
                        uint32_t *s;
                        uint32_t *d = (uint32_t *)cache_line_top;
                        cy = downscale_y;
                        if (ysrc + cy > src_img->h) // beyond bottom
                            cy = src_img->h - ysrc;
                        dest_x = dest_x_start;
                        if (bFlipX) {
                            dest_x = dest_x_end-1; // start from right
                        }
                        xsrc = src_x_start;
                        for (int x = dest_x_start; x < dest_x_end; x++, dest_x += delta_x) {
                            cx = downscale_x;
                            if (xsrc + cx > src_img->w)
                                cx = src_img->w - xsrc;
                            acc = 0;
                            for (ty=ysrc; ty < ysrc+cy; ty++) { // inner accumulation loop
                                s = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(src_img, ty);
                                for (tx=xsrc; tx<xsrc+cx; tx++) {
                                    acc += IMAGE_GET_BINARY_PIXEL_FAST(s, tx);
                                } // for tx
                            } // for ty
                            IMAGE_PUT_GRAYSCALE_PIXEL_FAST(d, dest_x, acc >= ((cx * cy)/2));
                            xsrc += downscale_x;
                        } // for x
                    }
                    break;
                case IMAGE_BPP_GRAYSCALE:
                {
                    uint8_t *s;
                    uint8_t *d = cache_line_top;
                    cy = downscale_y;
                    if (ysrc + cy > src_img->h) // beyond bottom
                        cy = src_img->h - ysrc;
                    dest_x = dest_x_start;
                    if (bFlipX) {
                        dest_x = dest_x_end-1; // start from right
                    }
                    xsrc = src_x_start;
                    for (int x = dest_x_start; x < dest_x_end; x++, dest_x += delta_x) {
                        cx = downscale_x;
                        if (xsrc + cx > src_img->w)
                            cx = src_img->w - xsrc;
                        acc = 0;
                        for (ty=ysrc; ty < ysrc+cy; ty++) { // inner accumulation loop
                            s = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(src_img, ty);
                            for (tx=xsrc; tx<xsrc+cx; tx++) {
                                acc += IMAGE_GET_GRAYSCALE_PIXEL_FAST(s, tx);
                            } // for tx
                        } // for ty
                        acc = (acc + (cx * cy)/2) / (cx * cy); // average the pixels
                        IMAGE_PUT_GRAYSCALE_PIXEL_FAST(d, dest_x, (uint8_t)acc);
                        xsrc += downscale_x;
                    } // for x
                }
                    break;
                case IMAGE_BPP_RGB565:
                    {
                        uint16_t *s, pix;
                        int acc_r, acc_g, acc_b;
                        uint16_t *d = (uint16_t *)cache_line_top;
                        cy = downscale_y;
                        if (ysrc + cy > src_img->h) // beyond bottom
                            cy = src_img->h - ysrc;
                        dest_x = dest_x_start;
                        if (bFlipX) {
                            dest_x = dest_x_end-1; // start from right
                        }
                        xsrc = src_x_start;
                        for (int x = dest_x_start; x < dest_x_end; x++, dest_x += delta_x) {
                            cx = downscale_x;
                            if (xsrc + cx > src_img->w)
                                cx = src_img->w - xsrc;
                            acc_r = acc_g = acc_b = 0;
                            for (ty=ysrc; ty < ysrc+cy; ty++) { // inner accumulation loop
                                s = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(src_img, ty);
                                for (tx=xsrc; tx<xsrc+cx; tx++) {
                                    pix = IMAGE_GET_RGB565_PIXEL_FAST(s, tx);
                                    acc_r += COLOR_RGB565_TO_R5(pix);
                                    acc_g += COLOR_RGB565_TO_G6(pix);
                                    acc_b += COLOR_RGB565_TO_B5(pix);
                                } // for tx
                            } // for ty
                            acc_r = (acc_r + (cx * cy)/2) / (cx * cy); // average the pixels
                            acc_g = (acc_g + (cx * cy)/2) / (cx * cy);
                            acc_b = (acc_b + (cx * cy)/2) / (cx * cy);
                            IMAGE_PUT_RGB565_PIXEL_FAST(d, dest_x, COLOR_R5_G6_B5_TO_RGB565(acc_r, acc_g, acc_b));
                            xsrc += downscale_x;
                        } // for x
                    }
                    break;
            } // switch on bpp
            imlib_combine_alpha(alpha, alpha_palette, cache_line_top, dest_y, dest_x_start, dest_x_end, bpp, dest_img);
            ysrc += downscale_y;
        } // for y
        fb_alloc_free_till_mark();
        return; // done with area scaling
    }
        if (hint & IMAGE_HINT_BILINEAR) {
            // Work from destination back to source
            for (int y = dest_y_start; y < dest_y_end; y++, dest_y += delta_y) {
                if (y & 1) {
                    cache_line_top = cache_line_2;
                } else {
                    cache_line_top = cache_line_1;
                }
                switch (bpp) {
                    case IMAGE_BPP_BINARY:
                    {
                        uint32_t *s1, *s2;
                        uint32_t *d = (uint32_t *)cache_line_top;
                        uint32_t ysrc, xsubfrac, ysubfrac;
                        ysrc = y_accum >> 16;
                        // keep source pointer from going out of bounds
                        if (ysrc >= src_img->h-1)
                           ysrc = src_img->h-1;
                        s1 = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(src_img, ysrc);
                        if (ysrc+1 >= src_img->h)
                            s2 = s1;
                        else
                            s2 = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(src_img, ysrc+1);
                        ysubfrac = y_accum & 0xffff;
                        if (ysubfrac >= 0x8000) // lower line takes priority
                           s1 = s2;
                        x_accum = src_x_start << 16;
                        dest_x = (bFlipX) ? dest_x_end-1 : dest_x_start;
                        for (int x = dest_x_start; x < dest_x_end; x++, dest_x += delta_x) {
                            int x00 = x_accum >> 16;
                            uint32_t pixel;
                            xsubfrac = x_accum & 0xffff; // x fractional part only
                            if (x00 >= src_img->w)
                                x00 = src_img->w-1;
                            if (x00 == src_img->w-1) { // keep it within source image bounds
                                pixel = IMAGE_GET_BINARY_PIXEL_FAST(s1, x00);
                            } else {
                                if (xsubfrac >= 0x8000) // right pixel takes priority
                                    pixel = IMAGE_GET_BINARY_PIXEL_FAST(s1, x00+1);
                                else
                                    pixel = IMAGE_GET_BINARY_PIXEL_FAST(s1, x00);
                            }
                            IMAGE_PUT_BINARY_PIXEL_FAST(d, dest_x, pixel);
                            x_accum += x_frac;
                        } // for x
                        imlib_combine_alpha(alpha, alpha_palette, cache_line_top, dest_y, dest_x_start, dest_x_end, bpp, dest_img);
                    }
                    break;

                    case IMAGE_BPP_GRAYSCALE:
                    {
                        uint8_t *s1, *s2;
                        uint8_t *dest_row_ptr = cache_line_top;
                        int32_t x_frac_src;
                        int32_t xsubfrac, ysubfrac;
                        int32_t ysrc, y_frac_src;
                        const uint32_t one_minus = 256;
                        const uint32_t half_frac = 128; // for rounding
                        y_frac_src = (int32_t)y_accum - 0x8000;
                        ysrc = y_frac_src >> 16;
                        // keep source pointer from going out of bounds
                        if (ysrc >= src_img->h-1)
                           ysrc = src_img->h-1;
                        if (ysrc >= src_img->h-1 || ysrc < 0) {
                            if (ysrc < 0) ysrc = 0;
                            s2 = s1 = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(src_img, ysrc);
                        } else {
                            s1 = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(src_img, ysrc);
                            s2 = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(src_img, ysrc+1);
                        }
                        ysubfrac = ((y_frac_src & 0xffff) >> 8);
                        ysubfrac |= ((one_minus - ysubfrac) << 16);
                        x_accum = src_x_start << 16;
                        x_frac_src = (int32_t)x_accum - 0x8000;
                        dest_x = (bFlipX) ? dest_x_end-1 : dest_x_start;
                        for (int x = dest_x_start; x < dest_x_end; x++, dest_x += delta_x) {
                            int32_t x00 = (x_frac_src >> 16);
                            uint32_t pix00, pix10, pix01, pix11, pixTop, pixBot;
                            xsubfrac = (x_frac_src & 0xffff) >> 8; // x fractional part only
                            xsubfrac |= ((one_minus - xsubfrac) << 16);
                            if (x00 >= src_img->w)
                                x00 = src_img->w-1;
                            if (x00 == src_img->w-1 || x00 < 0) { // keep it within source image bounds
                                if (x00 < 0) x00 = 0;
                                pix00 = pix10 = s1[x00];
                                pix01 = pix11 = s2[x00];
                            } else {
                                pix00 = s1[x00]; pix10 = s1[x00+1];
                                pix01 = s2[x00]; pix11 = s2[x00+1]; // get 4 neighboring pixels
                            }
                            // multiply and sum 2 pixels at a time with SMUAD
                            // horizontal average the 2 pairs
                            pixTop = (__SMUAD(xsubfrac, (pix00 << 16) | pix10) + half_frac) >> 8;
                            pixBot = (__SMUAD(xsubfrac, (pix01 << 16) | pix11) + half_frac) >> 8;
                            // vertical average the newly formed pair
                            pixTop = (__SMUAD(ysubfrac, (pixTop << 16) | pixBot) + half_frac) >> 8;
                            IMAGE_PUT_GRAYSCALE_PIXEL_FAST(dest_row_ptr, dest_x, (uint8_t)pixTop);
                            x_frac_src += x_frac;
                        } // for x
                        imlib_combine_alpha(alpha, alpha_palette, cache_line_top, dest_y, dest_x_start, dest_x_end, bpp, dest_img);
                    }
                    break;

                    case IMAGE_BPP_RGB565:
                    {
                        uint16_t *s1, *s2;
                        uint16_t *dest_row_ptr = (uint16_t *)cache_line_top;
                        uint32_t xsubfrac, ysubfrac; //, xfrac_2 ,yfrac_2;
                        int32_t y_frac_src, ysrc;
                        y_frac_src = (int32_t)y_accum - 0x8000;
                        ysrc = (y_frac_src >> 16);
                        // keep source pointer from going out of bounds
                        if (ysrc >= src_img->h-1)
                           ysrc = src_img->h-1;
                        if (ysrc >= src_img->h-1 || ysrc < 0) {
                            if (ysrc < 0) ysrc = 0;
                            if (color_palette) { // convert through given palette
                                draw_palette_rgb565(cache_convert_1, src_img, ysrc, color_palette);
                                s1 = s2 = (uint16_t*)cache_convert_1;
                            } else {
                                s2 = s1 = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(src_img, ysrc);
                            }
                        } else {
                            if (color_palette) { // convert through palette
                                s1 = (uint16_t *)cache_convert_1;
                                s2 = (uint16_t *)cache_convert_2;
                                // debug - skip/swap when possible
                                draw_palette_rgb565(cache_convert_1, src_img, ysrc, color_palette);
                                draw_palette_rgb565(cache_convert_1, src_img, ysrc+1, color_palette);
                            } else {
                                s1 = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(src_img, ysrc);
                                s2 = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(src_img, ysrc+1);
                            }
                        }
                        ysubfrac = (y_frac_src & 0xffff); // use it as a 16-bit fraction
                        x_accum = src_x_start << 16;
                        dest_x = (bFlipX) ? dest_x_end-1 : dest_x_start;
                        for (int x = dest_x_start; x < dest_x_end; x++, dest_x += delta_x) {
                            int32_t x_frac_src = (int32_t)x_accum - 0x8000;
                            int32_t x00 = (x_frac_src >> 16);
                            const uint32_t one_minus = 65536;
                            const uint32_t half_frac = 32768; // for rounding
                            uint32_t g00, g10, g01, g11, gTop, gBot; // green
                            uint32_t r00, r10, r01, r11, rTop, rBot; // red
                            uint32_t b00, b10, b01, b11, bTop, bBot; // blue
                            xsubfrac = (x_frac_src & 0xffff); // x fractional part only
                            if (x00 >= src_img->w)
                                x00 = src_img->w-1;
                            if (x00 == src_img->w-1 || x00 < 0) { // keep it within source image bounds
                                if (x00 < 0) x00 = 0;
                                g00 = g10 = s1[x00];
                                g01 = g11 = s2[x00];
                            } else {
                                g00 = s1[x00]; g10 = s1[x00+1];
                                g01 = s2[x00]; g11 = s2[x00+1]; // get 4 neighboring pixels
                            }
                            g00 = __builtin_bswap16(g00);
                            g10 = __builtin_bswap16(g10);
                            g01 = __builtin_bswap16(g01);
                            g11 = __builtin_bswap16(g11);
                            r00 = (g00 & 0xf800) >> 8; r10 = (g10 & 0xf800) >> 8;
                            r01 = (g01 & 0xf800) >> 8; r11 = (g11 & 0xf800) >> 8;
                            b00 = g00 & 0x1f; b10 = g10 & 0x1f;
                            b01 = g01 & 0x1f; b11 = g11 & 0x1f;
                            g00 = g00 & 0x7e0; g10 = g10 & 0x7e0;
                            g01 = g01 & 0x7e0; g11 = g11 & 0x7e0;
                            // interpolate the 2 pairs of pixels horizontally
                            gTop = (((one_minus-xsubfrac) * g00) + (xsubfrac * g10) + half_frac) >> 16;
                            gTop &= 0x7e0;
                            rTop = (((one_minus-xsubfrac) * r00) + (xsubfrac * r10) + half_frac) >> 16;
                            rTop &= 0xf8;
                            bTop = (((one_minus-xsubfrac) * b00) + (xsubfrac * b10) + half_frac) >> 16;
                            bTop &= 0x1f;
                            gBot = (((one_minus-xsubfrac) * g01) + (xsubfrac * g11) + half_frac) >> 16;
                            gBot &= 0x7e0;
                            rBot = (((one_minus-xsubfrac) * r01) + (xsubfrac * r11) + half_frac) >> 16;
                            rBot &= 0xf8;
                            bBot = (((one_minus-xsubfrac) * b01) + (xsubfrac * b11) + half_frac) >> 16;
                            bBot &= 0x1f;
                            // interpolate the newly formed pair vertically
                            gTop = (((one_minus-ysubfrac) * gTop) + (ysubfrac * gBot) + half_frac) >> 16;
                            gTop &= 0x7e0;
                            rTop = ((((one_minus-ysubfrac) * rTop) + (ysubfrac * rBot)) + half_frac) >> 16;
                            rTop &= 0xf8;
                            bTop = ((((one_minus-ysubfrac) * bTop) + (ysubfrac * bBot)) + half_frac) >> 16;
                            bTop &= 0x1f;
                            // combine R/G/B into RGB565 and byte swap
                            gTop = __builtin_bswap16((rTop<<8) | gTop | bTop);
                            IMAGE_PUT_RGB565_PIXEL_FAST(dest_row_ptr, dest_x, gTop);
                            x_accum += x_frac;
                        } // for x
                        imlib_combine_alpha(alpha, alpha_palette, cache_line_top, dest_y, dest_x_start, dest_x_end, bpp, dest_img);
                    }
                    break;

                } // switch on bpp
            y_accum += y_frac;
            } // for y
        } else if (hint & IMAGE_HINT_BICUBIC) {
            // Impliments the traditional bicubic interpolation algorithm which uses
            // a 4x4 filter block with the current pixel centered at (1,1) (C below).
            // However, instead of floating point math, it uses integer (fixed point).
            // The Cortex-M4/M7 has a hardware floating point unit, so doing FP math
            // doesn't take any extra time, but it does take extra time to convert
            // the integer pixels to floating point and back to integers again.
            // So this allows it to execute more quickly in pure integer math.
            //
            // +---+---+---+---+
            // | x | x | x | x |
            // +---+---+---+---+
            // | x | C | x | x |
            // +---+---+---+---+
            // | x | x | x | x |
            // +---+---+---+---+
            // | x | x | x | x |
            // +---+---+---+---+
            //
            // Work from destination pixels back to source pixels
            for (int y = dest_y_start; y < dest_y_end; y++, dest_y += delta_y) {
                if (y & 1) {
                    cache_line_top = cache_line_2;
                } else {
                    cache_line_top = cache_line_1;
                }
                switch (bpp) {
                    case IMAGE_BPP_BINARY:
                    {
                        uint32_t *s[4]; // 4 rows
                        // the pixels need to be signed integers for the BICUBIC calc
                        int pix0,pix1,pix2,pix3; // 4 columns
                        int32_t dx, dx2, dx3, dy, dy2, dy3;
                        int32_t d0,d1,d2,d3,a0,a1,a2,C[4], Cc;
                        uint32_t *d = (uint32_t *)cache_line_top;
                        // pixel is centered at (-0.5,-0.5) otherwise the image shifts
                        int32_t x_frac_src, y_frac_src = (int32_t)y_accum - 0x8000;
                        int32_t tty, ty = y_frac_src >> 16;
                        dy = (y_frac_src & 0xffff) >> 1;
                        // pre-calculate the ^2 and ^3 of the fraction
                        dy2 = (dy * dy) >> 15;
                        dy3 = (dy2 * dy) >> 15;
                        tty = ty-1; // line above
                        tty = (tty < 0) ? 0 : (tty >= src_img->h) ? (src_img->h -1): tty;
                        s[0] = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(src_img, tty);
                        tty = ty; // current line
                        tty = (tty < 0) ? 0 : (tty >= src_img->h) ? (src_img->h -1): tty;
                        s[1] = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(src_img, tty);
                        tty = ty + 1; // line below
                        tty = (tty >= src_img->h) ? (src_img->h -1): tty;
                        s[2] = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(src_img, tty);
                        tty = ty + 2; // line below + 1
                        tty = (tty >= src_img->h) ? (src_img->h -1): tty;
                        s[3] = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(src_img, tty);

                        x_accum = src_x_start << 16;
                        x_frac_src = (int32_t)x_accum - 0x8000;
                        dest_x = (bFlipX) ? dest_x_end-1 : dest_x_start;
                        for (int x = dest_x_start; x < dest_x_end; x++, dest_x += delta_x) {
                            int tx = x_frac_src >> 16;
                            // Create a 15-bit fraction so the square fits in a int32_t
                            dx = (x_frac_src & 0xffff) >> 1;
                            // pre-calculate the ^2 and ^3 of the fraction
                            dx2 = (dx * dx) >> 15;
                            dx3 = (dx2 * dx) >> 15;
                            if (tx >= src_img->w) tx = src_img->w-1;
                            for (int j = 0; j < 4; j++) { // bicubic y step (-1 to +2)
                                if (tx > 0) {
                                    pix0 = (int)IMAGE_GET_BINARY_PIXEL_FAST(s[j], tx-1);
                                    pix1 = (int)IMAGE_GET_BINARY_PIXEL_FAST(s[j], tx);
                                } else {
                                    pix0 = pix1 = (int)IMAGE_GET_BINARY_PIXEL_FAST(s[j], tx);
                                }
                                if (tx >= src_img->w-1) {
                                    pix2 = pix3 = pix1; // hit right edge
                                } else {
                                    pix2 = (int)IMAGE_GET_BINARY_PIXEL_FAST(s[j], tx+1);
                                    if (tx > src_img->w-2)
                                        pix3 = pix2;
                                    else
                                        pix3 = (int)IMAGE_GET_BINARY_PIXEL_FAST(s[j], tx+2);
                                }
                                d0 = pix0 << 8; // provide more resolution
                                d1 = pix1 << 8; // to the bicubic formula
                                d2 = pix2 << 8;
                                d3 = pix3 << 8;
                                a0 = ((d1 * 3) - (d2 * 3) - d0 + d3) >> 1;
                                a1 = d0 + (2*d2) - (((5*d1) + d3)>>1);
                                a2 = (d2 - d0) >> 1;
                                C[j] = d1 + (((a2 * dx) + (a1 * dx2) + (a0 * dx3)) >> 15);
                            } // for j
                            d0 = C[0];
                            d1 = C[1];
                            d2 = C[2];
                            d3 = C[3];
                            a0 = ((d1 * 3) - (d2 * 3) - d0 + d3) >> 1;
                            a1 = d0 + (2*d2) - (((5*d1) + d3)>>1);
                            a2 = (d2 - d0) >> 1;
                            Cc = d1 + (((a2 * dy) + (a1 * dy2) + (a0 * dy3)) >> 15);
                            IMAGE_PUT_BINARY_PIXEL_FAST(d, dest_x, (Cc >= 128));
                            x_frac_src += x_frac;
                        } // for x
                        imlib_combine_alpha(alpha, alpha_palette, cache_line_top, dest_y, dest_x_start, dest_x_end, bpp, dest_img);
                    }
                    break;

                    case IMAGE_BPP_GRAYSCALE:
                    {
                        uint8_t *s[4]; // 4 rows
                        // the pixels need to be signed integers for BICUBIC
                        int32_t pix0,pix1,pix2,pix3; // 4 columns
                        int32_t dx,dx2,dx3,dy,dy2,dy3;
                        int32_t d0,d1,d2,d3,a0,a1,a2, C[4];
                        uint8_t *d = cache_line_top;
                        int32_t y_frac_src, x_frac_src, ty, tty, ttx;
                        // pixel is centered at (-0.5,-0.5) otherwise the image shifts
                        y_frac_src = (int32_t)y_accum - 0x8000;
                        ty = y_frac_src >> 16;
                        // 15-bit fraction to fit a square of it in 32-bits
                        dy = (y_frac_src & 0xffff) >> 1;
                        // pre-calculate the ^2 and ^3 of the fraction
                        dy2 = (dy * dy) >> 15;
                        dy3 = (dy2 * dy) >> 15;
                        tty = ty-1; // line above
                        tty = (tty < 0) ? 0 : (tty >= src_img->h) ? (src_img->h -1): tty;
                        s[0] = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(src_img, tty);
                        tty = ty; // current line
                        tty = (tty < 0) ? 0 : (tty >= src_img->h) ? (src_img->h -1): tty;
                        s[1] = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(src_img, tty);
                        tty = ty + 1; // line below
                        tty = (tty >= src_img->h) ? (src_img->h -1): tty;
                        s[2] = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(src_img, tty);
                        tty = ty + 2; // line below + 1
                        tty = (tty >= src_img->h) ? (src_img->h -1): tty;
                        s[3] = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(src_img, tty);
                        x_accum = src_x_start << 16;
                        // pixel is centered at (-0.5,-0.5) otherwise the image shifts
                        x_frac_src = (int32_t)x_accum - 0x8000;
                        dest_x = (bFlipX) ? dest_x_end-1 : dest_x_start;
                        for (int x = dest_x_start; x < dest_x_end; x++, dest_x += delta_x) {
                            int tx = x_frac_src >> 16;
                            dx = (x_frac_src & 0xffff)  >> 1;
                            // pre-calculate the ^2 and ^3 of the fraction
                            dx2 = (dx * dx) >> 15;
                            dx3 = (dx2 * dx) >> 15;
                            for (int j = 0; j < 4; j++) { // bicubic y step (-1 to +2)
                                ttx = tx - 1; // left of center
                                ttx = (ttx < 0) ? 0 : (ttx >= src_img->w) ? (src_img->w -1): ttx;
                                pix0 = IMAGE_GET_GRAYSCALE_PIXEL_FAST(s[j], ttx);
                                ttx = tx;  // center
                                ttx = (ttx < 0) ? 0 : (ttx >= src_img->w) ? (src_img->w -1): ttx;
                                pix1 = IMAGE_GET_GRAYSCALE_PIXEL_FAST(s[j], ttx);
                                ttx = tx + 1;
                                ttx = (ttx >= src_img->w) ? (src_img->w -1): ttx;
                                pix2 = IMAGE_GET_GRAYSCALE_PIXEL_FAST(s[j], ttx);
                                ttx = tx + 2;
                                ttx = (ttx >= src_img->w) ? (src_img->w -1): ttx;
                                pix3 = IMAGE_GET_GRAYSCALE_PIXEL_FAST(s[j], ttx);
                                d0 = pix0 << 4; // give each pixel more resolution
                                d1 = pix1 << 4; // in the bicubic equation
                                d2 = pix2 << 4;
                                d3 = pix3 << 4;
                                a0 = ((d1 * 3) - (d2 * 3) - d0 + d3) >> 1;
                                a1 = d0 + (2*d2) - (((5*d1) + d3)>>1);
                                a2 = (d2 - d0) >> 1;
                                C[j] = d1 + (((a2 * dx) + (a1 * dx2) + (a0 * dx3)) >> 15);
    //                            a0 = (-d0/2.0f) + (3.0f*d1)/2.0f - (3.0f*d2)/2.0f + d3/2.0f;
    //                            a1 =  d0 - (5.0f*d1)/2.0f + 2.0f*d2 - d3/2.0f;
    //                            a2 = (-d0/2.0f) + d2/2.0f;
    //                            C[j] = d1 + a2*dx + a1*dx*dx + a0*dx*dx*dx;
                            } // or j
                            d0 = C[0];
                            d1 = C[1];
                            d2 = C[2];
                            d3 = C[3];
    //                        a0 = (-d0/2.0f) + (3.0f*d1)/2.0f - (3.0f*d2)/2.0f + d3/2.0f;
    //                        a1 =  d0 - (5.0f*d1)/2.0f + 2.0f*d2 - d3/2.0f;
    //                        a2 = (-d0/2.0f) + d2/2.0f;
    //                        pix0 = (int)(d1 + a2*dy + a1*dy*dy + a0*dy*dy*dy);
                            a0 = ((d1 * 3) - (d2 * 3) - d0 + d3) >> 1;
                            a1 = d0 + (2*d2) - (((5*d1) + d3)>>1);
                            a2 = (d2 - d0) >> 1;
                            pix0 = d1 + (((a2 * dy) + (a1 * dy2) + (a0 * dy3)) >> 15);
                            if (pix0 < 0) pix0 = 0;
                            pix0 >>= 4; // reduce to original resolution
                            IMAGE_PUT_GRAYSCALE_PIXEL_FAST(d, dest_x, (uint8_t)pix0);
                            x_frac_src += x_frac;
                        } // for x
                        imlib_combine_alpha(alpha, alpha_palette, cache_line_top, dest_y, dest_x_start, dest_x_end, bpp, dest_img);
                    }
                    break;

                    case IMAGE_BPP_RGB565:
                    {
                        // To make best use of the resolution of the pixel colors
                        // we need to shift them left a few places to keep them from
                        // 'falling through the cracks' when divided.
                        // We also can't bump up against the max size of significant bits
                        uint16_t *s[4]; // 4 rows
                        int32_t pix0,pix1,pix2,pix3; // 4 columns
                        int32_t dx,dx2,dx3,dy,dy2,dy3;
                        int32_t d0,d1,d2,d3,a0,a1,a2;
                        int32_t C_R[4], C_G[4], C_B[4];
                        int old_y, Cr, Cg, Cb;
                        uint16_t *d = (uint16_t*)cache_line_top;
                        int32_t y_frac_src, x_frac_src, ty, tty;
                        y_frac_src = (int32_t)y_accum - 0x8000;
                        ty = y_frac_src >> 16;
                        dy = (y_frac_src & 0xffff) >> 1;
                        // pre-calculate the ^2 and ^3 of the fraction
                        dy2 = (dy * dy) >> 15;
                        dy3 = (dy2 * dy) >> 15;
                        tty = ty-1; // line above
                        old_y = tty = (tty < 0) ? 0 : (tty >= src_img->h) ? (src_img->h -1): tty;
                        if (color_palette) { // convert through given palette
                            draw_palette_rgb565(cache_convert_1, src_img, tty, color_palette);
                            s[0] = (uint16_t*)cache_convert_1;
                        } else {
                            s[0] = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(src_img, tty);
                        }
                        tty = ty; // current line
                        tty = (tty < 0) ? 0 : (tty >= src_img->h) ? (src_img->h -1): tty;
                        if (color_palette) {
                            if (tty != old_y) { // different line
                                s[1] = (uint16_t*)cache_convert_2;
                                draw_palette_rgb565(cache_convert_2, src_img, tty, color_palette);
                            } else { // same line
                                s[1] = s[0];
                            }
                        } else {
                            s[1] = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(src_img, tty);
                        }
                        old_y = tty;
                        tty = ty + 1; // line below
                        tty = (tty >= src_img->h) ? (src_img->h -1): tty;
                        if (color_palette) {
                            if (tty != old_y) { // different line
                                s[2] = (uint16_t*)cache_convert_3;
                                draw_palette_rgb565(cache_convert_3, src_img, tty, color_palette);
                            } else { // same line
                                s[2] = s[1];
                            }
                        } else {
                            s[2] = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(src_img, tty);
                        }
                        old_y = tty;
                        tty = ty + 2; // line below + 1
                        tty = (tty >= src_img->h) ? (src_img->h -1): tty;
                        if (color_palette) {
                            if (tty != old_y) { // different line
                                s[3] = (uint16_t*)cache_convert_4;
                                draw_palette_rgb565(cache_convert_4, src_img, tty, color_palette);
                            } else { // same line
                                s[3] = s[2];
                            }
                        } else {
                            s[3] = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(src_img, tty);
                        }
                        x_frac_src = (src_x_start << 16) - 0x8000;
                        dest_x = (bFlipX) ? dest_x_end-1 : dest_x_start;
                        for (int x = dest_x_start; x < dest_x_end; x++, dest_x += delta_x) {
                            int32_t tx, ttx;
                            tx = x_frac_src >> 16;
                            dx = (x_frac_src & 0xffff) >> 1;
                            // pre-calculate the ^2 and ^3 of the fraction
                            dx2 = (dx * dx) >> 15;
                            dx3 = (dx2 * dx) >> 15;
                            for (int j = 0; j < 4; j++) { // bicubic y step (-1 to +2)
                                ttx = tx - 1; // left of center
                                ttx = (ttx < 0) ? 0 : (ttx >= src_img->w) ? (src_img->w -1): ttx;
                                pix0 = IMAGE_GET_RGB565_PIXEL_FAST(s[j], ttx);
                                ttx = tx;  // center
                                ttx = (ttx < 0) ? 0 : (ttx >= src_img->w) ? (src_img->w -1): ttx;
                                pix1 = IMAGE_GET_RGB565_PIXEL_FAST(s[j], ttx);
                                ttx = tx + 1;
                                ttx = (ttx >= src_img->w) ? (src_img->w -1): ttx;
                                pix2 = IMAGE_GET_RGB565_PIXEL_FAST(s[j], ttx);
                                ttx = tx + 2;
                                ttx = (ttx >= src_img->w) ? (src_img->w -1): ttx;
                                pix3 = IMAGE_GET_RGB565_PIXEL_FAST(s[j], ttx);
                                pix0 = __builtin_bswap16(pix0);
                                pix1 = __builtin_bswap16(pix1);
                                pix2 = __builtin_bswap16(pix2);
                                pix3 = __builtin_bswap16(pix3);
                                // Red
                                d0 = (int)((pix0 & 0xf800) >> 4);
                                d1 = (int)((pix1 & 0xf800) >> 4);
                                d2 = (int)((pix2 & 0xf800) >> 4);
                                d3 = (int)((pix3 & 0xf800) >> 4);
                                a0 = ((d1 * 3) - (d2 * 3) - d0 + d3) >> 1;
                                a1 = d0 + (2*d2) - (((5*d1) + d3)>>1);
                                a2 = (d2 - d0) >> 1;
                                C_R[j] = d1 + (((a2 * dx) + (a1 * dx2) + (a0 * dx3)) >> 15);
                                // Green
                                d0 = (int)((pix0 & 0x7e0) >> 0);
                                d1 = (int)((pix1 & 0x7e0) >> 0);
                                d2 = (int)((pix2 & 0x7e0) >> 0);
                                d3 = (int)((pix3 & 0x7e0) >> 0);
                                a0 = ((d1 * 3) - (d2 * 3) - d0 + d3) >> 1;
                                a1 = d0 + (2*d2) - (((5*d1) + d3)>>1);
                                a2 = (d2 - d0) >> 1;
                                C_G[j] = d1 + (((a2 * dx) + (a1 * dx2) + (a0 * dx3)) >> 15);
                                // Blue
                                d0 = (int)((pix0 & 0x1f) << 4);
                                d1 = (int)((pix1 & 0x1f) << 4);
                                d2 = (int)((pix2 & 0x1f) << 4);
                                d3 = (int)((pix3 & 0x1f) << 4);
                                a0 = ((d1 * 3) - (d2 * 3) - d0 + d3) >> 1;
                                a1 = d0 + (2*d2) - (((5*d1) + d3)>>1);
                                a2 = (d2 - d0) >> 1;
                                C_B[j] = d1 + (((a2 * dx) + (a1 * dx2) + (a0 * dx3)) >> 15);
                            } // for j
                            // output final pixel, R first
                            d0 = C_R[0];
                            d1 = C_R[1];
                            d2 = C_R[2];
                            d3 = C_R[3];
                            a0 = ((d1 * 3) - (d2 * 3) - d0 + d3) >> 1;
                            a1 = d0 + (2*d2) - (((5*d1) + d3)>>1);
                            a2 = (d2 - d0) >> 1;
                            Cr = d1 + (((a2 * dy) + (a1 * dy2) + (a0 * dy3)) >> 15);

                            d0 = C_G[0];
                            d1 = C_G[1];
                            d2 = C_G[2];
                            d3 = C_G[3];
                            a0 = ((d1 * 3) - (d2 * 3) - d0 + d3) >> 1;
                            a1 = d0 + (2*d2) - (((5*d1) + d3)>>1);
                            a2 = (d2 - d0) >> 1;
                            Cg = d1 + (((a2 * dy) + (a1 * dy2) + (a0 * dy3)) >> 15);

                            d0 = C_B[0];
                            d1 = C_B[1];
                            d2 = C_B[2];
                            d3 = C_B[3];
                            a0 = ((d1 * 3) - (d2 * 3) - d0 + d3) >> 1;
                            a1 = d0 + (2*d2) - (((5*d1) + d3)>>1);
                            a2 = (d2 - d0) >> 1;
                            Cb = d1 + (((a2 * dy) + (a1 * dy2) + (a0 * dy3)) >> 15);
                            if (Cr < 0) Cr = 0;
                            else {
                                Cr = (Cr & 0x1f80) >> 7;
                                if (Cr > COLOR_R5_MAX) Cr = COLOR_R5_MAX;
                            }
                            if (Cg < 0) Cg = 0;
                            else {
                                Cg = (Cg & 0xfe0) >> 5;
                                if (Cg > COLOR_G6_MAX) Cg = COLOR_G6_MAX;
                            }
                            if (Cb < 0) Cb = 0;
                            else {
                                Cb = (Cb & 0x3f0) >> 4;
                                if (Cb > COLOR_B5_MAX) Cb = COLOR_B5_MAX;
                            }
                            pix0 = COLOR_R5_G6_B5_TO_RGB565((uint8_t)Cr, (uint8_t)Cg, (uint8_t)Cb);
                            IMAGE_PUT_RGB565_PIXEL_FAST(d, dest_x, pix0);
                            x_frac_src += x_frac;
                        } // for x
                        imlib_combine_alpha(alpha, alpha_palette, cache_line_top, dest_y, dest_x_start, dest_x_end, bpp, dest_img);
                    }
                    break;
                } // switch on bpp
                y_accum += y_frac;
            } // for y
        } else { // nearest neighbor
            // Work from destination back to source
            for (int y = dest_y_start; y < dest_y_end; y++, dest_y += delta_y) {
                if (y & 1) {
                    cache_line_top = cache_line_2;
                } else {
                    cache_line_top = cache_line_1;
                }
                switch (bpp) {
                    case IMAGE_BPP_BINARY:
                    {
                        uint32_t *src_row_ptr = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(src_img, y_accum >> 16);
                        uint32_t *d = (uint32_t *)cache_line_top;
                        x_accum = src_x_start << 16;
                        dest_x = (bFlipX) ? dest_x_end-1 : dest_x_start;
                        for (int x = dest_x_start; x < dest_x_end; x++, dest_x += delta_x) {
                            uint32_t pixel = IMAGE_GET_BINARY_PIXEL_FAST(src_row_ptr, x_accum >> 16);
                            IMAGE_PUT_BINARY_PIXEL_FAST(d, dest_x, pixel);
                            x_accum += x_frac;
                        } // for x
                        imlib_combine_alpha(alpha, alpha_palette, cache_line_top, dest_y, dest_x_start, dest_x_end, bpp, dest_img);
                    }
                    break; // BINARY

                    case IMAGE_BPP_GRAYSCALE:
                    {
                       uint8_t *src_row_ptr = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(src_img, y_accum >> 16);
                       uint8_t *dest_row_ptr = cache_line_top;
                        x_accum = src_x_start << 16;
                        dest_x = (bFlipX) ? dest_x_end-1 : dest_x_start;
                        for (int x = dest_x_start; x < dest_x_end; x++, dest_x += delta_x) {
                            uint8_t pixel = IMAGE_GET_GRAYSCALE_PIXEL_FAST(src_row_ptr, x_accum >> 16);
                            IMAGE_PUT_GRAYSCALE_PIXEL_FAST(dest_row_ptr, x, pixel);
                            x_accum += x_frac;
                        } // for x
                        imlib_combine_alpha(alpha, alpha_palette, cache_line_top, dest_y, dest_x_start, dest_x_end, bpp, dest_img);
                    }
                    break; // GRAYSCALE

                    case IMAGE_BPP_RGB565:
                    {
                        uint16_t *src_row_ptr = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(src_img, y_accum >> 16);
                        uint16_t *dest_row_ptr = (uint16_t *)cache_line_top;
                        if (color_palette) {
                            src_row_ptr = (uint16_t*)cache_convert_1;
                            draw_palette_rgb565(cache_convert_1, src_img, y_accum >> 16, color_palette);
                        }
                        x_accum = src_x_start << 16;
                        dest_x = (bFlipX) ? dest_x_end-1 : dest_x_start;
                        for (int x = dest_x_start; x < dest_x_end; x++, dest_x += delta_x) {
                            uint16_t pixel = IMAGE_GET_RGB565_PIXEL_FAST(src_row_ptr, x_accum >> 16);
                            IMAGE_PUT_RGB565_PIXEL_FAST(dest_row_ptr, dest_x, pixel);
                            x_accum += x_frac;
                        } // for x
                        imlib_combine_alpha(alpha, alpha_palette, cache_line_top, dest_y, dest_x_start, dest_x_end, bpp, dest_img);
                    }
                    break; // RGB565
                } // switch on pixel type
                y_accum += y_frac;
            } // for y
        } // nearest neighbor

        // De-allocate cache lines
        fb_alloc_free_till_mark();

} /* imlib_draw_image() */

#ifdef IMLIB_ENABLE_FLOOD_FILL
void imlib_flood_fill(image_t *img, int x, int y,
                      float seed_threshold, float floating_threshold,
                      int c, bool invert, bool clear_background, image_t *mask)
{
    if ((0 <= x) && (x < img->w) && (0 <= y) && (y < img->h)) {
        image_t out;
        out.w = img->w;
        out.h = img->h;
        out.bpp = IMAGE_BPP_BINARY;
        out.data = fb_alloc0(image_size(&out), FB_ALLOC_NO_HINT);

        if (mask) {
            for (int y = 0, yy = out.h; y < yy; y++) {
                uint32_t *row_ptr = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(&out, y);
                for (int x = 0, xx = out.w; x < xx; x++) {
                    if (image_get_mask_pixel(mask, x, y)) IMAGE_SET_BINARY_PIXEL_FAST(row_ptr, x);
                }
            }
        }

        int color_seed_threshold = 0;
        int color_floating_threshold = 0;

        switch(img->bpp) {
            case IMAGE_BPP_BINARY: {
                color_seed_threshold = fast_floorf(seed_threshold * COLOR_BINARY_MAX);
                color_floating_threshold = fast_floorf(floating_threshold * COLOR_BINARY_MAX);
                break;
            }
            case IMAGE_BPP_GRAYSCALE: {
                color_seed_threshold = fast_floorf(seed_threshold * COLOR_GRAYSCALE_MAX);
                color_floating_threshold = fast_floorf(floating_threshold * COLOR_GRAYSCALE_MAX);
                break;
            }
            case IMAGE_BPP_RGB565: {
                color_seed_threshold = COLOR_R5_G6_B5_TO_RGB565(fast_floorf(seed_threshold * COLOR_R5_MAX),
                                                                fast_floorf(seed_threshold * COLOR_G6_MAX),
                                                                fast_floorf(seed_threshold * COLOR_B5_MAX));
                color_floating_threshold = COLOR_R5_G6_B5_TO_RGB565(fast_floorf(floating_threshold * COLOR_R5_MAX),
                                                                    fast_floorf(floating_threshold * COLOR_G6_MAX),
                                                                    fast_floorf(floating_threshold * COLOR_B5_MAX));
                break;
            }
            default: {
                break;
            }
        }

        imlib_flood_fill_int(&out, img, x, y, color_seed_threshold, color_floating_threshold, NULL, NULL);

        switch(img->bpp) {
            case IMAGE_BPP_BINARY: {
                for (int y = 0, yy = out.h; y < yy; y++) {
                    uint32_t *row_ptr = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(img, y);
                    uint32_t *out_row_ptr = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(&out, y);
                    for (int x = 0, xx = out.w; x < xx; x++) {
                        if (IMAGE_GET_BINARY_PIXEL_FAST(out_row_ptr, x) ^ invert) {
                            IMAGE_PUT_BINARY_PIXEL_FAST(row_ptr, x, c);
                        } else if (clear_background) {
                            IMAGE_PUT_BINARY_PIXEL_FAST(row_ptr, x, 0);
                        }
                    }
                }
                break;
            }
            case IMAGE_BPP_GRAYSCALE: {
                for (int y = 0, yy = out.h; y < yy; y++) {
                    uint8_t *row_ptr = IMAGE_COMPUTE_GRAYSCALE_PIXEL_ROW_PTR(img, y);
                    uint32_t *out_row_ptr = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(&out, y);
                    for (int x = 0, xx = out.w; x < xx; x++) {
                        if (IMAGE_GET_BINARY_PIXEL_FAST(out_row_ptr, x) ^ invert) {
                            IMAGE_PUT_GRAYSCALE_PIXEL_FAST(row_ptr, x, c);
                        } else if (clear_background) {
                            IMAGE_PUT_GRAYSCALE_PIXEL_FAST(row_ptr, x, 0);
                        }
                    }
                }
                break;
            }
            case IMAGE_BPP_RGB565: {
                for (int y = 0, yy = out.h; y < yy; y++) {
                    uint16_t *row_ptr = IMAGE_COMPUTE_RGB565_PIXEL_ROW_PTR(img, y);
                    uint32_t *out_row_ptr = IMAGE_COMPUTE_BINARY_PIXEL_ROW_PTR(&out, y);
                    for (int x = 0, xx = out.w; x < xx; x++) {
                        if (IMAGE_GET_BINARY_PIXEL_FAST(out_row_ptr, x) ^ invert) {
                            IMAGE_PUT_RGB565_PIXEL_FAST(row_ptr, x, c);
                        } else if (clear_background) {
                            IMAGE_PUT_RGB565_PIXEL_FAST(row_ptr, x, 0);
                        }
                    }
                }
                break;
            }
            default: {
                break;
            }
        }

        fb_free();
    }
}
#endif // IMLIB_ENABLE_FLOOD_FILL
