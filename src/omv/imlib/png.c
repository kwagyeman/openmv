/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2021 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2021 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * PNG CODEC
 */
#include <stdio.h>

#include "ff_wrapper.h"
#include "imlib.h"
#include "omv_boardconfig.h"

// The PNG encoder and decoder
#include "PNGenc.h"
#include "PNGdec.h"
#include "png_enc.inl"
#include "png_dec.inl"

#define TIME_PNG   (0)
#if (TIME_PNG == 1)
#include "py/mphal.h"
#endif

const uint8_t ucMirror[256] =
     {0, 128, 64, 192, 32, 160, 96, 224, 16, 144, 80, 208, 48, 176, 112, 240,
      8, 136, 72, 200, 40, 168, 104, 232, 24, 152, 88, 216, 56, 184, 120, 248,
      4, 132, 68, 196, 36, 164, 100, 228, 20, 148, 84, 212, 52, 180, 116, 244,
      12, 140, 76, 204, 44, 172, 108, 236, 28, 156, 92, 220, 60, 188, 124, 252,
      2, 130, 66, 194, 34, 162, 98, 226, 18, 146, 82, 210, 50, 178, 114, 242,
      10, 138, 74, 202, 42, 170, 106, 234, 26, 154, 90, 218, 58, 186, 122, 250,
      6, 134, 70, 198, 38, 166, 102, 230, 22, 150, 86, 214, 54, 182, 118, 246,
      14, 142, 78, 206, 46, 174, 110, 238, 30, 158, 94, 222, 62, 190, 126, 254,
      1, 129, 65, 193, 33, 161, 97, 225, 17, 145, 81, 209, 49, 177, 113, 241,
      9, 137, 73, 201, 41, 169, 105, 233, 25, 153, 89, 217, 57, 185, 121, 249,
      5, 133, 69, 197, 37, 165, 101, 229, 21, 149, 85, 213, 53, 181, 117, 245,
      13, 141, 77, 205, 45, 173, 109, 237, 29, 157, 93, 221, 61, 189, 125, 253,
      3, 131, 67, 195, 35, 163, 99, 227, 19, 147, 83, 211, 51, 179, 115, 243,
      11, 139, 75, 203, 43, 171, 107, 235, 27, 155, 91, 219, 59, 187, 123, 251,
      7, 135, 71, 199, 39, 167, 103, 231, 23, 151, 87, 215, 55, 183, 119, 247,
      15, 143, 79, 207, 47, 175, 111, 239, 31, 159, 95, 223, 63, 191, 127, 255};

bool png_compress(image_t *src, image_t *dst)
{
uint8_t *pTemp;
#if (TIME_PNG==1)
    mp_uint_t start = mp_hal_ticks_ms();
#endif

    // If dst->data == NULL then we need to fb_alloc() space for the payload which will be fb_free()'d
    // by the caller. We have to alloc this memory for all cases if we return from the method.
    if (!dst->data) {
        uint32_t avail = fb_avail();
        uint32_t space = sizeof(PNGIMAGE) + (src->w * 4) + 1024;

        if (avail < space) {
            fb_alloc_fail();
        }

        dst->size = IMLIB_IMAGE_MAX_SIZE(avail - space);
        dst->data = fb_alloc(dst->size, FB_ALLOC_PREFER_SIZE);
    }

    if (src->is_compressed) {
        return true;
    }
    pTemp = (uint8_t *)fb_alloc(src->w*4, FB_ALLOC_NO_HINT);
    PNGIMAGE *pPNG = (PNGIMAGE *) fb_alloc(sizeof(PNGIMAGE), FB_ALLOC_NO_HINT);
    memset(pPNG, 0, sizeof(PNGIMAGE));

    int rc, y, iPitch = 0;
    const uint8_t pBinaryPalette[] = {0,0,0,0xff,0xff,0xff};

    pPNG->iTransparent = -1; // no transparent color

    // Set up the output buffer
    pPNG->pOutput = dst->data;
    pPNG->iBufferSize = dst->size;

    // Set up the encoding parameters in the PNGIMAGE structure
    pPNG->iWidth = src->w;
    pPNG->iHeight = src->h;
    pPNG->ucCompLevel = 9; // DEBUG - allow this to change?
    pPNG->y = 0; // first line

    switch (src->pixfmt) {
        case PIXFORMAT_BINARY:
            pPNG->ucBpp = 1;
            iPitch = (src->w + 31) >> 5;
            pPNG->ucPixelType = PNG_PIXEL_INDEXED;
            memcpy(pPNG->ucPalette, pBinaryPalette, sizeof(pBinaryPalette));
            break;
        case PIXFORMAT_GRAYSCALE:
            iPitch = src->w;
            pPNG->ucBpp = 8;
            pPNG->ucPixelType = PNG_PIXEL_GRAYSCALE;
            break;
        case PIXFORMAT_RGB565:
            pPNG->ucBpp = 24;
            iPitch = src->w * 2;
            pPNG->ucPixelType = PNG_PIXEL_TRUECOLOR;
            break;
        case PIXFORMAT_YUV_ANY:
            pPNG->ucBpp = 24;
            iPitch = src->w;
            pPNG->ucPixelType = PNG_PIXEL_TRUECOLOR;
            break;
        case PIXFORMAT_BAYER_ANY:
            pPNG->ucBpp = 24;
            iPitch = src->w;
            pPNG->ucPixelType = PNG_PIXEL_TRUECOLOR;
            break;
    }

    // Encode the image a line at a time
    rc = PNG_SUCCESS;
    for (y = 0; y < src->h && rc == PNG_SUCCESS; y++) {
        uint16_t us, *pPixels;
        uint8_t *s, *d = pTemp;
        pPixels = (uint16_t *)&src->data[y * iPitch];
        // Convert the input format into one of the supported PNG pixel formats
        switch (src->pixfmt) {
            case PIXFORMAT_GRAYSCALE:
                 memcpy(pTemp, pPixels, iPitch);
                 break;
            case PIXFORMAT_BINARY:
                s = (uint8_t *)pPixels;
                for (int x=0; x<iPitch; x++) {
                    *d++ = ucMirror[*s++];
                }
                break;
            case PIXFORMAT_RGB565:
                for (int x=0; x<src->w; x++) {
                    us = *pPixels++;
                    *d++ = (uint8_t)(((us >> 8) & 0xf8) | (us >> 13)); // red
                    *d++ = (uint8_t)(((us >> 3) & 0xfc) | ((us >> 9) & 0x3)); // green
                    *d++ = (uint8_t)(((us & 0x1f) << 3) | ((us & 0x1c) >> 2)); // blue
                }
                break;
        } // switch on format
        rc = PNGAddLine(pPNG, pTemp, y);
    } // for y
    fb_free(); // Free PNGIMAGE structure

    dst->size = pPNG->iDataSize; // output file size

#if (TIME_PNG==1)
    printf("time: %lums\n", mp_hal_ticks_ms() - start);
#endif
    return false;
}

//
// PNG decoder callback
// Called once per line of output
// A convenient place to convert the pixel format
// without having to allocate an intermediate buffer
//
void png_callback(PNGDRAW *pDraw)
{
    image_t *dst = (image_t *) pDraw->pUser; // pointer to destination image

    // Switch on requested output pixel format
    switch (dst->pixfmt) { // convert the pixel format already
        case PIXFORMAT_GRAYSCALE:
            if (pDraw->iPixelType == PNG_PIXEL_TRUECOLOR_ALPHA || pDraw->iPixelType == PNG_PIXEL_TRUECOLOR) {
                uint8_t *s = pDraw->pPixels;
                int pixel;
                uint8_t *d = (uint8_t *)dst->data;
                const int iDelta = (pDraw->iPixelType == PNG_PIXEL_TRUECOLOR_ALPHA) ? 4:3;
                d += pDraw->y * pDraw->iWidth; // starting offset
                for (int i=0; i<pDraw->iWidth; i++) {
                    pixel = s[0] + (s[1]<<1) + s[2]; // easy grayscale
                    *d++ = (uint8_t)(pixel >> 2);
                    s += iDelta;
                }
            } else if (pDraw->iPixelType == PNG_PIXEL_GRAYSCALE) {
                uint8_t *d = (uint8_t *)dst->data;
                d += pDraw->y * pDraw->iWidth; // starting offset
                memcpy(d, pDraw->pPixels, pDraw->iWidth);
            } else if (pDraw->iPixelType == PNG_PIXEL_INDEXED) { // indexed = palette colors
                uint8_t *pPal = pDraw->pPalette;
                uint32_t pixel;
                uint8_t c0, c1, uc, *s, *d, *p;
                d = (uint8_t *)dst->data + (pDraw->y * pDraw->iWidth);
                s = pDraw->pPixels;
                switch (pDraw->iBpp) {
                    case 1:
                        pixel = pPal[0] + (pPal[1]<<1) + pPal[2]; // color 0
                        c0 = (uint8_t)(pixel >> 2);
                        pixel = pPal[3] + (pPal[4]<<1) + pPal[5]; // color 1
                        c1 = (uint8_t)(pixel >> 2);
                        for (int i=0; i<pDraw->iWidth; i+=8) {
                            uc = *s++;
                            for (int j=0; j<8; j++) { // work on individual bits
                                if (uc & 0x80) {
                                    *d++ = c1;
                                } else {
                                    *d++ = c0;
                                }
                                uc <<= 1;
                            }
                        }
                        break;
                    case 2:
                        for (int i=0; i<pDraw->iWidth; i+=4) {
                            uc = *s++;
                            for (int j=0; j<4; j++) { // work on pairs of bits
                                p = &pPal[(uc>>6) * 3];
                                pixel = p[0] + (p[1]<<1) + p[2]; // quick gray calc
                                *d++ = (uint8_t)(pixel >> 2);
                                uc <<= 2;
                            }
                        }
                        break;
                    case 4:
                        for (int i=0; i<pDraw->iWidth; i+=2) {
                            uc = *s++;
                            p = &pPal[(uc>>4) * 3];
                            pixel = p[0] + (p[1]<<1) + p[2]; // quick gray calc
                            *d++ = (uint8_t)(pixel >> 2);
                            p = &pPal[(uc&0xf) * 3];
                            pixel = p[0] + (p[1]<<1) + p[2];
                            *d++ = (uint8_t)(pixel >> 2);
                        }
                        break;
                    case 8:
                        for (int i=0; i<pDraw->iWidth; i++) {
                            p = &pPal[s[i] * 3];
                            pixel = p[0] + (p[1]<<1) + p[2]; // quick gray calc
                            *d++ = (uint8_t)(pixel >> 2);
                        }
                        break;
                } // switch on source bits per pixel
            } else if (pDraw->iPixelType == PNG_PIXEL_GRAY_ALPHA) {
                uint8_t c, a, *s, *d = (uint8_t *)dst->data;
                int j;
                d += pDraw->y * pDraw->iWidth; // starting offset
                s = pDraw->pPixels;
                for (int i=0; i<pDraw->iWidth; i++) {
                    c = *s++; // gray level
                    a = *s++; // alpha
                    j = (a * c) >> 8; // multiply by the alpha
                    *d++ = (uint8_t)j;
                }
            }
            break;
        case PIXFORMAT_BINARY:
            if (pDraw->iPixelType == PNG_PIXEL_TRUECOLOR_ALPHA || pDraw->iPixelType == PNG_PIXEL_TRUECOLOR) {
                uint8_t *s = pDraw->pPixels;
                int pixel;
                uint8_t uc, ucMask, *d = (uint8_t *)dst->data;
                const int iPitch = IMAGE_BINARY_LINE_LEN_BYTES(dst);
                const int iDelta = (pDraw->iPixelType == PNG_PIXEL_TRUECOLOR_ALPHA) ? 4 : 3;
                d += pDraw->y * iPitch; // starting offset
                ucMask = 0x1;
                uc = 0;
                for (int i=0; i<pDraw->iWidth; i++) {
                    pixel = s[0] + (s[1]<<1) + s[2]; // easy grayscale
                    if (pixel >= 512) { // white
                        uc |= ucMask;
                    }
                    ucMask <<= 1;
                    if (ucMask == 0) { // new byte
                        *d++ = uc;
                        uc = 0;
                        ucMask = 0x1;
                    }
                    s += iDelta;
                }
                *d++ = uc; // store last partial byte
            } else if (pDraw->iPixelType == PNG_PIXEL_GRAYSCALE) {
                uint8_t *s = pDraw->pPixels;
                uint8_t uc, ucMask, *d = (uint8_t *)dst->data;
                const int iPitch = IMAGE_BINARY_LINE_LEN_BYTES(dst);
                d += pDraw->y * iPitch; // starting offset
                uc = 0;
                ucMask = 0x1;
                for (int i=0; i<pDraw->iWidth; i++) {
                    if (s[i] >= 128) { // white
                        uc |= ucMask;
                    }
                    ucMask <<= 1;
                    if (ucMask == 0) { // new byte
                        *d++ = uc;
                        uc = 0;
                        ucMask = 0x1;
                    }
                }
                *d++ = uc; // store last partial byte
            } else if (pDraw->iPixelType == PNG_PIXEL_GRAY_ALPHA) {
                uint8_t *s = pDraw->pPixels;
                uint8_t uc, ucMask, *d = (uint8_t *)dst->data;
                uint32_t pixel;
                const int iPitch = IMAGE_BINARY_LINE_LEN_BYTES(dst);
                d += pDraw->y * iPitch; // starting offset
                uc = 0;
                ucMask = 0x1;
                for (int i=0; i<pDraw->iWidth; i++) {
                    pixel = *s++; // gray level
                    pixel *= *s++; // times alpha
                    if (pixel >= 32768) { // white
                        uc |= ucMask;
                    }
                    ucMask <<= 1;
                    if (ucMask == 0) { // new byte
                        *d++ = uc;
                        uc = 0;
                        ucMask = 0x1;
                    }
                }
                *d++ = uc; // store last partial byte
            } else if (pDraw->iPixelType == PNG_PIXEL_INDEXED) { // indexed = palette colors
                uint8_t *pPal = pDraw->pPalette;
                uint32_t pixel;
                uint8_t c0, c1, uc, ucOut, ucMask, *s, *d, *p;
                const int iPitch = IMAGE_BINARY_LINE_LEN_BYTES(dst);
                d = (uint8_t *)dst->data;
                d += pDraw->y * iPitch; // starting offset
                s = pDraw->pPixels;
                switch (pDraw->iBpp) {
                    case 1:
                        pixel = pPal[0] + (pPal[1]<<1) + pPal[2]; // color 0
                        c0 = (uint8_t)(pixel >> 9);
                        pixel = pPal[3] + (pPal[4]<<1) + pPal[5]; // color 1
                        c1 = (uint8_t)(pixel >> 9);
                        ucOut = 0;
                        for (int i=0; i<pDraw->iWidth; i+=8) {
                            uc = *s++;
                            for (int j=0; j<8; j++) { // work on individual bits
                                if (uc & 0x80) {
                                    ucOut |= (c0 << j);
                                } else {
                                    ucOut |= (c1 << j);
                                }
                                uc <<= 1;
                            }
                            *d++ = ucOut;
                            ucOut = 0;
                        }
                        if (ucOut) {
                            *d++ = ucOut; // store final partial byte
                        }
                        break;
                    case 2:
                        ucOut = 0;
                        ucMask = 1;
                        for (int i=0; i<pDraw->iWidth; i+=4) {
                            uc = *s++;
                            for (int j=0; j<4; j++) { // work on pairs of bits
                                p = &pPal[(uc>>6) * 3];
                                pixel = (p[0] + (p[1]<<1) + p[2]) >> 9; // quick gray calc
                                if (pixel) {
                                    ucOut |= ucMask;
                                }
                                uc <<= 2;
                                ucMask <<= 1;
                                if (ucMask == 0) {
                                    *d++ = ucOut;
                                    ucMask = 1;
                                    ucOut = 0;
                                }
                            }
                        }
                        if (ucOut)
                            *d++ = ucOut; // store partial byte
                        break;
                    case 4:
                        ucOut = 0;
                        ucMask = 1;
                        for (int i=0; i<pDraw->iWidth; i+=2) {
                            uc = *s++;
                            p = &pPal[(uc>>4) * 3];
                            pixel = (p[0] + (p[1]<<1) + p[2]) >> 9; // quick gray calc
                            if (pixel) {
                                ucOut |= ucMask;
                            }
                            ucMask <<= 1;
                            p = &pPal[(uc&0xf) * 3];
                            pixel = (p[0] + (p[1]<<1) + p[2]) >> 9;
                            if (pixel) {
                                ucOut |= ucMask;
                            }
                            ucMask <<= 1;
                            if (ucMask == 0) {
                                *d++ = ucOut;
                                ucOut = 0;
                                ucMask = 1;
                            }
                        }
                        if (ucOut)
                            *d++ = ucOut; // store last partial byte
                        break;
                    case 8:
                        ucOut = 0;
                        ucMask = 1;
                        for (int i=0; i<pDraw->iWidth; i++) {
                            p = &pPal[s[i] * 3];
                            pixel = (p[0] + (p[1]<<1) + p[2]) >> 9; // quick gray calc
                            if (pixel) {
                                ucOut |= ucMask;
                            }
                            ucMask <<= 1;
                            if (ucMask == 0) {
                                *d++ = ucOut;
                                ucOut = 0;
                                ucMask = 1;
                            }
                        }
                        if (ucOut) {
                            *d++ = ucOut; // store last partial byte
                        }
                        break;
                } // switch on source bits per pixel
            }
            break;
        case PIXFORMAT_RGB565: {
                uint16_t *d = (uint16_t *)dst->data;
                d += (pDraw->y * pDraw->iWidth);
                PNGRGB565(pDraw, d, PNG_RGB565_LITTLE_ENDIAN, 0, 0);
            }
            break;
   } /* switch on pixel format */
} /* png_callback() */

void png_decompress(image_t *dst, image_t *src)
{
    PNG_DEC_IMAGE *pPNG = (PNG_DEC_IMAGE *) fb_alloc(sizeof(PNG_DEC_IMAGE), FB_ALLOC_NO_HINT);
    memset(pPNG, 0, sizeof(PNG_DEC_IMAGE));

    if (PNGDEC_openRAM(pPNG, src->data, src->size, NULL) == PNG_SUCCESS) {
       pPNG->pfnDraw = png_callback; // give the PNG decoder a pointer to the callback function
       DecodePNG(pPNG, (void *) dst, (dst->pixfmt == PIXFORMAT_RGB565) ? PNG_FAST_PALETTE : 0);
    }

    fb_free(); // free the PNG structure
}

#if defined(IMLIB_ENABLE_IMAGE_FILE_IO)
// This function inits the geometry values of an image.
void png_read_geometry(FIL *fp, image_t *img, const char *path, png_read_settings_t *rs)
{
    uint32_t header;
    file_seek(fp, 12); // start of IHDR
    read_long(fp, &header);
    if (header == 0x52444849) { // IHDR
        uint32_t width, height;
        read_long(fp, &width);
        read_long(fp, &height);
        width = __builtin_bswap32(width);
        height = __builtin_bswap32(height);

        rs->png_w = width;
        rs->png_h = height;
        rs->png_size = IMLIB_IMAGE_MAX_SIZE(f_size(fp));

        img->w = rs->png_w;
        img->h = rs->png_h;
        img->size = rs->png_size;
        img->pixfmt = PIXFORMAT_PNG;
    } else {
        ff_file_corrupted(fp);
    }
}

// This function reads the pixel values of an image.
void png_read_pixels(FIL *fp, image_t *img)
{
    file_seek(fp, 0);
    read_data(fp, img->pixels, img->size);
}

void png_read(image_t *img, const char *path)
{
    FIL fp;
    png_read_settings_t rs;

    file_read_open(&fp, path);

    // Do not use file_buffer_on() here.
    png_read_geometry(&fp, img, path, &rs);

    if (!img->pixels) {
        img->pixels = xalloc(img->size);
    }

    png_read_pixels(&fp, img);
    file_close(&fp);
}

void png_write(image_t *img, const char *path)
{
    FIL fp;
printf("Entering png_write\n");
    file_write_open(&fp, path);
    if (img->pixfmt == PIXFORMAT_PNG) {
printf("pixfmt == PIXFORMAT_PNG\n");
        write_data(&fp, img->pixels, img->size);
    } else {
        image_t out = { .w=img->w, .h=img->h, .pixfmt=PIXFORMAT_PNG, .size=0, .pixels=NULL }; // alloc in png compress
printf("About to call png_compress\n");
        png_compress(img, &out);
        write_data(&fp, out.pixels, out.size);
        fb_free(); // frees alloc in png_compress()
    }
    file_close(&fp);
}
#endif //IMLIB_ENABLE_IMAGE_FILE_IO)
