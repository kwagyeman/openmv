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

bool png_compress(image_t *src, image_t *dst, bool realloc)
{
    #if (TIME_PNG==1)
    mp_uint_t start = mp_hal_ticks_ms();
    #endif
    PNGIMAGE *pPNG;
    int rc, y, iPitch = 0;
    uint8_t *pPixels;
    const uint8_t pBinaryPalette[] = {0,0,0,0xff,0xff,0xff};
    
    if (!dst->data) {
        uint32_t size=0;
        dst->data = fb_alloc_all(&size, FB_ALLOC_PREFER_SIZE | FB_ALLOC_CACHE_ALIGN);
        dst->size = IMLIB_IMAGE_MAX_SIZE(size);
    }

    if (src->is_compressed) {
        return true;
    }

    pPNG = (PNGIMAGE *)fb_alloc(sizeof(PNGIMAGE), FB_ALLOC_PREFER_SIZE | FB_ALLOC_CACHE_ALIGN);
    if (pPNG == NULL)
        return true; // out of memory
    memset(pPNG, 0, sizeof(PNGIMAGE));
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
            iPitch = (src->w + 7) >> 3;
            pPNG->ucPixelType = PNG_PIXEL_INDEXED;
            memcpy(pPNG->ucPalette, pBinaryPalette, sizeof(pBinaryPalette));
            break;
        case PIXFORMAT_GRAYSCALE:
            iPitch = src->w;
            pPNG->ucBpp = 8;
            pPNG->ucPixelType = PNG_PIXEL_GRAYSCALE;
            break;
        case PIXFORMAT_RGB565:
            pPNG->ucBpp = 8;
            iPitch = src->w * 2;
            pPNG->ucPixelType = PNG_PIXEL_TRUECOLOR;
            break;
        case PIXFORMAT_YUV_ANY:
            pPNG->ucBpp = 8;
            iPitch = src->w;
            pPNG->ucPixelType = PNG_PIXEL_TRUECOLOR;
            break;
        case PIXFORMAT_BAYER_ANY:
            pPNG->ucBpp = 8;
            iPitch = src->w;
            pPNG->ucPixelType = PNG_PIXEL_TRUECOLOR;
            break;
    }
    // Encode the image a line at a time
    rc = PNG_SUCCESS;
    for (y = 0; y < src->h && rc == PNG_SUCCESS; y++) {
        pPixels = &src->data[y * iPitch];
        rc = PNGAddLine(pPNG, pPixels, pPNG->y);
        pPNG->y++;
    }
    fb_free(); // Free PNGIMAGE structure
    if (rc != PNG_SUCCESS) { // an error occurred - probably not enough memory
        fb_free(); // free the output buffer
        return true;
    }
    dst->size = pPNG->iCompressedSize; // output file size
    
    // png support does not need to be crazy fast. Please use the draw image front
    // end to write the data line by line into the comrpession buffer. See the draw
    // image call back method which will let you make draw image output data in any
    // format. (jpeg compress its own get_mcu which is very fast but also duplicate
    // code - it was necessary because we stream jpeg images).
    //
    // Using this method gives you binary, grayscale, rgb565, bayer, and yuv support for
    // free. PLEASE DO NOT WRITE YOUR OWN PIXEL PROCESSING.

    // DO STUFF - SEE JPEG COMPRESS CODE for how to mimick.

    #if (TIME_PNG==1)
    printf("time: %lums\n", mp_hal_ticks_ms() - start);
    #endif

    return false;
}

void png_decompress(image_t *dst, image_t *src)
{
PNG_DEC_IMAGE *pPNG;
int iSize, rc;

    printf("In png_decompress\n");
    pPNG = (PNG_DEC_IMAGE *)fb_alloc(sizeof(PNG_DEC_IMAGE), FB_ALLOC_PREFER_SIZE | FB_ALLOC_CACHE_ALIGN);
    if (pPNG == NULL)
        return; // DEBUG - out of memory
    printf("allocated PNG_DEC_IMAGE structure\n");
    memset(pPNG, 0, sizeof(PNG_DEC_IMAGE));
    rc = PNGDEC_openRAM(pPNG, src->data, src->size, NULL);
    if (rc != PNG_SUCCESS) {
       fb_free(); // DEBUG - throw error here
       printf("PNGDEC_openRAM failed!\n");
       return;
    }
    printf("PNGDEC_openRAM succeeded!\n");
    iSize = PNGDEC_getBufferSize(pPNG);
    dst->data = (uint8_t *)fb_alloc(iSize, FB_ALLOC_PREFER_SIZE | FB_ALLOC_CACHE_ALIGN);
    if (dst->data == NULL) {
       fb_free(); // free PNG structure
       return; // DEBUG - throw error here
    }
    rc = DecodePNG(pPNG, NULL, 0);
    if (rc != PNG_SUCCESS) {
       printf("DecodePNG failed!\n");
       fb_free();
       fb_free();
       return; // DEBUG - throw error here
    } else {
       printf("DecodePNG succeeded!\n");
    }

    // Handle like jpeg decompress... but, one method for BINARY, GRAYSCALE, RGB565.
    return; // good decode
}

#if defined(IMLIB_ENABLE_IMAGE_FILE_IO)
// This function inits the geometry values of an image.
void png_read_geometry(FIL *fp, image_t *img, const char *path, png_read_settings_t *rs)
{
    for (;;) {
        uint32_t header;
        file_seek(fp, 12); // start of IHDR
        read_long(fp, &header);
        if (header == 0x52444849) // IHDR
        {   
            uint32_t width, height;
            read_long(fp, &width);
            read_long(fp, &height);
            width = __builtin_bswap32(width);
            height = __builtin_bswap32(height);
printf("PNG geometry %d x %d\n", (int)width, (int)height);
            rs->png_w   = width;
            rs->png_h   = height;
            rs->png_size = IMLIB_IMAGE_MAX_SIZE(f_size(fp));

            img->w      = rs->png_w;
            img->h      = rs->png_h;
            img->size   = rs->png_size;
            img->pixfmt = PIXFORMAT_PNG;
            return;
        } else {
            ff_file_corrupted(fp);
        }
    }
} /* png_read_geometry() */

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
    file_write_open(&fp, path);
    if (img->pixfmt == PIXFORMAT_PNG) {
        write_data(&fp, img->pixels, img->size);
    } else {
        image_t out = { .w=img->w, .h=img->h, .pixfmt=PIXFORMAT_PNG, .size=0, .pixels=NULL }; // alloc in png compress
        // When png_compress needs more memory than in currently allocated it
        // will try to realloc. MP will detect that the pointer is outside of
        // the heap and return NULL which will cause an out of memory error.
        png_compress(img, &out, false);
        write_data(&fp, out.pixels, out.size);
        fb_free(); // frees alloc in png_compress()
    }
    file_close(&fp);
}
#endif //IMLIB_ENABLE_IMAGE_FILE_IO)
