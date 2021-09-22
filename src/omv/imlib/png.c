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

#define TIME_PNG   (0)
#if (TIME_PNG == 1)
#include "py/mphal.h"
#endif

bool png_compress(image_t *src, image_t *dst, bool realloc)
{
    #if (TIME_PNG==1)
    mp_uint_t start = mp_hal_ticks_ms();
    #endif

    if (!dst->data) {
        uint32_t size=0;
        dst->data = fb_alloc_all(&size, FB_ALLOC_PREFER_SIZE | FB_ALLOC_CACHE_ALIGN);
        dst->size = IMLIB_IMAGE_MAX_SIZE(size);
    }

    if (src->is_compressed) {
        return true;
    }

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
    // Handle like jpeg decompress... but, one method for BINARY, GRAYSCALE, RGB565.
}

#if defined(IMLIB_ENABLE_IMAGE_FILE_IO)
// This function inits the geometry values of an image.
void png_read_geometry(FIL *fp, image_t *img, const char *path, png_read_settings_t *rs)
{
    // Larry, see the jpeg version for how to populate the png read settings
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
