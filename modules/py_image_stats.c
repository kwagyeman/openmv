/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright (C) 2013-2026 OpenMV, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Image statistics Python module.
 */
#include "imlib_config.h"

#include "py/nlr.h"
#include "py/obj.h"
#include "py/objlist.h"
#include "py/objtuple.h"
#include "py/runtime.h"
#include "py/mphal.h"

#include "imlib.h"
#include "umalloc.h"
#include "py_assert.h"
#include "py_helper.h"
#include "py_image.h"
#include "py_image_stats.h"

//////////////
// Get Methods
//////////////

#ifdef IMLIB_ENABLE_GET_SIMILARITY
// Similarity Object //
typedef struct py_similarity_obj {
    mp_obj_base_t base;
    mp_obj_t avg, std, min, max;
} py_similarity_obj_t;

static void py_similarity_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    py_similarity_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print,
              "{\"mean\":%f, \"stdev\":%f, \"min\":%f, \"max\":%f}",
              mp_obj_get_float_to_d(self->avg),
              mp_obj_get_float_to_d(self->std),
              mp_obj_get_float_to_d(self->min),
              mp_obj_get_float_to_d(self->max));
}

static void py_similarity_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest) {
    py_similarity_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (dest[0] == MP_OBJ_NULL) {
        switch (attr) {
            case MP_QSTR_mean:
                dest[0] = self->avg;
                break;
            case MP_QSTR_stdev:
                dest[0] = self->std;
                break;
            case MP_QSTR_min:
                dest[0] = self->min;
                break;
            case MP_QSTR_max:
                dest[0] = self->max;
                break;
            default:
                // Continue lookup in locals_dict.
                dest[1] = MP_OBJ_SENTINEL;
                break;
        }
    }
}

static MP_DEFINE_CONST_OBJ_TYPE(
    py_similarity_type,
    MP_QSTR_similarity,
    MP_TYPE_FLAG_NONE,
    attr, py_similarity_attr,
    print, py_similarity_print
    );

static mp_obj_t py_image_get_similarity(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {
        ARG_image, ARG_x, ARG_y, ARG_x_scale, ARG_y_scale, ARG_roi,
        ARG_channel, ARG_alpha, ARG_color_palette, ARG_alpha_palette, ARG_hint, ARG_dssim
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_image, MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_x, MP_ARG_INT,  {.u_int = 0 } },
        { MP_QSTR_y, MP_ARG_INT,  {.u_int = 0 } },
        { MP_QSTR_x_scale, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_y_scale, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_roi, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_rgb_channel, MP_ARG_INT | MP_ARG_KW_ONLY,  {.u_int = -1 } },
        { MP_QSTR_alpha, MP_ARG_INT | MP_ARG_KW_ONLY,  {.u_int = 255 } },
        { MP_QSTR_color_palette, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_alpha_palette, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_hint, MP_ARG_INT | MP_ARG_KW_ONLY,  {.u_int = 0 } },
        { MP_QSTR_dssim, MP_ARG_BOOL | MP_ARG_KW_ONLY, {.u_bool = false } },
    };

    // Parse args.
    image_t *image = py_helper_arg_to_image(pos_args[0], ARG_IMAGE_MUTABLE);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    image_t *other = py_helper_arg_to_image(args[ARG_image].u_obj, ARG_IMAGE_ANY | ARG_IMAGE_ALLOC);
    rectangle_t roi = py_helper_arg_to_roi(args[ARG_roi].u_obj, other);

    if (args[ARG_channel].u_int < -1 || args[ARG_channel].u_int > 2) {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("RGB channel can be 0, 1, or 2"));
    }

    if (args[ARG_alpha].u_int < 0 || args[ARG_alpha].u_int > 255) {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Alpha ranges between 0 and 255"));
    }

    float x_scale = 1.0f;
    float y_scale = 1.0f;
    py_helper_arg_to_scale(args[ARG_x_scale].u_obj, args[ARG_y_scale].u_obj, &x_scale, &y_scale);

    const uint16_t *color_palette = py_helper_arg_to_palette(args[ARG_color_palette].u_obj, PIXFORMAT_RGB565);
    const uint8_t *alpha_palette = py_helper_arg_to_palette(args[ARG_alpha_palette].u_obj, PIXFORMAT_GRAYSCALE);

    float avg = 0.0f, std = 0.0f, min = 0.0f, max = 0.0f;
    imlib_get_similarity(image, other, args[ARG_x].u_int, args[ARG_y].u_int, x_scale, y_scale, &roi,
                         args[ARG_channel].u_int, args[ARG_alpha].u_int, color_palette, alpha_palette,
                         args[ARG_hint].u_int | IMAGE_HINT_BLACK_BACKGROUND, args[ARG_dssim].u_bool,
                         &avg, &std, &min, &max);

    py_similarity_obj_t *o = m_new_obj(py_similarity_obj_t);
    o->base.type = &py_similarity_type;
    o->avg = mp_obj_new_float(avg);
    o->std = mp_obj_new_float(std);
    o->min = mp_obj_new_float(min);
    o->max = mp_obj_new_float(max);
    return o;
}
MP_DEFINE_CONST_FUN_OBJ_KW(py_image_get_similarity_obj, 1, py_image_get_similarity);
#endif // IMLIB_ENABLE_GET_SIMILARITY

// Statistics Object //
typedef struct py_statistics_obj {
    mp_obj_base_t base;
    pixformat_t pixfmt;
    mp_obj_t LMean, LMedian, LMode, LSTDev, LMin, LMax, LLQ, LUQ,
             AMean, AMedian, AMode, ASTDev, AMin, AMax, ALQ, AUQ,
             BMean, BMedian, BMode, BSTDev, BMin, BMax, BLQ, BUQ;
} py_statistics_obj_t;

static void py_statistics_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    py_statistics_obj_t *self = MP_OBJ_TO_PTR(self_in);
    switch (self->pixfmt) {
        case PIXFORMAT_BINARY: {
            mp_printf(print,
                      "{\"mean\":%d, \"median\":%d, \"mode\":%d, \"stdev\":%d, \"min\":%d, \"max\":%d, \"lq\":%d, \"uq\":%d}",
                      mp_obj_get_int(self->LMean),
                      mp_obj_get_int(self->LMedian),
                      mp_obj_get_int(self->LMode),
                      mp_obj_get_int(self->LSTDev),
                      mp_obj_get_int(self->LMin),
                      mp_obj_get_int(self->LMax),
                      mp_obj_get_int(self->LLQ),
                      mp_obj_get_int(self->LUQ));
            break;
        }
        case PIXFORMAT_GRAYSCALE: {
            mp_printf(print,
                      "{\"mean\":%d, \"median\":%d, \"mode\":%d, \"stdev\":%d, \"min\":%d, \"max\":%d, \"lq\":%d, \"uq\":%d}",
                      mp_obj_get_int(self->LMean),
                      mp_obj_get_int(self->LMedian),
                      mp_obj_get_int(self->LMode),
                      mp_obj_get_int(self->LSTDev),
                      mp_obj_get_int(self->LMin),
                      mp_obj_get_int(self->LMax),
                      mp_obj_get_int(self->LLQ),
                      mp_obj_get_int(self->LUQ));
            break;
        }
        case PIXFORMAT_RGB565: {
            mp_printf(print,
                      "{\"l_mean\":%d, \"l_median\":%d, \"l_mode\":%d, \"l_stdev\":%d, \"l_min\":%d, \"l_max\":%d, \"l_lq\":%d, \"l_uq\":%d,"
                      " \"a_mean\":%d, \"a_median\":%d, \"a_mode\":%d, \"a_stdev\":%d, \"a_min\":%d, \"a_max\":%d, \"a_lq\":%d, \"a_uq\":%d,"
                      " \"b_mean\":%d, \"b_median\":%d, \"b_mode\":%d, \"b_stdev\":%d, \"b_min\":%d, \"b_max\":%d, \"b_lq\":%d, \"b_uq\":%d}",
                      mp_obj_get_int(self->LMean),
                      mp_obj_get_int(self->LMedian),
                      mp_obj_get_int(self->LMode),
                      mp_obj_get_int(self->LSTDev),
                      mp_obj_get_int(self->LMin),
                      mp_obj_get_int(self->LMax),
                      mp_obj_get_int(self->LLQ),
                      mp_obj_get_int(self->LUQ),
                      mp_obj_get_int(self->AMean),
                      mp_obj_get_int(self->AMedian),
                      mp_obj_get_int(self->AMode),
                      mp_obj_get_int(self->ASTDev),
                      mp_obj_get_int(self->AMin),
                      mp_obj_get_int(self->AMax),
                      mp_obj_get_int(self->ALQ),
                      mp_obj_get_int(self->AUQ),
                      mp_obj_get_int(self->BMean),
                      mp_obj_get_int(self->BMedian),
                      mp_obj_get_int(self->BMode),
                      mp_obj_get_int(self->BSTDev),
                      mp_obj_get_int(self->BMin),
                      mp_obj_get_int(self->BMax),
                      mp_obj_get_int(self->BLQ),
                      mp_obj_get_int(self->BUQ));
            break;
        }
        default: {
            mp_printf(print, "{}");
            break;
        }
    }
}

static void py_statistics_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest) {
    py_statistics_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (dest[0] == MP_OBJ_NULL) {
        switch (attr) {
            case MP_QSTR_mean:
            case MP_QSTR_l_mean:
                dest[0] = self->LMean;
                break;
            case MP_QSTR_median:
            case MP_QSTR_l_median:
                dest[0] = self->LMedian;
                break;
            case MP_QSTR_mode:
            case MP_QSTR_l_mode:
                dest[0] = self->LMode;
                break;
            case MP_QSTR_stdev:
            case MP_QSTR_l_stdev:
                dest[0] = self->LSTDev;
                break;
            case MP_QSTR_min:
            case MP_QSTR_l_min:
                dest[0] = self->LMin;
                break;
            case MP_QSTR_max:
            case MP_QSTR_l_max:
                dest[0] = self->LMax;
                break;
            case MP_QSTR_lq:
            case MP_QSTR_l_lq:
                dest[0] = self->LLQ;
                break;
            case MP_QSTR_uq:
            case MP_QSTR_l_uq:
                dest[0] = self->LUQ;
                break;
            case MP_QSTR_a_mean:
                dest[0] = self->AMean;
                break;
            case MP_QSTR_a_median:
                dest[0] = self->AMedian;
                break;
            case MP_QSTR_a_mode:
                dest[0] = self->AMode;
                break;
            case MP_QSTR_a_stdev:
                dest[0] = self->ASTDev;
                break;
            case MP_QSTR_a_min:
                dest[0] = self->AMin;
                break;
            case MP_QSTR_a_max:
                dest[0] = self->AMax;
                break;
            case MP_QSTR_a_lq:
                dest[0] = self->ALQ;
                break;
            case MP_QSTR_a_uq:
                dest[0] = self->AUQ;
                break;
            case MP_QSTR_b_mean:
                dest[0] = self->BMean;
                break;
            case MP_QSTR_b_median:
                dest[0] = self->BMedian;
                break;
            case MP_QSTR_b_mode:
                dest[0] = self->BMode;
                break;
            case MP_QSTR_b_stdev:
                dest[0] = self->BSTDev;
                break;
            case MP_QSTR_b_min:
                dest[0] = self->BMin;
                break;
            case MP_QSTR_b_max:
                dest[0] = self->BMax;
                break;
            case MP_QSTR_b_lq:
                dest[0] = self->BLQ;
                break;
            case MP_QSTR_b_uq:
                dest[0] = self->BUQ;
                break;
            default:
                // Continue lookup in locals_dict.
                dest[1] = MP_OBJ_SENTINEL;
                break;
        }
    }
}

static MP_DEFINE_CONST_OBJ_TYPE(
    py_statistics_type,
    MP_QSTR_statistics,
    MP_TYPE_FLAG_NONE,
    attr, py_statistics_attr,
    print, py_statistics_print
    );

// Percentile Object //
typedef struct py_percentile_obj {
    mp_obj_base_t base;
    pixformat_t pixfmt;
    mp_obj_t LValue, AValue, BValue;
} py_percentile_obj_t;

static void py_percentile_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    py_percentile_obj_t *self = MP_OBJ_TO_PTR(self_in);
    switch (self->pixfmt) {
        case PIXFORMAT_BINARY: {
            mp_printf(print, "{\"value\":%d}",
                      mp_obj_get_int(self->LValue));
            break;
        }
        case PIXFORMAT_GRAYSCALE: {
            mp_printf(print, "{\"value\":%d}",
                      mp_obj_get_int(self->LValue));
            break;
        }
        case PIXFORMAT_RGB565: {
            mp_printf(print, "{\"l_value:%d\", \"a_value\":%d, \"b_value\":%d}",
                      mp_obj_get_int(self->LValue),
                      mp_obj_get_int(self->AValue),
                      mp_obj_get_int(self->BValue));
            break;
        }
        default: {
            mp_printf(print, "{}");
            break;
        }
    }
}

static void py_percentile_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest) {
    py_percentile_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (dest[0] == MP_OBJ_NULL) {
        switch (attr) {
            case MP_QSTR_value:
            case MP_QSTR_l_value:
                dest[0] = self->LValue;
                break;
            case MP_QSTR_a_value:
                dest[0] = self->AValue;
                break;
            case MP_QSTR_b_value:
                dest[0] = self->BValue;
                break;
            default:
                // Continue lookup in locals_dict.
                dest[1] = MP_OBJ_SENTINEL;
                break;
        }
    }
}

static MP_DEFINE_CONST_OBJ_TYPE(
    py_percentile_type,
    MP_QSTR_percentile,
    MP_TYPE_FLAG_NONE,
    attr, py_percentile_attr,
    print, py_percentile_print
    );

// Threshold Object //
typedef struct py_threshold_obj {
    mp_obj_base_t base;
    pixformat_t pixfmt;
    mp_obj_t LValue, AValue, BValue;
} py_threshold_obj_t;

static void py_threshold_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    py_threshold_obj_t *self = MP_OBJ_TO_PTR(self_in);
    switch (self->pixfmt) {
        case PIXFORMAT_BINARY: {
            mp_printf(print, "{\"value\":%d}",
                      mp_obj_get_int(self->LValue));
            break;
        }
        case PIXFORMAT_GRAYSCALE: {
            mp_printf(print, "{\"value\":%d}",
                      mp_obj_get_int(self->LValue));
            break;
        }
        case PIXFORMAT_RGB565: {
            mp_printf(print, "{\"l_value\":%d, \"a_value\":%d, \"b_value\":%d}",
                      mp_obj_get_int(self->LValue),
                      mp_obj_get_int(self->AValue),
                      mp_obj_get_int(self->BValue));
            break;
        }
        default: {
            mp_printf(print, "{}");
            break;
        }
    }
}

static void py_threshold_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest) {
    py_threshold_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (dest[0] == MP_OBJ_NULL) {
        switch (attr) {
            case MP_QSTR_value:
            case MP_QSTR_l_value:
                dest[0] = self->LValue;
                break;
            case MP_QSTR_a_value:
                dest[0] = self->AValue;
                break;
            case MP_QSTR_b_value:
                dest[0] = self->BValue;
                break;
            default:
                // Continue lookup in locals_dict.
                dest[1] = MP_OBJ_SENTINEL;
                break;
        }
    }
}

static MP_DEFINE_CONST_OBJ_TYPE(
    py_threshold_type,
    MP_QSTR_threshold,
    MP_TYPE_FLAG_NONE,
    attr, py_threshold_attr,
    print, py_threshold_print
    );

// Histogram Object //
typedef struct py_histogram_obj {
    mp_obj_base_t base;
    pixformat_t pixfmt;
    mp_obj_t LBins, ABins, BBins;
} py_histogram_obj_t;

static void py_histogram_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    py_histogram_obj_t *self = MP_OBJ_TO_PTR(self_in);
    switch (self->pixfmt) {
        case PIXFORMAT_BINARY: {
            mp_printf(print, "{\"bins\":");
            mp_obj_print_helper(print, self->LBins, kind);
            mp_printf(print, "}");
            break;
        }
        case PIXFORMAT_GRAYSCALE: {
            mp_printf(print, "{\"bins\":");
            mp_obj_print_helper(print, self->LBins, kind);
            mp_printf(print, "}");
            break;
        }
        case PIXFORMAT_RGB565: {
            mp_printf(print, "{\"l_bins\":");
            mp_obj_print_helper(print, self->LBins, kind);
            mp_printf(print, ", \"a_bins\":");
            mp_obj_print_helper(print, self->ABins, kind);
            mp_printf(print, ", \"b_bins\":");
            mp_obj_print_helper(print, self->BBins, kind);
            mp_printf(print, "}");
            break;
        }
        default: {
            mp_printf(print, "{}");
            break;
        }
    }
}

static void py_histogram_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest) {
    py_histogram_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (dest[0] == MP_OBJ_NULL) {
        switch (attr) {
            case MP_QSTR_bins:
            case MP_QSTR_l_bins:
                dest[0] = self->LBins;
                break;
            case MP_QSTR_a_bins:
                dest[0] = self->ABins;
                break;
            case MP_QSTR_b_bins:
                dest[0] = self->BBins;
                break;
            default:
                // Continue lookup in locals_dict.
                dest[1] = MP_OBJ_SENTINEL;
                break;
        }
    }
}

mp_obj_t py_histogram_get_percentile(mp_obj_t self_in, mp_obj_t percentile) {
    histogram_t hist;
    hist.LBinCount = ((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->LBins)->len;
    hist.ABinCount = ((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->ABins)->len;
    hist.BBinCount = ((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->BBins)->len;
    hist.LBins = uma_malloc(hist.LBinCount * sizeof(float), UMA_DTCM);
    hist.ABins = uma_malloc(hist.ABinCount * sizeof(float), UMA_DTCM);
    hist.BBins = uma_malloc(hist.BBinCount * sizeof(float), UMA_DTCM);

    for (int i = 0; i < hist.LBinCount; i++) {
        hist.LBins[i] = mp_obj_get_float_to_f(((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->LBins)->items[i]);
    }

    for (int i = 0; i < hist.ABinCount; i++) {
        hist.ABins[i] = mp_obj_get_float_to_f(((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->ABins)->items[i]);
    }

    for (int i = 0; i < hist.BBinCount; i++) {
        hist.BBins[i] = mp_obj_get_float_to_f(((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->BBins)->items[i]);
    }

    percentile_t p;
    imlib_get_percentile(&p, ((py_histogram_obj_t *) self_in)->pixfmt, &hist, mp_obj_get_float_to_f(percentile));

    py_percentile_obj_t *o = m_new_obj(py_percentile_obj_t);
    o->base.type = &py_percentile_type;
    o->pixfmt = ((py_histogram_obj_t *) self_in)->pixfmt;

    o->LValue = mp_obj_new_int(p.LValue);
    o->AValue = mp_obj_new_int(p.AValue);
    o->BValue = mp_obj_new_int(p.BValue);

    uma_free(hist.BBins);
    uma_free(hist.ABins);
    uma_free(hist.LBins);

    return o;
}
static MP_DEFINE_CONST_FUN_OBJ_2(py_histogram_get_percentile_obj, py_histogram_get_percentile);

mp_obj_t py_histogram_get_threshold(mp_obj_t self_in) {
    histogram_t hist;
    hist.LBinCount = ((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->LBins)->len;
    hist.ABinCount = ((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->ABins)->len;
    hist.BBinCount = ((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->BBins)->len;
    hist.LBins = uma_malloc(hist.LBinCount * sizeof(float), UMA_DTCM);
    hist.ABins = uma_malloc(hist.ABinCount * sizeof(float), UMA_DTCM);
    hist.BBins = uma_malloc(hist.BBinCount * sizeof(float), UMA_DTCM);

    for (int i = 0; i < hist.LBinCount; i++) {
        hist.LBins[i] = mp_obj_get_float_to_f(((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->LBins)->items[i]);
    }

    for (int i = 0; i < hist.ABinCount; i++) {
        hist.ABins[i] = mp_obj_get_float_to_f(((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->ABins)->items[i]);
    }

    for (int i = 0; i < hist.BBinCount; i++) {
        hist.BBins[i] = mp_obj_get_float_to_f(((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->BBins)->items[i]);
    }

    threshold_t t;
    imlib_get_threshold(&t, ((py_histogram_obj_t *) self_in)->pixfmt, &hist);

    py_threshold_obj_t *o = m_new_obj(py_threshold_obj_t);
    o->base.type = &py_threshold_type;
    o->pixfmt = ((py_threshold_obj_t *) self_in)->pixfmt;

    o->LValue = mp_obj_new_int(t.LValue);
    o->AValue = mp_obj_new_int(t.AValue);
    o->BValue = mp_obj_new_int(t.BValue);

    uma_free(hist.BBins);
    uma_free(hist.ABins);
    uma_free(hist.LBins);

    return o;
}
static MP_DEFINE_CONST_FUN_OBJ_1(py_histogram_get_threshold_obj, py_histogram_get_threshold);

mp_obj_t py_histogram_get_statistics(mp_obj_t self_in) {
    histogram_t hist;
    hist.LBinCount = ((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->LBins)->len;
    hist.ABinCount = ((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->ABins)->len;
    hist.BBinCount = ((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->BBins)->len;
    hist.LBins = uma_malloc(hist.LBinCount * sizeof(float), UMA_DTCM);
    hist.ABins = uma_malloc(hist.ABinCount * sizeof(float), UMA_DTCM);
    hist.BBins = uma_malloc(hist.BBinCount * sizeof(float), UMA_DTCM);

    for (int i = 0; i < hist.LBinCount; i++) {
        hist.LBins[i] = mp_obj_get_float_to_f(((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->LBins)->items[i]);
    }

    for (int i = 0; i < hist.ABinCount; i++) {
        hist.ABins[i] = mp_obj_get_float_to_f(((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->ABins)->items[i]);
    }

    for (int i = 0; i < hist.BBinCount; i++) {
        hist.BBins[i] = mp_obj_get_float_to_f(((mp_obj_list_t *) ((py_histogram_obj_t *) self_in)->BBins)->items[i]);
    }

    statistics_t stats;
    imlib_get_statistics(&stats, ((py_histogram_obj_t *) self_in)->pixfmt, &hist);

    py_statistics_obj_t *o = m_new_obj(py_statistics_obj_t);
    o->base.type = &py_statistics_type;
    o->pixfmt = ((py_histogram_obj_t *) self_in)->pixfmt;

    o->LMean = mp_obj_new_int(stats.LMean);
    o->LMedian = mp_obj_new_int(stats.LMedian);
    o->LMode = mp_obj_new_int(stats.LMode);
    o->LSTDev = mp_obj_new_int(stats.LSTDev);
    o->LMin = mp_obj_new_int(stats.LMin);
    o->LMax = mp_obj_new_int(stats.LMax);
    o->LLQ = mp_obj_new_int(stats.LLQ);
    o->LUQ = mp_obj_new_int(stats.LUQ);
    o->AMean = mp_obj_new_int(stats.AMean);
    o->AMedian = mp_obj_new_int(stats.AMedian);
    o->AMode = mp_obj_new_int(stats.AMode);
    o->ASTDev = mp_obj_new_int(stats.ASTDev);
    o->AMin = mp_obj_new_int(stats.AMin);
    o->AMax = mp_obj_new_int(stats.AMax);
    o->ALQ = mp_obj_new_int(stats.ALQ);
    o->AUQ = mp_obj_new_int(stats.AUQ);
    o->BMean = mp_obj_new_int(stats.BMean);
    o->BMedian = mp_obj_new_int(stats.BMedian);
    o->BMode = mp_obj_new_int(stats.BMode);
    o->BSTDev = mp_obj_new_int(stats.BSTDev);
    o->BMin = mp_obj_new_int(stats.BMin);
    o->BMax = mp_obj_new_int(stats.BMax);
    o->BLQ = mp_obj_new_int(stats.BLQ);
    o->BUQ = mp_obj_new_int(stats.BUQ);

    uma_free(hist.BBins);
    uma_free(hist.ABins);
    uma_free(hist.LBins);

    return o;
}
static MP_DEFINE_CONST_FUN_OBJ_1(py_histogram_get_statistics_obj, py_histogram_get_statistics);

static const mp_rom_map_elem_t py_histogram_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_get_percentile), MP_ROM_PTR(&py_histogram_get_percentile_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_threshold), MP_ROM_PTR(&py_histogram_get_threshold_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_stats), MP_ROM_PTR(&py_histogram_get_statistics_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_statistics), MP_ROM_PTR(&py_histogram_get_statistics_obj) },
    { MP_ROM_QSTR(MP_QSTR_statistics), MP_ROM_PTR(&py_histogram_get_statistics_obj) }
};

static MP_DEFINE_CONST_DICT(py_histogram_locals_dict, py_histogram_locals_dict_table);

static MP_DEFINE_CONST_OBJ_TYPE(
    py_histogram_type,
    MP_QSTR_histogram,
    MP_TYPE_FLAG_NONE,
    attr, py_histogram_attr,
    print, py_histogram_print,
    locals_dict, &py_histogram_locals_dict
    );

static mp_obj_t py_image_get_histogram(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_thresholds, ARG_invert, ARG_roi, ARG_bins, ARG_l_bins, ARG_a_bins, ARG_b_bins, ARG_difference };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_thresholds,  MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_invert,      MP_ARG_BOOL | MP_ARG_KW_ONLY, {.u_bool = false} },
        { MP_QSTR_roi,         MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_bins,        MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = -1} },
        { MP_QSTR_l_bins,      MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = -1} },
        { MP_QSTR_a_bins,      MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = -1} },
        { MP_QSTR_b_bins,      MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = -1} },
        { MP_QSTR_difference,  MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_NONE} },
    };

    image_t *image = py_helper_arg_to_image(pos_args[0], ARG_IMAGE_MUTABLE);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    list_t thresholds;
    list_init(&thresholds, sizeof(color_thresholds_list_lnk_data_t));
    if (args[ARG_thresholds].u_obj != mp_const_none) {
        py_helper_arg_to_thresholds(args[ARG_thresholds].u_obj, &thresholds);
    }
    rectangle_t roi = py_helper_arg_to_roi(args[ARG_roi].u_obj, image);
    image_t *other = NULL;
    if (args[ARG_difference].u_obj != mp_const_none) {
        other = py_helper_arg_to_image(args[ARG_difference].u_obj, ARG_IMAGE_ANY | ARG_IMAGE_ALLOC);
    }

    histogram_t hist;
    switch (image->pixfmt) {
        case PIXFORMAT_BINARY: {
            int bins = (args[ARG_bins].u_int >= 0) ? args[ARG_bins].u_int : (COLOR_BINARY_MAX - COLOR_BINARY_MIN + 1);
            PY_ASSERT_TRUE_MSG(bins >= 2, "bins must be >= 2");
            hist.LBinCount = (args[ARG_l_bins].u_int >= 0) ? args[ARG_l_bins].u_int : bins;
            PY_ASSERT_TRUE_MSG(hist.LBinCount >= 2, "l_bins must be >= 2");
            hist.ABinCount = 0;
            hist.BBinCount = 0;
            hist.LBins = uma_malloc(hist.LBinCount * sizeof(float), UMA_DTCM);
            hist.ABins = NULL;
            hist.BBins = NULL;
            imlib_get_histogram(&hist, image, &roi, &thresholds, args[ARG_invert].u_bool, other);
            list_free(&thresholds);
            break;
        }
        case PIXFORMAT_GRAYSCALE: {
            int bins = (args[ARG_bins].u_int >= 0) ? args[ARG_bins].u_int : (COLOR_GRAYSCALE_MAX - COLOR_GRAYSCALE_MIN + 1);
            PY_ASSERT_TRUE_MSG(bins >= 2, "bins must be >= 2");
            hist.LBinCount = (args[ARG_l_bins].u_int >= 0) ? args[ARG_l_bins].u_int : bins;
            PY_ASSERT_TRUE_MSG(hist.LBinCount >= 2, "l_bins must be >= 2");
            hist.ABinCount = 0;
            hist.BBinCount = 0;
            hist.LBins = uma_malloc(hist.LBinCount * sizeof(float), UMA_DTCM);
            hist.ABins = NULL;
            hist.BBins = NULL;
            imlib_get_histogram(&hist, image, &roi, &thresholds, args[ARG_invert].u_bool, other);
            list_free(&thresholds);
            break;
        }
        case PIXFORMAT_RGB565: {
            int l_bins = (args[ARG_bins].u_int >= 0) ? args[ARG_bins].u_int : (COLOR_L_MAX - COLOR_L_MIN + 1);
            PY_ASSERT_TRUE_MSG(l_bins >= 2, "bins must be >= 2");
            hist.LBinCount = (args[ARG_l_bins].u_int >= 0) ? args[ARG_l_bins].u_int : l_bins;
            PY_ASSERT_TRUE_MSG(hist.LBinCount >= 2, "l_bins must be >= 2");
            int a_bins = (args[ARG_bins].u_int >= 0) ? args[ARG_bins].u_int : (COLOR_A_MAX - COLOR_A_MIN + 1);
            PY_ASSERT_TRUE_MSG(a_bins >= 2, "bins must be >= 2");
            hist.ABinCount = (args[ARG_a_bins].u_int >= 0) ? args[ARG_a_bins].u_int : a_bins;
            PY_ASSERT_TRUE_MSG(hist.ABinCount >= 2, "a_bins must be >= 2");
            int b_bins = (args[ARG_bins].u_int >= 0) ? args[ARG_bins].u_int : (COLOR_B_MAX - COLOR_B_MIN + 1);
            PY_ASSERT_TRUE_MSG(b_bins >= 2, "bins must be >= 2");
            hist.BBinCount = (args[ARG_b_bins].u_int >= 0) ? args[ARG_b_bins].u_int : b_bins;
            PY_ASSERT_TRUE_MSG(hist.BBinCount >= 2, "b_bins must be >= 2");
            hist.LBins = uma_malloc(hist.LBinCount * sizeof(float), UMA_DTCM);
            hist.ABins = uma_malloc(hist.ABinCount * sizeof(float), UMA_DTCM);
            hist.BBins = uma_malloc(hist.BBinCount * sizeof(float), UMA_DTCM);
            imlib_get_histogram(&hist, image, &roi, &thresholds, args[ARG_invert].u_bool, other);
            list_free(&thresholds);
            break;
        }
        default: {
            return MP_OBJ_NULL;
        }
    }

    py_histogram_obj_t *o = m_new_obj(py_histogram_obj_t);
    o->base.type = &py_histogram_type;
    o->pixfmt = image->pixfmt;

    o->LBins = mp_obj_new_list(hist.LBinCount, NULL);
    o->ABins = mp_obj_new_list(hist.ABinCount, NULL);
    o->BBins = mp_obj_new_list(hist.BBinCount, NULL);

    for (int i = 0; i < hist.LBinCount; i++) {
        ((mp_obj_list_t *) o->LBins)->items[i] = mp_obj_new_float(hist.LBins[i]);
    }

    for (int i = 0; i < hist.ABinCount; i++) {
        ((mp_obj_list_t *) o->ABins)->items[i] = mp_obj_new_float(hist.ABins[i]);
    }

    for (int i = 0; i < hist.BBinCount; i++) {
        ((mp_obj_list_t *) o->BBins)->items[i] = mp_obj_new_float(hist.BBins[i]);
    }

    uma_free(hist.BBins);
    uma_free(hist.ABins);
    uma_free(hist.LBins);

    return o;
}
MP_DEFINE_CONST_FUN_OBJ_KW(py_image_get_histogram_obj, 1, py_image_get_histogram);

static mp_obj_t py_image_get_statistics(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_thresholds, ARG_invert, ARG_roi, ARG_bins, ARG_l_bins, ARG_a_bins, ARG_b_bins, ARG_difference };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_thresholds,  MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_invert,      MP_ARG_BOOL | MP_ARG_KW_ONLY, {.u_bool = false} },
        { MP_QSTR_roi,         MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_bins,        MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = -1} },
        { MP_QSTR_l_bins,      MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = -1} },
        { MP_QSTR_a_bins,      MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = -1} },
        { MP_QSTR_b_bins,      MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = -1} },
        { MP_QSTR_difference,  MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_NONE} },
    };

    image_t *image = py_helper_arg_to_image(pos_args[0], ARG_IMAGE_MUTABLE);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    list_t thresholds;
    list_init(&thresholds, sizeof(color_thresholds_list_lnk_data_t));
    if (args[ARG_thresholds].u_obj != mp_const_none) {
        py_helper_arg_to_thresholds(args[ARG_thresholds].u_obj, &thresholds);
    }
    rectangle_t roi = py_helper_arg_to_roi(args[ARG_roi].u_obj, image);
    image_t *other = NULL;
    if (args[ARG_difference].u_obj != mp_const_none) {
        other = py_helper_arg_to_image(args[ARG_difference].u_obj, ARG_IMAGE_ANY | ARG_IMAGE_ALLOC);
    }

    histogram_t hist;
    switch (image->pixfmt) {
        case PIXFORMAT_BINARY: {
            int bins = (args[ARG_bins].u_int >= 0) ? args[ARG_bins].u_int : (COLOR_BINARY_MAX - COLOR_BINARY_MIN + 1);
            PY_ASSERT_TRUE_MSG(bins >= 2, "bins must be >= 2");
            hist.LBinCount = (args[ARG_l_bins].u_int >= 0) ? args[ARG_l_bins].u_int : bins;
            PY_ASSERT_TRUE_MSG(hist.LBinCount >= 2, "l_bins must be >= 2");
            hist.ABinCount = 0;
            hist.BBinCount = 0;
            hist.LBins = uma_malloc(hist.LBinCount * sizeof(float), UMA_DTCM);
            hist.ABins = NULL;
            hist.BBins = NULL;
            imlib_get_histogram(&hist, image, &roi, &thresholds, args[ARG_invert].u_bool, other);
            list_free(&thresholds);
            break;
        }
        case PIXFORMAT_GRAYSCALE: {
            int bins = (args[ARG_bins].u_int >= 0) ? args[ARG_bins].u_int : (COLOR_GRAYSCALE_MAX - COLOR_GRAYSCALE_MIN + 1);
            PY_ASSERT_TRUE_MSG(bins >= 2, "bins must be >= 2");
            hist.LBinCount = (args[ARG_l_bins].u_int >= 0) ? args[ARG_l_bins].u_int : bins;
            PY_ASSERT_TRUE_MSG(hist.LBinCount >= 2, "l_bins must be >= 2");
            hist.ABinCount = 0;
            hist.BBinCount = 0;
            hist.LBins = uma_malloc(hist.LBinCount * sizeof(float), UMA_DTCM);
            hist.ABins = NULL;
            hist.BBins = NULL;
            imlib_get_histogram(&hist, image, &roi, &thresholds, args[ARG_invert].u_bool, other);
            list_free(&thresholds);
            break;
        }
        case PIXFORMAT_RGB565: {
            int l_bins = (args[ARG_bins].u_int >= 0) ? args[ARG_bins].u_int : (COLOR_L_MAX - COLOR_L_MIN + 1);
            PY_ASSERT_TRUE_MSG(l_bins >= 2, "bins must be >= 2");
            hist.LBinCount = (args[ARG_l_bins].u_int >= 0) ? args[ARG_l_bins].u_int : l_bins;
            PY_ASSERT_TRUE_MSG(hist.LBinCount >= 2, "l_bins must be >= 2");
            int a_bins = (args[ARG_bins].u_int >= 0) ? args[ARG_bins].u_int : (COLOR_A_MAX - COLOR_A_MIN + 1);
            PY_ASSERT_TRUE_MSG(a_bins >= 2, "bins must be >= 2");
            hist.ABinCount = (args[ARG_a_bins].u_int >= 0) ? args[ARG_a_bins].u_int : a_bins;
            PY_ASSERT_TRUE_MSG(hist.ABinCount >= 2, "a_bins must be >= 2");
            int b_bins = (args[ARG_bins].u_int >= 0) ? args[ARG_bins].u_int : (COLOR_B_MAX - COLOR_B_MIN + 1);
            PY_ASSERT_TRUE_MSG(b_bins >= 2, "bins must be >= 2");
            hist.BBinCount = (args[ARG_b_bins].u_int >= 0) ? args[ARG_b_bins].u_int : b_bins;
            PY_ASSERT_TRUE_MSG(hist.BBinCount >= 2, "b_bins must be >= 2");
            hist.LBins = uma_malloc(hist.LBinCount * sizeof(float), UMA_DTCM);
            hist.ABins = uma_malloc(hist.ABinCount * sizeof(float), UMA_DTCM);
            hist.BBins = uma_malloc(hist.BBinCount * sizeof(float), UMA_DTCM);
            imlib_get_histogram(&hist, image, &roi, &thresholds, args[ARG_invert].u_bool, other);
            list_free(&thresholds);
            break;
        }
        default: {
            return MP_OBJ_NULL;
        }
    }

    statistics_t stats;
    imlib_get_statistics(&stats, image->pixfmt, &hist);

    py_statistics_obj_t *o = m_new_obj(py_statistics_obj_t);
    o->base.type = &py_statistics_type;
    o->pixfmt = image->pixfmt;

    o->LMean = mp_obj_new_int(stats.LMean);
    o->LMedian = mp_obj_new_int(stats.LMedian);
    o->LMode = mp_obj_new_int(stats.LMode);
    o->LSTDev = mp_obj_new_int(stats.LSTDev);
    o->LMin = mp_obj_new_int(stats.LMin);
    o->LMax = mp_obj_new_int(stats.LMax);
    o->LLQ = mp_obj_new_int(stats.LLQ);
    o->LUQ = mp_obj_new_int(stats.LUQ);
    o->AMean = mp_obj_new_int(stats.AMean);
    o->AMedian = mp_obj_new_int(stats.AMedian);
    o->AMode = mp_obj_new_int(stats.AMode);
    o->ASTDev = mp_obj_new_int(stats.ASTDev);
    o->AMin = mp_obj_new_int(stats.AMin);
    o->AMax = mp_obj_new_int(stats.AMax);
    o->ALQ = mp_obj_new_int(stats.ALQ);
    o->AUQ = mp_obj_new_int(stats.AUQ);
    o->BMean = mp_obj_new_int(stats.BMean);
    o->BMedian = mp_obj_new_int(stats.BMedian);
    o->BMode = mp_obj_new_int(stats.BMode);
    o->BSTDev = mp_obj_new_int(stats.BSTDev);
    o->BMin = mp_obj_new_int(stats.BMin);
    o->BMax = mp_obj_new_int(stats.BMax);
    o->BLQ = mp_obj_new_int(stats.BLQ);
    o->BUQ = mp_obj_new_int(stats.BUQ);

    uma_free(hist.BBins);
    uma_free(hist.ABins);
    uma_free(hist.LBins);

    return o;
}
MP_DEFINE_CONST_FUN_OBJ_KW(py_image_get_statistics_obj, 1, py_image_get_statistics);
