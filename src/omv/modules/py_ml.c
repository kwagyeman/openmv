/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2024 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2024 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * Python Machine Learning Module.
 */
#include <stdio.h>
#include "py/runtime.h"
#include "py/obj.h"
#include "py/objlist.h"
#include "py/objtuple.h"
#include "py/binary.h"

#include "py_helper.h"
#include "imlib_config.h"

#ifdef IMLIB_ENABLE_TFLM
#include "py_image.h"
#include "file_utils.h"
#include "py_ml.h"
#include "tflm_builtin_models.h"
#include "ulab/code/ndarray.h"

#define PY_ML_GRAYSCALE_RANGE   ((COLOR_GRAYSCALE_MAX) -(COLOR_GRAYSCALE_MIN))
#define PY_ML_GRAYSCALE_MID     (((PY_ML_GRAYSCALE_RANGE) +1) / 2)

STATIC const char *py_ml_map_dtype(py_ml_dtype_t dtype) {
    if (dtype == PY_ML_DTYPE_UINT8) {
        return "uint8";
    } else if (dtype == PY_ML_DTYPE_INT8) {
        return "int8";
    } else if (dtype == PY_ML_DTYPE_INT16) {
        return "int16";
    } else {
        return "float";
    }
}

// TF Input/Output callback functions.
typedef mp_obj_t py_ml_output_data_t;

typedef struct _py_ml_input_callback_data {
    void *data;
    rectangle_t roi;
    py_ml_scale_t scale;
    float mean[3];
    float stdev[3];
} py_ml_input_data_t;

static size_t py_ml_tuple_sum(mp_obj_tuple_t *o) {
    if (o->len < 1) {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Unexpected tensor shape"));
    }

    size_t size = mp_obj_get_int(o->items[0]);
    for (size_t i = 1; i < o->len; i++) {
        size *= mp_obj_get_int(o->items[i]);
    }
    return size;
}

static void py_ml_tuple_hwc(mp_obj_tuple_t *o, size_t *h, size_t *w, size_t *c) {
    if (o->len != 1 || ((mp_obj_tuple_t *) MP_OBJ_TO_PTR(o->items[0]))->len != 4) {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Unexpected tensor shape"));
    }
    o = MP_OBJ_TO_PTR(o->items[0]);
    *h = mp_obj_get_int(o->items[1]);
    *w = mp_obj_get_int(o->items[2]);
    *c = mp_obj_get_int(o->items[3]);
}

STATIC void py_ml_input_callback(py_ml_model_obj_t *model, void *arg) {
    // TODO we assume that there's a single input.
    void *model_input = ml_backend_get_input(model, 0);
    py_ml_input_data_t *input_data = (py_ml_input_data_t *) arg;

    // TODO we assume that the input shape is (1, h, w, c)
    size_t input_height = 0, input_width = 0, input_channels = 0;
    py_ml_tuple_hwc(model->input_shape, &input_height, &input_width, &input_channels);

    int shift = (model->input_dtype == PY_ML_DTYPE_INT8) ? PY_ML_GRAYSCALE_MID : 0;
    float fscale = 1.0f, fadd = 0.0f;

    switch (input_data->scale) {
        case PY_ML_SCALE_0_1: // convert 0->255 to 0->1
            fscale = 1.0f / 255.0f;
            break;
        case PY_ML_SCALE_S1_1: // convert 0->255 to -1->1
            fscale = 2.0f / 255.0f;
            fadd = -1.0f;
            break;
        case PY_ML_SCALE_S128_127: // convert 0->255 to -128->127
            fadd = -128.0f;
            break;
        case PY_ML_SCALE_NONE: // convert 0->255 to 0->255
        default:
            break;
    }

    float fscale_r = fscale, fadd_r = fadd;
    float fscale_g = fscale, fadd_g = fadd;
    float fscale_b = fscale, fadd_b = fadd;

    // To normalize the input image we need to subtract the mean and divide by the standard deviation.
    // We can do this by applying the normalization to fscale and fadd outside the loop.
    // Red
    fadd_r = (fadd_r - input_data->mean[0]) / input_data->stdev[0];
    fscale_r /= input_data->stdev[0];

    // Green
    fadd_g = (fadd_g - input_data->mean[1]) / input_data->stdev[1];
    fscale_g /= input_data->stdev[1];

    // Blue
    fadd_b = (fadd_b - input_data->mean[2]) / input_data->stdev[2];
    fscale_b /= input_data->stdev[2];

    // Grayscale -> Y = 0.299R + 0.587G + 0.114B
    float mean = (input_data->mean[0] * 0.299f) + (input_data->mean[1] * 0.587f) + (input_data->mean[2] * 0.114f);
    float std = (input_data->stdev[0] * 0.299f) + (input_data->stdev[1] * 0.587f) + (input_data->stdev[2] * 0.114f);
    fadd = (fadd - mean) / std;
    fscale /= std;

    image_t dst_img;
    dst_img.w = input_width;
    dst_img.h = input_height;
    dst_img.data = (uint8_t *) model_input;

    if (input_channels == 1) {
        dst_img.pixfmt = PIXFORMAT_GRAYSCALE;
    } else if (input_channels == 3) {
        dst_img.pixfmt = PIXFORMAT_RGB565;
    } else {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Expected model input channels to be 1 or 3!"));
    }

    imlib_draw_image(&dst_img, input_data->data, 0, 0, 1.0f, 1.0f, &input_data->roi,
                     -1, 256, NULL, NULL, IMAGE_HINT_BILINEAR | IMAGE_HINT_CENTER |
                     IMAGE_HINT_SCALE_ASPECT_EXPAND | IMAGE_HINT_BLACK_BACKGROUND, NULL, NULL, NULL);

    int size = (input_width * input_height) - 1; // must be int per countdown loop

    if (input_channels == 1) {
        // GRAYSCALE
        if (model->input_dtype == PY_ML_DTYPE_FLOAT) {
            // convert u8 -> f32
            uint8_t *model_input_u8 = (uint8_t *) model_input;
            float *model_input_f32 = (float *) model_input;
            for (; size >= 0; size -= 1) {
                model_input_f32[size] = (model_input_u8[size] * fscale) + fadd;
            }
        } else {
            if (shift) {
                // convert u8 -> s8
                uint8_t *model_input_8 = (uint8_t *) model_input;
                #if (__ARM_ARCH > 6)
                for (; size >= 3; size -= 4) {
                    *((uint32_t *) (model_input_8 + size - 3)) ^= 0x80808080;
                }
                #endif
                for (; size >= 0; size -= 1) {
                    model_input_8[size] ^= PY_ML_GRAYSCALE_MID;
                }
            }
        }
    } else if (input_channels == 3) {
        // RGB888
        int rgb_size = size * 3; // must be int per countdown loop
        if (model->input_dtype == PY_ML_DTYPE_FLOAT) {
            uint16_t *model_input_u16 = (uint16_t *) model_input;
            float *model_input_f32 = (float *) model_input;
            for (; size >= 0; size -= 1, rgb_size -= 3) {
                int pixel = model_input_u16[size];
                model_input_f32[rgb_size] = (COLOR_RGB565_TO_R8(pixel) * fscale_r) + fadd_r;
                model_input_f32[rgb_size + 1] = (COLOR_RGB565_TO_G8(pixel) * fscale_g) + fadd_g;
                model_input_f32[rgb_size + 2] = (COLOR_RGB565_TO_B8(pixel) * fscale_b) + fadd_b;
            }
        } else {
            uint16_t *model_input_u16 = (uint16_t *) model_input;
            uint8_t *model_input_8 = (uint8_t *) model_input;
            for (; size >= 0; size -= 1, rgb_size -= 3) {
                int pixel = model_input_u16[size];
                model_input_8[rgb_size] = COLOR_RGB565_TO_R8(pixel) ^ shift;
                model_input_8[rgb_size + 1] = COLOR_RGB565_TO_G8(pixel) ^ shift;
                model_input_8[rgb_size + 2] = COLOR_RGB565_TO_B8(pixel) ^ shift;
            }
        }
    }
}

STATIC void py_ml_input_callback_regression(py_ml_model_obj_t *model, void *arg) {
    // TODO we assume that there's a single input.
    void *model_input = ml_backend_get_input(model, 0);
    py_ml_input_data_t *input_data = (py_ml_input_data_t *) arg;

    mp_obj_tuple_t *input_shape = MP_OBJ_TO_PTR(model->input_shape->items[0]);
    ndarray_obj_t *input_array = MP_OBJ_TO_PTR(*((mp_obj_t *) input_data->data));

    if (input_array->ndim != input_shape->len) {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Input shape does not match the model input shape"));
    }
    for (size_t i = 0; i < input_array->ndim; i++) {
        if (input_array->shape[i] != mp_obj_get_int(input_shape->items[i])) {
            mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Input shape does not match the model input shape"));
        }
    }

    if (model->input_dtype == PY_ML_DTYPE_FLOAT) {
        float *model_input_float = (float *) model_input;
        for (size_t i = 0; i < input_array->len; i++) {
            float value = ndarray_get_float_index(input_array->array, input_array->dtype, i);
            model_input_float[i] = value;
        }
    } else if (model->input_dtype == PY_ML_DTYPE_INT8) {
        int8_t *model_input_8 = (int8_t *) model_input;
        for (size_t i = 0; i < input_array->len; i++) {
            float value = ndarray_get_float_index(input_array->array, input_array->dtype, i);
            model_input_8[i] = (int8_t) ((value / model->input_scale) + model->input_zero_point);
        }
    } else if (model->input_dtype == PY_ML_DTYPE_UINT8) {
        uint8_t *model_input_8 = (uint8_t *) model_input;
        for (size_t i = 0; i < input_array->len; i++) {
            float value = ndarray_get_float_index(input_array->array, input_array->dtype, i);
            model_input_8[i] = (uint8_t) ((value / model->input_scale) + model->input_zero_point);
        }
    } else {
        int16_t *model_input_16 = (int16_t *) model_input;
        for (size_t i = 0; i < input_array->len; i++) {
            float value = ndarray_get_float_index(input_array->array, input_array->dtype, i);
            model_input_16[i] = (int16_t) ((value / model->input_scale) + model->input_zero_point);
        }
    }
}

STATIC void py_ml_output_callback(py_ml_model_obj_t *model, void *arg) {
    mp_obj_list_t *output_list = MP_OBJ_TO_PTR(mp_obj_new_list(model->outputs_size, NULL));
    for (size_t i = 0; i < model->outputs_size; i++) {
        void *model_output = ml_backend_get_output(model, i);
        size_t size = py_ml_tuple_sum(MP_OBJ_TO_PTR(model->output_shape->items[i]));
        mp_obj_tuple_t *output = MP_OBJ_TO_PTR(mp_obj_new_tuple(size, NULL));

        if (model->output_dtype == PY_ML_DTYPE_FLOAT) {
            for (size_t j = 0; j < size; j++) {
                output->items[j] = mp_obj_new_float(((float *) model_output)[j]);
            }
        } else if (model->output_dtype == PY_ML_DTYPE_INT8) {
            for (size_t j = 0; j < size; j++) {
                float v = (((int8_t *) model_output)[j] - model->output_zero_point);
                output->items[j] = mp_obj_new_float(v * model->output_scale);
            }
        } else if (model->output_dtype == PY_ML_DTYPE_UINT8) {
            for (size_t j = 0; j < size; j++) {
                float v = (((uint8_t *) model_output)[j] - model->output_zero_point);
                output->items[j] = mp_obj_new_float(v * model->output_scale);
            }
        } else {
            for (size_t j = 0; j < size; j++) {
                float v = (((int8_t *) model_output)[j] - model->output_zero_point);
                output->items[j] = mp_obj_new_float(v * model->output_scale);
            }
        }
        output_list->items[i] = MP_OBJ_FROM_PTR(output);
    }
    *((py_ml_output_data_t *) arg) = MP_OBJ_FROM_PTR(output_list);
}

// TF Model Object.
static const mp_obj_type_t py_ml_model_type;

STATIC void py_ml_model_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    py_ml_model_obj_t *self = MP_OBJ_TO_PTR(self_in);
    mp_printf(print,
              "{size: %d, ram: %d, inputs_size: %d, input_dtype: %s, input_scale: %f, input_zero_point: %d, "
              "outputs_size: %d output_dtype: %s, output_scale: %f, output_zero_point: %d}",
              self->size, self->memory_size, self->inputs_size, py_ml_map_dtype(self->input_dtype),
              (double) self->input_scale, self->input_zero_point, self->outputs_size, py_ml_map_dtype(self->output_dtype),
              (double) self->output_scale, self->output_zero_point);
}

STATIC mp_obj_t py_ml_model_predict(uint n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum { ARG_roi, ARG_callback, ARG_scale, ARG_mean, ARG_stdev };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_roi, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_callback, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_scale, MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = PY_ML_SCALE_0_1} },
        { MP_QSTR_mean, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_stdev, MP_ARG_OBJ | MP_ARG_KW_ONLY, {.u_rom_obj = MP_ROM_NONE} },
    };

    // Parse args.
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 2, pos_args + 2, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    py_ml_model_obj_t *model = MP_OBJ_TO_PTR(pos_args[0]);

    py_ml_input_data_t input_data = {
        .scale = args[ARG_scale].u_int,
        .mean = {0.0f, 0.0f, 0.0f},
        .stdev = {1.0f, 1.0f, 1.0f}
    };
    ml_backend_input_callback_t input_callback = py_ml_input_callback;

    py_ml_output_data_t output_data;
    ml_backend_output_callback_t output_callback = py_ml_output_callback;

    if (MP_OBJ_IS_TYPE(pos_args[1], &ulab_ndarray_type)) {
        input_data.data = (void *) &pos_args[1];
        input_callback = py_ml_input_callback_regression;
    } else if (MP_OBJ_IS_TYPE(pos_args[1], &py_image_type)) {
        input_data.data = py_helper_arg_to_image(pos_args[1], ARG_IMAGE_ANY);
        input_data.roi = py_helper_arg_to_roi(args[ARG_roi].u_obj, input_data.data);
        py_helper_arg_to_float_array(args[ARG_mean].u_obj, input_data.mean, 3);
        py_helper_arg_to_float_array(args[ARG_stdev].u_obj, input_data.stdev, 3);
    } else {
        mp_raise_msg(&mp_type_ValueError, MP_ERROR_TEXT("Unsupported input type"));
    }

    ml_backend_run_inference(model, input_callback, &input_data, output_callback, &output_data);

    if (args[ARG_callback].u_obj != mp_const_none) {
        mp_obj_t rect = mp_obj_new_tuple(4, (mp_obj_t []) { mp_obj_new_int(input_data.roi.x),
                                                            mp_obj_new_int(input_data.roi.y),
                                                            mp_obj_new_int(input_data.roi.w),
                                                            mp_obj_new_int(input_data.roi.h) });
        mp_obj_t fun_args[3] = { MP_OBJ_FROM_PTR(model), output_data, rect };
        if (!MP_OBJ_IS_TYPE(pos_args[1], &py_image_type)) {
            output_data = mp_call_function_n_kw(args[ARG_callback].u_obj, 2, 0, fun_args);
        } else {
            output_data = mp_call_function_n_kw(args[ARG_callback].u_obj, 3, 0, fun_args);
        }
    }

    return output_data;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(py_ml_model_predict_obj, 2, py_ml_model_predict);

STATIC void py_ml_model_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest) {
    py_ml_model_obj_t *self = MP_OBJ_TO_PTR(self_in);
    const char *str;
    if (dest[0] == MP_OBJ_NULL) {
        // Load attribute.
        switch (attr) {
            case MP_QSTR_len:
                dest[0] = mp_obj_new_int(self->size);
                break;
            case MP_QSTR_ram:
                dest[0] = mp_obj_new_int(self->memory_size);
                break;
            case MP_QSTR_input_shape:
                dest[0] = MP_OBJ_FROM_PTR(self->input_shape);
                break;
            case MP_QSTR_input_dtype:
                str = py_ml_map_dtype(self->input_dtype);
                dest[0] = mp_obj_new_str(str, strlen(str));
                break;
            case MP_QSTR_input_scale:
                dest[0] = mp_obj_new_float(self->input_scale);
                break;
            case MP_QSTR_input_zero_point:
                dest[0] = mp_obj_new_int(self->input_zero_point);
                break;
            case MP_QSTR_output_shape:
                dest[0] = MP_OBJ_FROM_PTR(self->output_shape);
                break;
            case MP_QSTR_output_dtype:
                str = py_ml_map_dtype(self->output_dtype);
                dest[0] = mp_obj_new_str(str, strlen(str));
                break;
            case MP_QSTR_output_scale:
                dest[0] = mp_obj_new_float(self->output_scale);
                break;
            case MP_QSTR_output_zero_point:
                dest[0] = mp_obj_new_int(self->output_zero_point);
                break;
            default:
                // Continue lookup in locals_dict.
                dest[1] = MP_OBJ_SENTINEL;
                break;
        }
    }
}

mp_obj_t py_ml_model_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_path, ARG_load_to_fb };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_path, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_load_to_fb, MP_ARG_INT | MP_ARG_KW_ONLY, {.u_bool = false } },
    };

    // Parse args.
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    fb_alloc_mark();

    const char *path = mp_obj_str_get_str(args[ARG_path].u_obj);

    py_ml_model_obj_t *model = m_new_obj_with_finaliser(py_ml_model_obj_t);
    model->base.type = &py_ml_model_type;
    model->data = NULL;
    model->fb_alloc = args[ARG_load_to_fb].u_int;
    mp_obj_list_t *labels = NULL;

    for (const tflm_builtin_model_t *_model = &tflm_builtin_models[0]; _model->name != NULL; _model++) {
        if (!strcmp(path, _model->name)) {
            // Load model data.
            model->size = _model->size;
            model->data = (unsigned char *) _model->data;

            // Load model labels
            labels = MP_OBJ_TO_PTR(mp_obj_new_list(_model->n_labels, NULL));
            for (int l = 0; l < _model->n_labels; l++) {
                const char *label = _model->labels[l];
                labels->items[l] = mp_obj_new_str(label, strlen(label));
            }
            break;
        }
    }

    if (model->data == NULL) {
        #if defined(IMLIB_ENABLE_IMAGE_FILE_IO)
        FIL fp;
        file_open(&fp, path, false, FA_READ | FA_OPEN_EXISTING);
        model->size = f_size(&fp);
        model->data = model->fb_alloc ? fb_alloc(model->size, FB_ALLOC_PREFER_SIZE) : xalloc(model->size);
        file_read(&fp, model->data, model->size);
        file_close(&fp);
        #else
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("Image I/O is not supported"));
        #endif
    }

    if (model->fb_alloc) {
        // The model's data will Not be free'd on exceptions.
        fb_alloc_mark_permanent();
    } else {
        fb_alloc_free_till_mark();
    }


    ml_backend_init_model(model);

    if (model->input_scale == 0.0f) {
        model->input_scale = 1.0;
    }

    if (model->output_scale == 0.0f) {
        model->output_scale = 1.0;
    }

    if (labels == NULL) {
        return MP_OBJ_FROM_PTR(model);
    } else {
        return mp_obj_new_tuple(2, (mp_obj_t []) {MP_OBJ_FROM_PTR(labels), MP_OBJ_FROM_PTR(model)});
    }
}

STATIC mp_obj_t py_ml_model_deinit(mp_obj_t self_in) {
    py_ml_model_obj_t *model = MP_OBJ_TO_PTR(self_in);
    if (model->fb_alloc) {
        fb_alloc_free_till_mark_past_mark_permanent();
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_ml_model_deinit_obj, py_ml_model_deinit);

STATIC const mp_rom_map_elem_t py_ml_model_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___del__),             MP_ROM_PTR(&py_ml_model_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_predict),             MP_ROM_PTR(&py_ml_model_predict_obj) },
};

STATIC MP_DEFINE_CONST_DICT(py_ml_model_locals_dict, py_ml_model_locals_dict_table);

STATIC MP_DEFINE_CONST_OBJ_TYPE(
    py_ml_model_type,
    MP_QSTR_ml_model,
    MP_TYPE_FLAG_NONE,
    attr, py_ml_model_attr,
    print, py_ml_model_print,
    make_new, py_ml_model_make_new,
    locals_dict, &py_ml_model_locals_dict
    );

extern const mp_obj_type_t py_ml_nms_type;

STATIC const mp_rom_map_elem_t py_ml_globals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),            MP_OBJ_NEW_QSTR(MP_QSTR_ml) },
    { MP_ROM_QSTR(MP_QSTR_Model),               MP_ROM_PTR(&py_ml_model_type) },
    { MP_ROM_QSTR(MP_QSTR_NMS),                 MP_ROM_PTR(&py_ml_nms_type) },
    { MP_ROM_QSTR(MP_QSTR_SCALE_NONE),          MP_ROM_INT(PY_ML_SCALE_NONE) },
    { MP_ROM_QSTR(MP_QSTR_SCALE_0_1),           MP_ROM_INT(PY_ML_SCALE_0_1) },
    { MP_ROM_QSTR(MP_QSTR_SCALE_S1_1),          MP_ROM_INT(PY_ML_SCALE_S1_1) },
    { MP_ROM_QSTR(MP_QSTR_SCALE_S128_127),      MP_ROM_INT(PY_ML_SCALE_S128_127) },
};

STATIC MP_DEFINE_CONST_DICT(py_ml_globals_dict, py_ml_globals_dict_table);

const mp_obj_module_t ml_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_t) &py_ml_globals_dict
};

// Alias for backwards compatibility
MP_REGISTER_EXTENSIBLE_MODULE(MP_QSTR_tf, ml_module);
MP_REGISTER_EXTENSIBLE_MODULE(MP_QSTR_ml, ml_module);
#endif // IMLIB_ENABLE_TFLM
