/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2013-2021 Ibrahim Abdelkader <iabdalkader@openmv.io>
 * Copyright (c) 2013-2021 Kwabena W. Agyeman <kwagyeman@openmv.io>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * This file contains image sensor driver utility functions and some default (weak)
 * implementations of common functions that can be replaced by port-specific drivers.
 */
#if MICROPY_PY_SENSOR
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "py/mphal.h"
#include "sensor.h"
#include "ov2640.h"
#include "ov5640.h"
#include "ov7725.h"
#include "ov7670.h"
#include "ov7690.h"
#include "ov9650.h"
#include "mt9v0xx.h"
#include "mt9m114.h"
#include "lepton.h"
#include "hm01b0.h"
#include "hm0360.h"
#include "paj6100.h"
#include "frogeye2020.h"
#include "gc2145.h"
#include "framebuffer.h"
#include "omv_boardconfig.h"
#include "omv_gpio.h"
#include "omv_i2c.h"
#include "unaligned_memcpy.h"

#ifndef OMV_ISC_MAX_DEVICES
#define OMV_ISC_MAX_DEVICES (5)
#endif

#ifndef __weak
#define __weak    __attribute__((weak))
#endif

#define SENSOR_TIMEOUT_MS   (3000)

// Sensor frame size/resolution table.
const int resolution[][2] = {
    {0,    0   },
    // C/SIF Resolutions
    {88,   72  },    /* QQCIF     */
    {176,  144 },    /* QCIF      */
    {352,  288 },    /* CIF       */
    {88,   60  },    /* QQSIF     */
    {176,  120 },    /* QSIF      */
    {352,  240 },    /* SIF       */
    // VGA Resolutions
    {40,   30  },    /* QQQQVGA   */
    {80,   60  },    /* QQQVGA    */
    {160,  120 },    /* QQVGA     */
    {320,  240 },    /* QVGA      */
    {640,  480 },    /* VGA       */
    {30,   20  },    /* HQQQQVGA  */
    {60,   40  },    /* HQQQVGA   */
    {120,  80  },    /* HQQVGA    */
    {240,  160 },    /* HQVGA     */
    {480,  320 },    /* HVGA      */
    // FFT Resolutions
    {64,   32  },    /* 64x32     */
    {64,   64  },    /* 64x64     */
    {128,  64  },    /* 128x64    */
    {128,  128 },    /* 128x128   */
    // Himax Resolutions
    {160,  160 },    /* 160x160   */
    {320,  320 },    /* 320x320   */
    // Other
    {128,  160 },    /* LCD       */
    {128,  160 },    /* QQVGA2    */
    {720,  480 },    /* WVGA      */
    {752,  480 },    /* WVGA2     */
    {800,  600 },    /* SVGA      */
    {1024, 768 },    /* XGA       */
    {1280, 768 },    /* WXGA      */
    {1280, 1024},    /* SXGA      */
    {1280, 960 },    /* SXGAM     */
    {1600, 1200},    /* UXGA      */
    {1280, 720 },    /* HD        */
    {1920, 1080},    /* FHD       */
    {2560, 1440},    /* QHD       */
    {2048, 1536},    /* QXGA      */
    {2560, 1600},    /* WQXGA     */
    {2592, 1944},    /* WQXGA2    */
};

__weak void sensor_init0() {
    // Reset the sensor state
    memset(&sensor, 0, sizeof(sensor_t));
}

__weak int sensor_init() {
    // Reset the sensor state
    memset(&sensor, 0, sizeof(sensor_t));
    return SENSOR_ERROR_CTL_UNSUPPORTED;
}

__weak int sensor_abort() {
    return SENSOR_ERROR_CTL_UNSUPPORTED;
}

__weak int sensor_reset() {
    // Disable any ongoing frame capture.
    sensor_abort();

    // Reset the sensor state
    sensor.sde = 0;
    sensor.pixformat = 0;
    sensor.framesize = 0;
    sensor.framerate = 0;
    sensor.last_frame_ms = 0;
    sensor.last_frame_ms_valid = false;
    sensor.gainceiling = 0;
    sensor.hmirror = false;
    sensor.vflip = false;
    sensor.transpose = false;
    #if MICROPY_PY_IMU
    sensor.auto_rotation = (sensor.chip_id == OV7690_ID);
    #else
    sensor.auto_rotation = false;
    #endif // MICROPY_PY_IMU
    sensor.vsync_callback = NULL;
    sensor.frame_callback = NULL;

    // Reset default color palette.
    sensor.color_palette = rainbow_table;

    sensor.disable_full_flush = false;

    // Restore shutdown state on reset.
    sensor_shutdown(false);

    // Disable the bus before reset.
    omv_i2c_enable(&sensor.i2c_bus, false);

    #if defined(DCMI_RESET_PIN)
    // Hard-reset the sensor
    if (sensor.reset_pol == ACTIVE_HIGH) {
        omv_gpio_write(DCMI_RESET_PIN, 1);
        mp_hal_delay_ms(10);
        omv_gpio_write(DCMI_RESET_PIN, 0);
    } else {
        omv_gpio_write(DCMI_RESET_PIN, 0);
        mp_hal_delay_ms(10);
        omv_gpio_write(DCMI_RESET_PIN, 1);
    }
    #endif

    mp_hal_delay_ms(20);

    // Re-enable the bus.
    omv_i2c_enable(&sensor.i2c_bus, true);

    // Call sensor-specific reset function
    if (sensor.reset != NULL
        && sensor.reset(&sensor) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    // Reset framebuffers
    framebuffer_reset_buffers();

    return 0;
}

static int sensor_detect() {
    uint8_t devs_list[OMV_ISC_MAX_DEVICES];
    int n_devs = omv_i2c_scan(&sensor.i2c_bus, devs_list, OMV_ARRAY_SIZE(devs_list));

    for (int i = 0; i < OMV_MIN(n_devs, OMV_ISC_MAX_DEVICES); i++) {
        uint8_t slv_addr = devs_list[i];
        switch (slv_addr) {
            #if (OMV_ENABLE_OV2640 == 1)
            case OV2640_SLV_ADDR: // Or OV9650.
                omv_i2c_readb(&sensor.i2c_bus, slv_addr, OV_CHIP_ID, &sensor.chip_id);
                return slv_addr;
            #endif // (OMV_ENABLE_OV2640 == 1)

            #if (OMV_ENABLE_OV5640 == 1)
            case OV5640_SLV_ADDR:
                omv_i2c_readb2(&sensor.i2c_bus, slv_addr, OV5640_CHIP_ID, &sensor.chip_id);
                return slv_addr;
            #endif // (OMV_ENABLE_OV5640 == 1)

            #if (OMV_ENABLE_OV7725 == 1) || (OMV_ENABLE_OV7670 == 1) || (OMV_ENABLE_OV7690 == 1)
            case OV7725_SLV_ADDR: // Or OV7690 or OV7670.
                omv_i2c_readb(&sensor.i2c_bus, slv_addr, OV_CHIP_ID, &sensor.chip_id);
                return slv_addr;
            #endif //(OMV_ENABLE_OV7725 == 1) || (OMV_ENABLE_OV7670 == 1) || (OMV_ENABLE_OV7690 == 1)

            #if (OMV_ENABLE_MT9V0XX == 1)
            case MT9V0XX_SLV_ADDR:
                omv_i2c_readw(&sensor.i2c_bus, slv_addr, ON_CHIP_ID, &sensor.chip_id_w);
                return slv_addr;
            #endif //(OMV_ENABLE_MT9V0XX == 1)

            #if (OMV_ENABLE_MT9M114 == 1)
            case MT9M114_SLV_ADDR:
                omv_i2c_readw2(&sensor.i2c_bus, slv_addr, ON_CHIP_ID, &sensor.chip_id_w);
                return slv_addr;
            #endif // (OMV_ENABLE_MT9M114 == 1)

            #if (OMV_ENABLE_LEPTON == 1)
            case LEPTON_SLV_ADDR:
                sensor.chip_id = LEPTON_ID;
                return slv_addr;
            #endif // (OMV_ENABLE_LEPTON == 1)

            #if (OMV_ENABLE_HM01B0 == 1) || (OMV_ENABLE_HM0360 == 1)
            case HM0XX0_SLV_ADDR:
                omv_i2c_readb2(&sensor.i2c_bus, slv_addr, HIMAX_CHIP_ID, &sensor.chip_id);
                return slv_addr;
            #endif // (OMV_ENABLE_HM01B0 == 1) || (OMV_ENABLE_HM0360 == 1)

            #if (OMV_ENABLE_GC2145 == 1)
            case GC2145_SLV_ADDR:
                omv_i2c_readb(&sensor.i2c_bus, slv_addr, GC_CHIP_ID, &sensor.chip_id);
                return slv_addr;
            #endif //(OMV_ENABLE_GC2145 == 1)

            #if (OMV_ENABLE_FROGEYE2020 == 1)
            case FROGEYE2020_SLV_ADDR:
                sensor.chip_id_w = FROGEYE2020_ID;
                return slv_addr;
            #endif // (OMV_ENABLE_FROGEYE2020 == 1)
        }
    }

    return 0;
}

int sensor_probe_init(uint32_t bus_id, uint32_t bus_speed) {
    int init_ret = 0;

    #if defined(DCMI_POWER_PIN)
    sensor.pwdn_pol = ACTIVE_HIGH;
    // Do a power cycle
    omv_gpio_write(DCMI_POWER_PIN, 1);
    mp_hal_delay_ms(10);

    omv_gpio_write(DCMI_POWER_PIN, 0);
    mp_hal_delay_ms(10);
    #endif

    #if defined(DCMI_RESET_PIN)
    sensor.reset_pol = ACTIVE_HIGH;
    // Reset the sensor
    omv_gpio_write(DCMI_RESET_PIN, 1);
    mp_hal_delay_ms(10);

    omv_gpio_write(DCMI_RESET_PIN, 0);
    mp_hal_delay_ms(10);
    #endif

    // Initialize the camera bus.
    omv_i2c_init(&sensor.i2c_bus, bus_id, bus_speed);
    mp_hal_delay_ms(10);

    // Scan the bus multiple times using different reset and power-down
    // polarities, until a supported sensor is detected.
    if ((sensor.slv_addr = sensor_detect()) == 0) {
        // No devices were detected, try scanning the bus
        // again with different reset/power-down polarities.
        #if defined(DCMI_RESET_PIN)
        sensor.reset_pol = ACTIVE_LOW;
        omv_gpio_write(DCMI_RESET_PIN, 1);
        mp_hal_delay_ms(10);
        #endif

        if ((sensor.slv_addr = sensor_detect()) == 0) {
            #if defined(DCMI_POWER_PIN)
            sensor.pwdn_pol = ACTIVE_LOW;
            omv_gpio_write(DCMI_POWER_PIN, 1);
            mp_hal_delay_ms(10);
            #endif

            if ((sensor.slv_addr = sensor_detect()) == 0) {
                #if defined(DCMI_RESET_PIN)
                sensor.reset_pol = ACTIVE_HIGH;
                omv_gpio_write(DCMI_RESET_PIN, 0);
                mp_hal_delay_ms(10);
                #endif
                sensor.slv_addr = sensor_detect();
            }
        }

        // If no devices were detected on the I2C bus, try the SPI bus.
        if (sensor.slv_addr == 0) {
            if (0) {
            #if (OMV_ENABLE_PAJ6100 == 1)
            } else if (paj6100_detect(&sensor)) {
                // Found PixArt PAJ6100
                sensor.chip_id_w = PAJ6100_ID;
                sensor.pwdn_pol = ACTIVE_LOW;
                sensor.reset_pol = ACTIVE_LOW;
            #endif
            } else {
                return SENSOR_ERROR_ISC_UNDETECTED;
            }
        }
    }

    // A supported sensor was detected, try to initialize it.
    switch (sensor.chip_id_w) {
        #if (OMV_ENABLE_OV2640 == 1)
        case OV2640_ID:
            if (sensor_set_xclk_frequency(OV2640_XCLK_FREQ) != 0) {
                return SENSOR_ERROR_TIM_INIT_FAILED;
            }
            init_ret = ov2640_init(&sensor);
            break;
        #endif // (OMV_ENABLE_OV2640 == 1)

        #if (OMV_ENABLE_OV5640 == 1)
        case OV5640_ID: {
            int freq = OMV_OV5640_XCLK_FREQ;
            #if (OMV_OV5640_REV_Y_CHECK == 1)
            if (HAL_GetREVID() < 0x2003) {
                // Is this REV Y?
                freq = OMV_OV5640_REV_Y_FREQ;
            }
            #endif
            if (sensor_set_xclk_frequency(freq) != 0) {
                return SENSOR_ERROR_TIM_INIT_FAILED;
            }
            init_ret = ov5640_init(&sensor);
            break;
        }
        #endif // (OMV_ENABLE_OV5640 == 1)

        #if (OMV_ENABLE_OV7670 == 1)
        case OV7670_ID:
            if (sensor_set_xclk_frequency(OV7670_XCLK_FREQ) != 0) {
                return SENSOR_ERROR_TIM_INIT_FAILED;
            }
            init_ret = ov7670_init(&sensor);
            break;
        #endif // (OMV_ENABLE_OV7670 == 1)

        #if (OMV_ENABLE_OV7690 == 1)
        case OV7690_ID:
            if (sensor_set_xclk_frequency(OV7690_XCLK_FREQ) != 0) {
                return SENSOR_ERROR_TIM_INIT_FAILED;
            }
            init_ret = ov7690_init(&sensor);
            break;
        #endif // (OMV_ENABLE_OV7690 == 1)

        #if (OMV_ENABLE_OV7725 == 1)
        case OV7725_ID:
            init_ret = ov7725_init(&sensor);
            break;
        #endif // (OMV_ENABLE_OV7725 == 1)

        #if (OMV_ENABLE_OV9650 == 1)
        case OV9650_ID:
            init_ret = ov9650_init(&sensor);
            break;
        #endif // (OMV_ENABLE_OV9650 == 1)

        #if (OMV_ENABLE_MT9V0XX == 1)
        case MT9V0X2_ID_V_1:
        case MT9V0X2_ID_V_2:
            // Force old versions to the newest.
            sensor.chip_id_w = MT9V0X2_ID;
        case MT9V0X2_ID:
        case MT9V0X4_ID:
            if (sensor_set_xclk_frequency(MT9V0XX_XCLK_FREQ) != 0) {
                return SENSOR_ERROR_TIM_INIT_FAILED;
            }
            init_ret = mt9v0xx_init(&sensor);
            break;
        #endif //(OMV_ENABLE_MT9V0XX == 1)

        #if (OMV_ENABLE_MT9M114 == 1)
        case MT9M114_ID:
            if (sensor_set_xclk_frequency(MT9M114_XCLK_FREQ) != 0) {
                return SENSOR_ERROR_TIM_INIT_FAILED;
            }
            init_ret = mt9m114_init(&sensor);
            break;
        #endif //(OMV_ENABLE_MT9M114 == 1)

        #if (OMV_ENABLE_LEPTON == 1)
        case LEPTON_ID:
            if (sensor_set_xclk_frequency(LEPTON_XCLK_FREQ) != 0) {
                return SENSOR_ERROR_TIM_INIT_FAILED;
            }
            init_ret = lepton_init(&sensor);
            break;
        #endif // (OMV_ENABLE_LEPTON == 1)

        #if (OMV_ENABLE_HM01B0 == 1)
        case HM01B0_ID:
            if (sensor_set_xclk_frequency(HM01B0_XCLK_FREQ) != 0) {
                return SENSOR_ERROR_TIM_INIT_FAILED;
            }
            init_ret = hm01b0_init(&sensor);
            break;
        #endif //(OMV_ENABLE_HM01B0 == 1)

        #if (OMV_ENABLE_HM0360 == 1)
        case HM0360_ID:
            if (sensor_set_xclk_frequency(HM0360_XCLK_FREQ) != 0) {
                return SENSOR_ERROR_TIM_INIT_FAILED;
            }
            init_ret = hm0360_init(&sensor);
            break;
        #endif //(OMV_ENABLE_HM0360 == 1)

        #if (OMV_ENABLE_GC2145 == 1)
        case GC2145_ID:
            if (sensor_set_xclk_frequency(GC2145_XCLK_FREQ) != 0) {
                return SENSOR_ERROR_TIM_INIT_FAILED;
            }
            init_ret = gc2145_init(&sensor);
            break;
        #endif //(OMV_ENABLE_GC2145 == 1)

        #if (OMV_ENABLE_PAJ6100 == 1)
        case PAJ6100_ID:
            if (sensor_set_xclk_frequency(PAJ6100_XCLK_FREQ) != 0) {
                return SENSOR_ERROR_TIM_INIT_FAILED;
            }
            init_ret = paj6100_init(&sensor);
            break;
        #endif // (OMV_ENABLE_PAJ6100 == 1)

        #if (OMV_ENABLE_FROGEYE2020 == 1)
        case FROGEYE2020_ID:
            if (sensor_set_xclk_frequency(FROGEYE2020_XCLK_FREQ) != 0) {
                return SENSOR_ERROR_TIM_INIT_FAILED;
            }
            init_ret = frogeye2020_init(&sensor);
            break;
        #endif // (OMV_ENABLE_FROGEYE2020 == 1)

        default:
            return SENSOR_ERROR_ISC_UNSUPPORTED;
            break;
    }

    if (init_ret != 0) {
        // Sensor init failed.
        return SENSOR_ERROR_ISC_INIT_FAILED;
    }

    return 0;
}

__weak int sensor_get_id() {
    return sensor.chip_id_w;
}

__weak uint32_t sensor_get_xclk_frequency() {
    return SENSOR_ERROR_CTL_UNSUPPORTED;
}

__weak int sensor_set_xclk_frequency(uint32_t frequency) {
    return SENSOR_ERROR_CTL_UNSUPPORTED;
}

__weak bool sensor_is_detected() {
    return sensor.detected;
}

__weak int sensor_sleep(int enable) {
    // Disable any ongoing frame capture.
    sensor_abort();

    // Check if the control is supported.
    if (sensor.sleep == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.sleep(&sensor, enable) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    return 0;
}

__weak int sensor_shutdown(int enable) {
    int ret = 0;

    // Disable any ongoing frame capture.
    sensor_abort();

    #if defined(DCMI_POWER_PIN)
    if (enable) {
        if (sensor.pwdn_pol == ACTIVE_HIGH) {
            omv_gpio_write(DCMI_POWER_PIN, 1);
        } else {
            omv_gpio_write(DCMI_POWER_PIN, 0);
        }
    } else {
        if (sensor.pwdn_pol == ACTIVE_HIGH) {
            omv_gpio_write(DCMI_POWER_PIN, 0);
        } else {
            omv_gpio_write(DCMI_POWER_PIN, 1);
        }
    }
    #endif

    mp_hal_delay_ms(10);

    return ret;
}

__weak int sensor_read_reg(uint16_t reg_addr) {
    int ret;

    // Check if the control is supported.
    if (sensor.read_reg == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if ((ret = sensor.read_reg(&sensor, reg_addr)) == -1) {
        return SENSOR_ERROR_IO_ERROR;
    }

    return ret;
}

__weak int sensor_write_reg(uint16_t reg_addr, uint16_t reg_data) {
    // Check if the control is supported.
    if (sensor.write_reg == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.write_reg(&sensor, reg_addr, reg_data) == -1) {
        return SENSOR_ERROR_IO_ERROR;
    }

    return 0;
}

__weak int sensor_set_pixformat(pixformat_t pixformat) {
    // Check if the value has changed.
    if (sensor.pixformat == pixformat) {
        return 0;
    }

    // Some sensor drivers automatically switch to BAYER to reduce the frame size if it does not fit in RAM.
    // If the current format is BAYER (1BPP), and the target format is color and (2BPP), and the frame does not
    // fit in RAM it will just be switched back again to BAYER, so we keep the current format unchanged.
    uint32_t size = framebuffer_get_buffer_size();
    if ((sensor.pixformat == PIXFORMAT_BAYER)
        && ((pixformat == PIXFORMAT_RGB565) || (pixformat == PIXFORMAT_YUV422))
        && (MAIN_FB()->u * MAIN_FB()->v * 2 > size)
        && (MAIN_FB()->u * MAIN_FB()->v * 1 <= size)) {
        return 0;
    }

    // Cropping and transposing (and thus auto rotation) don't work in JPEG mode.
    if ((pixformat == PIXFORMAT_JPEG)
        && (sensor_get_cropped() || sensor.transpose || sensor.auto_rotation)) {
        return SENSOR_ERROR_PIXFORMAT_UNSUPPORTED;
    }

    // Disable any ongoing frame capture.
    sensor_abort();

    // Flush previous frame.
    framebuffer_update_jpeg_buffer();

    // Check if the control is supported.
    if (sensor.set_pixformat == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.set_pixformat(&sensor, pixformat) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    if (!sensor.disable_delays) {
        mp_hal_delay_ms(100); // wait for the camera to settle
    }

    // Set pixel format
    sensor.pixformat = pixformat;

    // Skip the first frame.
    MAIN_FB()->pixfmt = PIXFORMAT_INVALID;

    // Pickout a good buffer count for the user.
    framebuffer_auto_adjust_buffers();

    // Reconfigure the DCMI if needed.
    return sensor_dcmi_config(pixformat);
}

__weak int sensor_set_framesize(framesize_t framesize) {
    if (sensor.framesize == framesize) {
        // No change
        return 0;
    }

    // Disable any ongoing frame capture.
    sensor_abort();

    // Flush previous frame.
    framebuffer_update_jpeg_buffer();

    // Call the sensor specific function
    if (sensor.set_framesize == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    if (sensor.set_framesize(&sensor, framesize) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    if (!sensor.disable_delays) {
        mp_hal_delay_ms(100); // wait for the camera to settle
    }

    // Set framebuffer size
    sensor.framesize = framesize;

    // Skip the first frame.
    MAIN_FB()->pixfmt = PIXFORMAT_INVALID;

    // Set MAIN FB x offset, y offset, width, height, backup width, and backup height.
    MAIN_FB()->x = 0;
    MAIN_FB()->y = 0;
    MAIN_FB()->w = MAIN_FB()->u = resolution[framesize][0];
    MAIN_FB()->h = MAIN_FB()->v = resolution[framesize][1];

    // Pickout a good buffer count for the user.
    framebuffer_auto_adjust_buffers();

    return 0;
}

__weak int sensor_set_framerate(int framerate) {
    if (sensor.framerate == framerate) {
        // No change
        return 0;
    }

    if (framerate < 0) {
        return SENSOR_ERROR_INVALID_ARGUMENT;
    }

    // If the sensor implements framerate control use it.
    if (sensor.set_framerate != NULL
        && sensor.set_framerate(&sensor, framerate) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    } else {
        // Otherwise use software framerate control.
        sensor.framerate = framerate;
    }
    return 0;
}

__weak bool sensor_get_cropped() {
    if (sensor.framesize != FRAMESIZE_INVALID) {
        return (MAIN_FB()->x != 0)                                  // should be zero if not cropped.
               || (MAIN_FB()->y != 0)                               // should be zero if not cropped.
               || (MAIN_FB()->u != resolution[sensor.framesize][0]) // should be equal to the resolution if not cropped.
               || (MAIN_FB()->v != resolution[sensor.framesize][1]); // should be equal to the resolution if not cropped.
    }
    return false;
}


__weak uint32_t sensor_get_src_bpp() {
    switch (sensor.pixformat) {
        case PIXFORMAT_GRAYSCALE:
            return sensor.hw_flags.gs_bpp;
        case PIXFORMAT_RGB565:
        case PIXFORMAT_YUV422:
            return 2;
        case PIXFORMAT_BAYER:
        case PIXFORMAT_JPEG:
            return 1;
        default:
            return 0;
    }
}

__weak uint32_t sensor_get_dst_bpp() {
    switch (sensor.pixformat) {
        case PIXFORMAT_GRAYSCALE:
        case PIXFORMAT_BAYER:
            return 1;
        case PIXFORMAT_RGB565:
        case PIXFORMAT_YUV422:
            return 2;
        default:
            return 0;
    }
}

__weak int sensor_set_windowing(int x, int y, int w, int h) {
    // Check if the value has changed.
    if ((MAIN_FB()->x == x) && (MAIN_FB()->y == y) &&
        (MAIN_FB()->u == w) && (MAIN_FB()->v == h)) {
        return 0;
    }

    if (sensor.pixformat == PIXFORMAT_JPEG) {
        return SENSOR_ERROR_PIXFORMAT_UNSUPPORTED;
    }

    // Disable any ongoing frame capture.
    sensor_abort();

    // Flush previous frame.
    framebuffer_update_jpeg_buffer();

    // Skip the first frame.
    MAIN_FB()->pixfmt = PIXFORMAT_INVALID;

    MAIN_FB()->x = x;
    MAIN_FB()->y = y;
    MAIN_FB()->w = MAIN_FB()->u = w;
    MAIN_FB()->h = MAIN_FB()->v = h;

    // Pickout a good buffer count for the user.
    framebuffer_auto_adjust_buffers();

    return 0;
}

__weak int sensor_set_contrast(int level) {
    // Check if the control is supported.
    if (sensor.set_contrast == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.set_contrast(&sensor, level) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    return 0;
}

__weak int sensor_set_brightness(int level) {
    // Check if the control is supported.
    if (sensor.set_brightness == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.set_brightness(&sensor, level) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    return 0;
}

__weak int sensor_set_saturation(int level) {
    // Check if the control is supported.
    if (sensor.set_saturation == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.set_saturation(&sensor, level) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    return 0;
}

__weak int sensor_set_gainceiling(gainceiling_t gainceiling) {
    // Check if the value has changed.
    if (sensor.gainceiling == gainceiling) {
        return 0;
    }

    // Check if the control is supported.
    if (sensor.set_gainceiling == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.set_gainceiling(&sensor, gainceiling) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    // Set the new control value.
    sensor.gainceiling = gainceiling;

    return 0;
}

__weak int sensor_set_quality(int qs) {
    // Check if the control is supported.
    if (sensor.set_quality == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.set_quality(&sensor, qs) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    return 0;
}

__weak int sensor_set_colorbar(int enable) {
    // Check if the control is supported.
    if (sensor.set_colorbar == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.set_colorbar(&sensor, enable) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    return 0;
}

__weak int sensor_set_auto_gain(int enable, float gain_db, float gain_db_ceiling) {
    // Check if the control is supported.
    if (sensor.set_auto_gain == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.set_auto_gain(&sensor, enable, gain_db, gain_db_ceiling) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    return 0;
}

__weak int sensor_get_gain_db(float *gain_db) {
    // Check if the control is supported.
    if (sensor.get_gain_db == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.get_gain_db(&sensor, gain_db) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    return 0;
}

__weak int sensor_set_auto_exposure(int enable, int exposure_us) {
    // Check if the control is supported.
    if (sensor.set_auto_exposure == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.set_auto_exposure(&sensor, enable, exposure_us) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    return 0;
}

__weak int sensor_get_exposure_us(int *exposure_us) {
    // Check if the control is supported.
    if (sensor.get_exposure_us == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.get_exposure_us(&sensor, exposure_us) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    return 0;
}

__weak int sensor_set_auto_whitebal(int enable, float r_gain_db, float g_gain_db, float b_gain_db) {
    // Check if the control is supported.
    if (sensor.set_auto_whitebal == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.set_auto_whitebal(&sensor, enable, r_gain_db, g_gain_db, b_gain_db) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    return 0;
}

__weak int sensor_get_rgb_gain_db(float *r_gain_db, float *g_gain_db, float *b_gain_db) {
    // Check if the control is supported.
    if (sensor.get_rgb_gain_db == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.get_rgb_gain_db(&sensor, r_gain_db, g_gain_db, b_gain_db) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    return 0;
}

__weak int sensor_set_auto_blc(int enable, int *regs) {
    // Check if the control is supported.
    if (sensor.set_auto_blc == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.set_auto_blc(&sensor, enable, regs) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    return 0;
}

__weak int sensor_get_blc_regs(int *regs) {
    // Check if the control is supported.
    if (sensor.get_blc_regs == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.get_blc_regs(&sensor, regs) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    return 0;
}

__weak int sensor_set_hmirror(int enable) {
    // Check if the value has changed.
    if (sensor.hmirror == ((bool) enable)) {
        return 0;
    }

    // Disable any ongoing frame capture.
    sensor_abort();

    // Check if the control is supported.
    if (sensor.set_hmirror == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.set_hmirror(&sensor, enable) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    // Set the new control value.
    sensor.hmirror = enable;

    // Wait for the camera to settle
    if (!sensor.disable_delays) {
        mp_hal_delay_ms(100);
    }

    return 0;
}

__weak bool sensor_get_hmirror() {
    return sensor.hmirror;
}

__weak int sensor_set_vflip(int enable) {
    // Check if the value has changed.
    if (sensor.vflip == ((bool) enable)) {
        return 0;
    }

    // Disable any ongoing frame capture.
    sensor_abort();

    // Check if the control is supported.
    if (sensor.set_vflip == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.set_vflip(&sensor, enable) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    // Set the new control value.
    sensor.vflip = enable;

    // Wait for the camera to settle
    if (!sensor.disable_delays) {
        mp_hal_delay_ms(100);
    }

    return 0;
}

__weak bool sensor_get_vflip() {
    return sensor.vflip;
}

__weak int sensor_set_transpose(bool enable) {
    // Check if the value has changed.
    if (sensor.transpose == enable) {
        return 0;
    }

    // Disable any ongoing frame capture.
    sensor_abort();

    if (sensor.pixformat == PIXFORMAT_JPEG) {
        return SENSOR_ERROR_PIXFORMAT_UNSUPPORTED;
    }

    // Set the new control value.
    sensor.transpose = enable;

    return 0;
}

__weak bool sensor_get_transpose() {
    return sensor.transpose;
}

__weak int sensor_set_auto_rotation(bool enable) {
    // Check if the value has changed.
    if (sensor.auto_rotation == enable) {
        return 0;
    }

    // Disable any ongoing frame capture.
    sensor_abort();

    // Operation not supported on JPEG images.
    if (sensor.pixformat == PIXFORMAT_JPEG) {
        return SENSOR_ERROR_PIXFORMAT_UNSUPPORTED;
    }

    // Set the new control value.
    sensor.auto_rotation = enable;
    return 0;
}

__weak bool sensor_get_auto_rotation() {
    return sensor.auto_rotation;
}

__weak int sensor_set_framebuffers(int count) {
    // Disable any ongoing frame capture.
    sensor_abort();

    // Flush previous frame.
    framebuffer_update_jpeg_buffer();

    return framebuffer_set_buffers(count);
}

__weak int sensor_set_special_effect(sde_t sde) {
    // Check if the value has changed.
    if (sensor.sde == sde) {
        return 0;
    }

    // Check if the control is supported.
    if (sensor.set_special_effect == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.set_special_effect(&sensor, sde) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    // Set the new control value.
    sensor.sde = sde;

    return 0;
}

__weak int sensor_set_lens_correction(int enable, int radi, int coef) {
    // Check if the control is supported.
    if (sensor.set_lens_correction == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    // Call the sensor specific function.
    if (sensor.set_lens_correction(&sensor, enable, radi, coef) != 0) {
        return SENSOR_ERROR_CTL_FAILED;
    }

    return 0;
}

__weak int sensor_ioctl(int request, ... /* arg */) {
    // Disable any ongoing frame capture.
    sensor_abort();

    // Check if the control is supported.
    if (sensor.ioctl == NULL) {
        return SENSOR_ERROR_CTL_UNSUPPORTED;
    }

    va_list ap;
    va_start(ap, request);
    // Call the sensor specific function.
    int ret = sensor.ioctl(&sensor, request, ap);
    va_end(ap);

    return ((ret != 0) ? SENSOR_ERROR_CTL_FAILED : 0);
}

__weak int sensor_set_vsync_callback(vsync_cb_t vsync_cb) {
    sensor.vsync_callback = vsync_cb;
    return 0;
}

__weak int sensor_set_frame_callback(frame_cb_t vsync_cb) {
    sensor.frame_callback = vsync_cb;
    return 0;
}

__weak int sensor_set_color_palette(const uint16_t *color_palette) {
    sensor.color_palette = color_palette;
    return 0;
}

__weak const uint16_t *sensor_get_color_palette() {
    return sensor.color_palette;
}

__weak int sensor_check_framebuffer_size() {
    uint32_t bpp = sensor_get_dst_bpp();
    uint32_t size = framebuffer_get_buffer_size();
    return (((MAIN_FB()->u * MAIN_FB()->v * bpp) <= size) ? 0 : -1);
}

__weak int sensor_auto_crop_framebuffer() {
    uint32_t bpp = sensor_get_dst_bpp();
    uint32_t size = framebuffer_get_buffer_size();

    // If the pixformat is NULL/JPEG there we can't do anything to check if it fits before hand.
    if (!bpp) {
        return 0;
    }

    // MAIN_FB() fits, we are done.
    if ((MAIN_FB()->u * MAIN_FB()->v * bpp) <= size) {
        return 0;
    }

    if ((sensor.pixformat == PIXFORMAT_RGB565) || (sensor.pixformat == PIXFORMAT_YUV422)) {
        // Switch to bayer for the quick 2x savings.
        sensor_set_pixformat(PIXFORMAT_BAYER);
        bpp = 1;

        // MAIN_FB() fits, we are done (bpp is 1).
        if ((MAIN_FB()->u * MAIN_FB()->v) <= size) {
            return 0;
        }
    }

    int window_w = MAIN_FB()->u;
    int window_h = MAIN_FB()->v;

    // We need to shrink the frame buffer. We can do this by cropping. So, we will subtract columns
    // and rows from the frame buffer until it fits within the frame buffer.
    int max = IM_MAX(window_w, window_h);
    int min = IM_MIN(window_w, window_h);
    float aspect_ratio = max / ((float) min);
    float r = aspect_ratio, best_r = r;
    int c = 1, best_c = c;
    float best_err = FLT_MAX;

    // Find the width/height ratio that's within 1% of the aspect ratio with a loop limit.
    for (int i = 100; i; i--) {
        float err = fast_fabsf(r - fast_roundf(r));

        if (err <= best_err) {
            best_err = err;
            best_r = r;
            best_c = c;
        }

        if (best_err <= 0.01f) {
            break;
        }

        r += aspect_ratio;
        c += 1;
    }

    // Select the larger geometry to map the aspect ratio to.
    int u_sub, v_sub;

    if (window_w > window_h) {
        u_sub = fast_roundf(best_r);
        v_sub = best_c;
    } else {
        u_sub = best_c;
        v_sub = fast_roundf(best_r);
    }

    // Crop the frame buffer while keeping the aspect ratio and keeping the width/height even.
    while (((MAIN_FB()->u * MAIN_FB()->v * bpp) > size) || (MAIN_FB()->u % 2) || (MAIN_FB()->v % 2)) {
        MAIN_FB()->u -= u_sub;
        MAIN_FB()->v -= v_sub;
    }

    // Center the new window using the previous offset and keep the offset even.
    MAIN_FB()->x += (window_w - MAIN_FB()->u) / 2;
    MAIN_FB()->y += (window_h - MAIN_FB()->v) / 2;

    if (MAIN_FB()->x % 2) {
        MAIN_FB()->x -= 1;
    }
    if (MAIN_FB()->y % 2) {
        MAIN_FB()->y -= 1;
    }

    // Pickout a good buffer count for the user.
    framebuffer_auto_adjust_buffers();
    return 0;
}

const char *sensor_strerror(int error) {
    static const char *sensor_errors[] = {
        "No error.",
        "Sensor control failed.",
        "The requested operation is not supported by the image sensor.",
        "Failed to detect the image sensor or image sensor is detached.",
        "The detected image sensor is not supported.",
        "Failed to initialize the image sensor.",
        "Failed to initialize the image sensor clock.",
        "Failed to initialize the image sensor DMA.",
        "Failed to initialize the image sensor DCMI.",
        "An low level I/O error has occurred.",
        "Frame capture has failed.",
        "Frame capture has timed out.",
        "Frame size is not supported or is not set.",
        "Pixel format is not supported or is not set.",
        "Window is not supported or is not set.",
        "Frame rate is not supported or is not set.",
        "An invalid argument is used.",
        "The requested operation is not supported on the current pixel format.",
        "Frame buffer error.",
        "Frame buffer overflow, try reducing the frame size.",
        "JPEG frame buffer overflow.",
    };

    // Sensor errors are negative.
    error = ((error < 0) ? (error * -1) : error);

    if (error > (sizeof(sensor_errors) / sizeof(sensor_errors[0]))) {
        return "Unknown error.";
    } else {
        return sensor_errors[error];
    }
}

// Initializes DMA logic and returns the size of the dma transfer in bytes or an error code.
__weak int sensor_snapshot_dma_setup(sensor_t *sensor, int w, int h) {
    return 0;
}

// Returns the number of longs remaining in the DMA transfer.
__weak int sensor_snapshot_dma_counter() {
    return 0;
}

__weak int sensor_snapshot(sensor_t *sensor, image_t *image, uint32_t flags) {
    // Compress the framebuffer for the IDE preview, only if it's not the first frame,
    // the framebuffer is enabled and the image sensor does not support JPEG encoding.
    // Note: This doesn't run unless the IDE is connected and the framebuffer is enabled.
    framebuffer_update_jpeg_buffer();

    // Make sure the raw frame fits into the FB. It will be switched from RGB565 to BAYER
    // first to save space before being cropped until it fits.
    sensor_auto_crop_framebuffer();

    // The user may have changed the MAIN_FB width or height on the last image so we need
    // to restore that here. We don't have to restore bpp because that's taken care of
    // already in the code below. Note that we do the JPEG compression above first to save
    // the FB of whatever the user set it to and now we restore.
    int w = MAIN_FB()->u;
    int h = MAIN_FB()->v;

    // If sensor_snapshot_line_callback() happens before framebuffer_free_current_buffer(); below then the
    // transfer is stopped and it will be re-enabled again right afterwards in the single vbuffer
    // case. sensor_snapshot_dma_setup() will know the transfer was stopped.
    framebuffer_free_current_buffer();

    int length = sensor_snapshot_dma_setup(sensor, w, h);
    if (length < 0) {
        return length;
    }

    // Let the camera know we want to trigger it now.
    #if defined(DCMI_FSYNC_PIN)
    if (sensor->hw_flags.fsync) {
        omv_gpio_write(DCMI_FSYNC_PIN, 1);
    }
    #endif

    vbuffer_t *buffer = framebuffer_get_head(FB_NO_FLAGS);
    // Wait for the DMA to finish the transfer.
    for (mp_uint_t tick_start = mp_hal_ticks_ms(); buffer == NULL; buffer = framebuffer_get_head(FB_NO_FLAGS)) {
        MICROPY_EVENT_POLL_HOOK

        // If we haven't exited this loop before the timeout then we need to abort the transfer.
        if ((mp_hal_ticks_ms() - tick_start) > SENSOR_TIMEOUT_MS) {
            sensor_abort();

            #if defined(DCMI_FSYNC_PIN)
            if (sensor->hw_flags.fsync) {
                omv_gpio_write(DCMI_FSYNC_PIN, 0);
            }
            #endif

            return SENSOR_ERROR_CAPTURE_TIMEOUT;
        }
    }

    // We have to abort the JPEG data transfer since it will be stuck waiting for data.
    // line will contain how many transfers we completed.
    // The DMA counter must be used to get the number of remaining words to be transferred.
    if ((sensor->pixformat == PIXFORMAT_JPEG) && (sensor->chip_id == OV2640_ID)) {
        sensor_abort();
    }

    // We're done receiving data.
    #if defined(DCMI_FSYNC_PIN)
    if (sensor->hw_flags.fsync) {
        omv_gpio_write(DCMI_FSYNC_PIN, 0);
    }
    #endif

    // The JPEG in the frame buffer is actually invalid.
    if (buffer->jpeg_buffer_overflow) {
        return SENSOR_ERROR_JPEG_OVERFLOW;
    }

    // Prepare the frame buffer w/h/bpp values given the image type.

    if (!sensor->transpose) {
        MAIN_FB()->w = w;
        MAIN_FB()->h = h;
    } else {
        MAIN_FB()->w = h;
        MAIN_FB()->h = w;
    }

    // Fix the BPP.
    switch (sensor->pixformat) {
        case PIXFORMAT_GRAYSCALE:
            MAIN_FB()->pixfmt = PIXFORMAT_GRAYSCALE;
            break;
        case PIXFORMAT_RGB565:
            MAIN_FB()->pixfmt = PIXFORMAT_RGB565;
            break;
        case PIXFORMAT_BAYER:
            MAIN_FB()->pixfmt = PIXFORMAT_BAYER;
            MAIN_FB()->subfmt_id = sensor->hw_flags.bayer;
            break;
        case PIXFORMAT_YUV422: {
            bool yuv_order = sensor->hw_flags.yuv_order == SENSOR_HW_FLAGS_YUV422;
            int even = yuv_order ? PIXFORMAT_YUV422 : PIXFORMAT_YVU422;
            int odd = yuv_order ? PIXFORMAT_YVU422 : PIXFORMAT_YUV422;
            MAIN_FB()->pixfmt = (MAIN_FB()->x % 2) ? odd : even;
            break;
        }
        case PIXFORMAT_JPEG: {
            int32_t size = 0;
            if (sensor->chip_id == OV5640_ID) {
                // Offset contains the sum of all the bytes transferred from the offset buffers
                // while in sensor_snapshot_line_callback().
                size = buffer->offset;
            } else {
                // Offset contains the number of length transfers completed. To get the number of bytes transferred
                // within a transfer we have to look at the DMA counter and see how much data was moved.
                size = buffer->offset * length;

                if (sensor_snapshot_dma_counter()) {
                    // Add in the uncompleted transfer length.
                    size += ((length / sizeof(uint32_t)) - sensor_snapshot_dma_counter()) * sizeof(uint32_t);
                }
            }
            // Clean trailing data after 0xFFD9 at the end of the jpeg byte stream.
            MAIN_FB()->pixfmt = PIXFORMAT_JPEG;
            MAIN_FB()->size = jpeg_clean_trailing_bytes(size, buffer->data);
            break;
        }
        default:
            break;
    }

    // Set the user image.
    framebuffer_init_image(image);
    return 0;
}

void sensor_snapshot_end_of_frame() {
    // Reset sensor_snapshot_line_callback() frame drop state.
    sensor.first_line = false;
    if (sensor.drop_frame) {
        sensor.drop_frame = false;
        return;
    }

    framebuffer_get_tail(FB_NO_FLAGS);

    if (sensor.frame_callback) {
        sensor.frame_callback();
    }
}

// Abort everything using whatever is safe to call inside the interrupt handler.
__weak void sensor_abort_it() {
    return;
}

// Returns the hardware crop offset in bytes (hardware crop is only 1 - 3 bytes).
__weak uint32_t sensor_get_hw_crop(uint32_t bytes_per_pixel) {
    return 0;
}

#if defined(OMV_ENABLE_SENSOR_DMA_OFFLOAD)
// Ensure the processor does not receive anymore interrupts until the end of frame.
__weak void sensor_dma_offload_drop_frame() {
    return;
}

// Hand offload to DMA subsystem to copy all lines as they come in from linebuffer (src) to
// image (dst) with bits per pixel (bpp). The processor should not receive any more interrupts
// until the end of frame after this funciton is called.
__weak void sensor_dma_offload(void *dst, void *src, int bpp) {
    return;
}

// Copies a single linebuffer (src) to image (dst) without using the processor.
__weak void sensor_dma_memcpy(vbuffer_t *buffer, void *dst, void *src, int bpp, bool transposed) {
    return;
}
#endif

#define copy_line(dstp, srcp)                              \
    for (int i = MAIN_FB()->u, h = MAIN_FB()->v; i; i--) { \
        *dstp = *srcp++;                                   \
        dstp += h;                                         \
    }

#define copy_line_rev(dstp, srcp)                          \
    for (int i = MAIN_FB()->u, h = MAIN_FB()->v; i; i--) { \
        *dstp = __REV16(*srcp++);                          \
        dstp += h;                                         \
    }

// This function is called back after each line transfer is complete,
// with a pointer to the line buffer that was used. At this point the
// DMA transfers the next line to the other half of the line buffer.
// Returns true on the last line.
void sensor_snapshot_line_callback(uint32_t addr) {
    if (!sensor.first_line) {
        sensor.first_line = true;
        mp_uint_t tick = mp_hal_ticks_ms();
        mp_uint_t framerate_ms = IM_DIV(1000, sensor.framerate);

        // Drops frames to match the frame rate requested by the user. The frame is NOT copied to
        // SRAM/SDRAM when dropping to save CPU cycles/energy that would be wasted.
        // If framerate is zero then this does nothing...
        if (sensor.last_frame_ms_valid && ((tick - sensor.last_frame_ms) < framerate_ms)) {
            sensor.drop_frame = true;
        } else if (sensor.last_frame_ms_valid) {
            sensor.last_frame_ms += framerate_ms;
        } else {
            sensor.last_frame_ms = tick;
            sensor.last_frame_ms_valid = true;
        }
    }

    if (sensor.drop_frame) {
        // If we're dropping a frame in full offload mode it's safe to disable this interrupt saving
        // ourselves from having to service the DMA complete callback.
        #if defined(OMV_ENABLE_SENSOR_DMA_OFFLOAD)
        if (!sensor.transpose) {
            sensor_dma_offload_drop_frame();
        }
        #endif
        return;
    }

    vbuffer_t *buffer = framebuffer_get_tail(FB_PEEK);

    // If snapshot was not already waiting to receive data then we have missed this frame and have
    // to drop it. So, abort this and future transfers. Snapshot will restart the process.
    if (!buffer) {
        sensor_abort_it();
        sensor.first_line = false;
        sensor.drop_frame = false;
        sensor.last_frame_ms = 0;
        sensor.last_frame_ms_valid = false;
        // Reset the queue of frames when we start dropping frames.
        if (!sensor.disable_full_flush) {
            framebuffer_flush_buffers();
        }
        return;
    }

    // We are transferring the image from the DCMI hardware to line buffers so that we have more
    // control to post process the image data before writing it to the frame buffer. This requires
    // more CPU, but, allows us to crop and rotate the image as the data is received.

    // Additionally, the line buffers act as very large fifos which hide SDRAM memory access times
    // on the OpenMV Cam H7 Plus. When SDRAM refreshes the row you are trying to write to the fifo
    // depth on the DCMI hardware and DMA hardware is not enough to prevent data loss.

    if (sensor.pixformat == PIXFORMAT_JPEG) {
        if (sensor.hw_flags.jpeg_mode == 4) {
            // JPEG MODE 4:
            //
            // The width and height are fixed in each frame. The first two bytes are valid data
            // length in every line, followed by valid image data. Dummy data (0xFF) may be used as
            // padding at each line end if the current valid image data is less than the line width.
            //
            // In this mode `offset` holds the size of all jpeg data transferred.
            //
            // Note: We are using this mode for the OV5640 because it allows us to use the line
            // buffers to fifo the JPEG image data input so we can handle SDRAM refresh hiccups
            // that will cause data loss if we make the DMA hardware write directly to the FB.
            //
            uint16_t size = __REV16(*((uint16_t *) addr));
            // Prevent a buffer overflow when writing the jpeg data.
            if (buffer->offset + size > framebuffer_get_buffer_size()) {
                buffer->jpeg_buffer_overflow = true;
                return;
            }
            unaligned_memcpy(buffer->data + buffer->offset, ((uint16_t *) addr) + 1, size);
            buffer->offset += size;
        } else if (sensor.hw_flags.jpeg_mode == 3) {
            // JPEG MODE 3:
            //
            // Compression data is transmitted with programmable width. The last line width maybe
            // different from the other line (there is no dummy data). In each frame, the line
            // number may be different.
            //
            // In this mode `offset` will be incremented by one after 262,140 Bytes have been
            // transferred. If 524,280 Bytes have been transferred line will be incremented again.
            // The DMA counter must be used to get the amount of data transferred between.
            //
            // Note: In this mode the JPEG image data is written directly to the frame buffer. This
            // is not optimal. However, it works okay for the OV2640 since the PCLK is much lower
            // than the OV5640 PCLK. The OV5640 drops data in this mode. Hence using mode 4 above.
            //
            buffer->offset += 1;
        }
        return;
    }

    uint32_t bytes_per_pixel = sensor_get_src_bpp();
    uint8_t *src = ((uint8_t *) addr) + (MAIN_FB()->x * bytes_per_pixel) - sensor_get_hw_crop(bytes_per_pixel);
    uint8_t *dst = buffer->data;

    if (sensor.pixformat == PIXFORMAT_GRAYSCALE) {
        bytes_per_pixel = sizeof(uint8_t);
    }

    // For all non-JPEG and non-transposed modes we can completely offload image capture to MDMA
    // and we do not need to receive any line interrupts for the rest of the frame until it ends.
    #if defined(OMV_ENABLE_SENSOR_DMA_OFFLOAD)
    if (!sensor.transpose) {
        // NOTE: We're starting MDMA here because it gives the maximum amount of time before we
        // have to drop the frame if there's no space. If you use the FRAME/VSYNC callbacks then
        // you will have to drop the frame earlier than necessary if there's no space resulting
        // in the apparent unloaded FPS being lower than this method gives you.
        sensor_dma_offload(dst, src, bytes_per_pixel);
        return;
    }
    #endif

    if (!sensor.transpose) {
        dst += MAIN_FB()->u * bytes_per_pixel * buffer->offset++;
    } else {
        dst += bytes_per_pixel * buffer->offset++;
    }

    // Implement per line, per pixel cropping, and image transposing (for image rotation) in
    // in software using the CPU to transfer the image from the line buffers to the frame buffer.
    uint16_t *src16 = (uint16_t *) src;
    uint16_t *dst16 = (uint16_t *) dst;

    switch (sensor.pixformat) {
        case PIXFORMAT_BAYER:
            #if defined(OMV_ENABLE_SENSOR_DMA_OFFLOAD)
            sensor_dma_memcpy(buffer, dst, src, sizeof(uint8_t), sensor.transpose);
            #else
            if (!sensor.transpose) {
                unaligned_memcpy(dst, src, MAIN_FB()->u);
            } else {
                copy_line(dst, src);
            }
            #endif
            break;
        case PIXFORMAT_GRAYSCALE:
            #if defined(OMV_ENABLE_SENSOR_DMA_OFFLOAD)
            sensor_dma_memcpy(buffer, dst, src, sizeof(uint8_t), sensor.transpose);
            #else
            if (sensor.hw_flags.gs_bpp == 1) {
                // 1BPP GRAYSCALE.
                if (!sensor.transpose) {
                    unaligned_memcpy(dst, src, MAIN_FB()->u);
                } else {
                    copy_line(dst, src);
                }
            } else {
                // Extract Y channel from YUV.
                if (!sensor.transpose) {
                    unaligned_2_to_1_memcpy(dst, src16, MAIN_FB()->u);
                } else {
                    copy_line(dst, src16);
                }
            }
            #endif
            break;
        case PIXFORMAT_RGB565:
        case PIXFORMAT_YUV422:
            #if defined(OMV_ENABLE_SENSOR_DMA_OFFLOAD)
            sensor_dma_memcpy(buffer, dst16, src16, sizeof(uint16_t), sensor.transpose);
            #else
            if ((sensor.pixformat == PIXFORMAT_RGB565 && sensor.hw_flags.rgb_swap)
                || (sensor.pixformat == PIXFORMAT_YUV422 && sensor.hw_flags.yuv_swap)) {
                if (!sensor.transpose) {
                    unaligned_memcpy_rev16(dst16, src16, MAIN_FB()->u);
                } else {
                    copy_line_rev(dst16, src16);
                }
            } else {
                if (!sensor.transpose) {
                    unaligned_memcpy(dst16, src16, MAIN_FB()->u * sizeof(uint16_t));
                } else {
                    copy_line(dst16, src16);
                }
            }
            #endif
            break;
        default:
            break;
    }
}
#endif //MICROPY_PY_SENSOR
