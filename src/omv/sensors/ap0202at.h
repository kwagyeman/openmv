/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2023-2024 oclyke <oclyke@oclyke.dev>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * AP0202AT driver.
 */
#ifndef __AP0202AT_H__
#define __AP0202AT_H__

#include <stdint.h>
#include <stddef.h>
#include "sensor.h"

// AP0202AT EXTCLK frequency.
// Range 24 to 30 MHz does not require PLL reconfiguration.
#define AP0202AT_XCLK_FREQ    (24000000)

// This is the magic firmware ID that the patch loader expects.
// (Determined from AND9930-D / .ini files)
#define PATCHLDR_MAGIC_FIRMWARE_ID (0xa1030204)

// Status codes.
typedef enum {
    STATUS_SUCCESS = 0,
    STATUS_ERROR = 1,
    STATUS_ERROR_EINVAL,
    STATUS_ERROR_TIMEOUT,
    STATUS_ERROR_DOORBELL,

    // Flag indicates presence of errors from other systems.
    STATUS_SOURCE_EXTERNAL = 0x80,
} ap0202at_status_t;

// Patch information data structure.
typedef enum {
    PATCH_FORMAT_W2_ADDR_DATA24 = 0,
    NUM_PATCH_FORMATS,
    PATCH_FORMAT_UNKNOWN,
} ap0202at_patch_data_format_e;

typedef struct _ap0202at_patch_t {
    /** Patch data information. */
    ap0202at_patch_data_format_e format;
    const uint8_t* data;
    size_t len;
    uint16_t ram_address;
    uint16_t ram_size;

    /** AND9930/D Table 184. */
    // Address of the Patch’s loader function in Patch RAM
    uint16_t loader_address;
    // Unique patch identifier
    uint16_t patch_id;
    // Firmware ROM version identifier
    uint32_t firmware_id;
} ap0202at_patch_t;

// Get human-readable status string.
const char* ap0202at_status_to_string(ap0202at_status_t status);

// register access
ap0202at_status_t ap0202at_read_reg_direct(sensor_t *sensor, uint16_t reg_addr, uint16_t* reg_data);
ap0202at_status_t ap0202at_write_reg_direct(sensor_t *sensor, uint16_t reg_addr, uint16_t data);
ap0202at_status_t ap0202at_write_reg_masked(sensor_t *sensor, uint16_t reg_addr, uint16_t data, uint16_t mask);

// writing patches to ram
ap0202at_status_t ap0202at_write_patch_to_ram(sensor_t* sensor, const ap0202at_patch_t* patch_info);

// host command interface (HCI)
ap0202at_status_t ap0202at_host_command_emplace_parameter_offset_u8(uint8_t* pool, size_t pool_len, size_t offset, uint8_t param);
ap0202at_status_t ap0202at_host_command_emplace_parameter_offset_u16(uint8_t* pool, size_t pool_len, size_t offset, uint16_t param);
ap0202at_status_t ap0202at_host_command_emplace_parameter_offset_u32(uint8_t* pool, size_t pool_len, size_t offset, uint32_t param);
ap0202at_status_t ap0202at_host_command_get_doorbell_bit(sensor_t *sensor, uint16_t* result, bool* doorbell);
ap0202at_status_t ap0202at_host_command_poll_doorbell_bit_clear(sensor_t *sensor, uint16_t* result, uint16_t timeout_ms);
ap0202at_status_t ap0202at_host_command_load_parameter_pool(sensor_t *sensor, size_t offset, uint8_t* params, size_t params_len);
ap0202at_status_t ap0202at_host_command_unload_parameter_pool(sensor_t *sensor, size_t offset, uint8_t* params, size_t params_len);
ap0202at_status_t ap0202at_host_command_execute_command_synchronous(sensor_t *sensor, uint16_t command, uint16_t* result, uint16_t timeout_ms);
ap0202at_status_t ap0202at_host_command_start_command_asynchronous(sensor_t *sensor, uint16_t command, uint16_t* result, uint16_t timeout_ms);
ap0202at_status_t ap0202at_host_command_finish_command_asynchronous(sensor_t *sensor, uint16_t command, uint16_t* result, uint16_t timeout_ms);

// patch loader interface (PLI)
ap0202at_status_t ap0202at_patch_manager_load_patch(sensor_t *sensor, const uint16_t patch_index, uint16_t timeout_start_ms, uint16_t timeout_finish_ms);
ap0202at_status_t ap0202at_patch_manager_get_status(sensor_t *sensor, uint16_t* result, uint16_t timeout_ms);
ap0202at_status_t ap0202at_patch_manager_reserve_ram(sensor_t *sensor, const ap0202at_patch_t* patch, uint16_t timeout_ms);
ap0202at_status_t ap0202at_patch_manager_apply_patch(sensor_t *sensor, const ap0202at_patch_t* patch, uint16_t timeout_start_ms, uint16_t timeout_finish_ms);

// camera control interface (CCI)
ap0202at_status_t ap0202at_cci_manager_get_lock(sensor_t *sensor, uint16_t timeout_start_ms, uint16_t timeout_finish_ms);
ap0202at_status_t ap0202at_cci_manager_release_lock(sensor_t *sensor, uint16_t timeout_ms);
ap0202at_status_t ap0202at_cci_manager_config(sensor_t *sensor, uint32_t cci_speed_hz, uint16_t timeout_ms);
ap0202at_status_t ap0202at_cci_manager_set_device(sensor_t *sensor, uint8_t device_address, uint16_t timeout_ms);
ap0202at_status_t ap0202at_cci_manager_read(sensor_t *sensor, uint16_t register_address, uint8_t* data, uint8_t data_len, uint16_t timeout_start_ms, uint16_t timeout_finish_ms);
ap0202at_status_t ap0202at_cci_manager_write(sensor_t *sensor, uint16_t register_address, uint8_t* data, uint8_t data_len, uint16_t timeout_start_ms, uint16_t timeout_finish_ms);

// sensor manager
ap0202at_status_t ap0202at_sensor_manager_discover_sensor(sensor_t *sensor, uint8_t *cci_address, uint8_t *revision, uint16_t *model_id, uint16_t timeout_ms);
ap0202at_status_t ap0202at_sensor_manager_initialize_sensor(sensor_t *sensor, uint16_t timeout_ms);

// sysmgr commands
ap0202at_status_t ap0202at_sysmgr_set_state(sensor_t *sensor, uint8_t state, uint16_t timeout_ms);
ap0202at_status_t ap0202at_sysmgr_get_state(sensor_t *sensor, uint8_t *state, uint16_t timeout_ms);

// not sure what this function really is.
// what is a "sensor sequencer"?
ap0202at_status_t ap0202at_write_sensor_u16(sensor_t *sensor, uint16_t port_address, uint16_t data, uint16_t timeout_start_ms, uint16_t timeout_finish_ms);
ap0202at_status_t ap0202at_write_sensor_sequencer(sensor_t *sensor, uint16_t port_address, const uint16_t* sequencer_data, size_t sequencer_words);

// configuration mode
ap0202at_status_t ap0202at_enter_configuration_mode_hardware(sensor_t *sensor);
ap0202at_status_t ap0202at_enter_configuration_mode_software(sensor_t *sensor);

// reset process
ap0202at_status_t ap0202at_reset(sensor_t *sensor);

// sensor_t struct initialization
ap0202at_status_t ap0202at_init(sensor_t *sensor);

#endif // __AP0202AT_H__
