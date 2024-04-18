/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2023-2024 oclyke <oclyke@oclyke.dev>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * AP0202AT driver library.
 * The AP0202AT ISP is used in combination with other
 * sensors. The methods provided in this file are
 * applicable to the AP0202AT and may be used in
 * combination with drivers for other sensors.
 * 
 * When calling AP0202AT methods it is expected that the
 * sensor struct I2C bus and slave address target the ISP
 * itself. The ISP communicates with the attached sensor
 * on a separate I2C bus.
 */
#include "omv_boardconfig.h"
#if (OMV_AP0202AT_ENABLE == 1)

#include "sensor.h"

#include "py/mphal.h"

#include "omv_log.h"

#include "ap0202at.h"
#include "ap0202at_regs.h"

#include <stdint.h>

// Maximum time in milliseconds before host command
//   polling times out.
#define AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS (100)
#define AP0202AT_HOST_COMMAND_READ_POLL_TIMEOUT_MS (100)

// Maximum delay in milliseconds before SYSMGR_GET_STATE
//   polling times out during a reset.
#define AP0202AT_RESET_GET_STATE_TIMEOUT_MS (200)

/**
 * @brief Translate an AP0202AT status code to a string.
 * 
 * @param status the status code to be translated.
 * @return const char* the string representation of the status.
 */
const char* ap0202at_status_to_string(ap0202at_status_t status) {
    switch (status) {
        case STATUS_SUCCESS:
            return "SUCCESS";
        case STATUS_ERROR:
            return "ERROR";
        case STATUS_ERROR_EINVAL:
            return "ERROR_EINVAL";
        case STATUS_ERROR_TIMEOUT:
            return "ERROR_TIMEOUT";
        case STATUS_ERROR_DOORBELL:
            return "ERROR_DOORBELL";
        default:
            if (status & STATUS_SOURCE_EXTERNAL) {
                return "EXTERNAL_ERROR";
            } else {
                return "UNKNOWN_ERROR";
            }
    }
}

/**
 * @brief Read a register from the AP0202AT ISP.
 * @details Uses the two-wire interface to read a 16-bit
 *   wide register from the AP0202AT ISP.
 * 
 * This function does not perform a NULL check on the
 *   output parameter.
 * 
 * @param sensor the sensor struct.
 * @param reg_addr the register address.
 * @param reg_data [output] pointer to the register data.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_read_reg_direct(sensor_t *sensor, uint16_t reg_addr, uint16_t* reg_data) {
    ap0202at_status_t ret = STATUS_SUCCESS;

    // Perform read.
    // Translate OpenMV return code.
    int omv_return_code = omv_i2c_readw2(&sensor->i2c_bus, sensor->slv_addr, reg_addr, reg_data);
    if (omv_return_code != 0) {
        LOG_ERROR("Failed read with code %d\n", omv_return_code);
        return STATUS_SOURCE_EXTERNAL | omv_return_code;
    }

    return ret;
}

/**
 * @brief Write data to a register in the AP0202AT ISP.
 * @details Uses the two-wire interface to write a 16-bit
 *   wide register in the AP0202AT ISP.
 * 
 * @param sensor the sensor struct.
 * @param reg_addr the register address at which the data
 *   will be written.
 * @param data the data to be written to the register.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_write_reg_direct(sensor_t *sensor, uint16_t reg_addr, uint16_t data) {
    ap0202at_status_t ret = STATUS_SUCCESS;

    // Perform write.
    // Translate OpenMV return code.
    int omv_return_code = omv_i2c_writew2(&sensor->i2c_bus, sensor->slv_addr, reg_addr, data);
    if(omv_return_code != 0) {
        LOG_ERROR("Failed write with code %d\n", omv_return_code);
        return STATUS_SOURCE_EXTERNAL | omv_return_code;
    }

    return ret;
}

/**
 * @brief Write data to a register field in the AP0202AT
 *   ISP.
 * @details Uses the two-wire interface to perform a
 *   read-modify-write operation on a 16-bit wide register
 *   according to a given mask.
 * 
 * The mask should be SET in the bits which are to be
 *   modified. Cleared bits will be left unchanged in the
 *   register.
 * 
 * Both the register value and the mask must be correctly
 *   aligned within the register. This function does not
 *   perform any shifting.
 * 
 * @param sensor the sensor struct.
 * @param reg_addr the register address at which the data
 *   will be written.
 * @param data the data to be written to the register.
 * @param mask the mask indicating portion of data to be
 *   written in the register.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_write_reg_masked(sensor_t *sensor, uint16_t reg_addr, uint16_t data, uint16_t mask) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t reg_data;

    // Read the existing data from the register.
    ret = ap0202at_read_reg_direct(sensor, reg_addr, &reg_data);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("Failed to read register. Status %d, (%s)\n", ret, ap0202at_status_to_string(ret));
        return ret;
    }
    
    // Update the register data.
    reg_data &= ~mask;
    reg_data |= (data & mask);

    // Write the updated data to the register.
    ret = ap0202at_write_reg_direct(sensor, reg_addr, reg_data);
    if(ret != STATUS_SUCCESS) {
        LOG_ERROR("Failed to write register. Status %d, (%s)\n", ret, ap0202at_status_to_string(ret));
        return ret;
    }

    return ret;
}

/**
 * @brief Writes a patch to the AP0202AT ISP.
 * 
 * @param sensor the sensor struct.
 * @param patch_info pointer to the patch information.
 * @return ap0202at_status_t 
 */
ap0202at_status_t ap0202at_write_patch_to_ram(sensor_t* sensor, const ap0202at_patch_t* patch) {
    ap0202at_status_t ret = STATUS_SUCCESS;

    // Check the data format.
    // This function only supports the W2_ADDR_DATA24 format.
    if (patch->format != PATCH_FORMAT_W2_ADDR_DATA24) {
        LOG_ERROR("Unsupported data format: %d\n", patch->format);
        return STATUS_ERROR_EINVAL;
    }

    uint16_t physical_address = patch->ram_address + AP0202AT_ADDR_PATCH_RAM_START;

    // Set the access control state.
    // PHYSICAL_UPPER_ADDRESS: 0x00 (Bits 17:16 of the physical address, assumed to be 0)
    // PHY_REGIO: 0x00 (Physical access to patch RAM)
    // EN_UPPER_32K_PHY_ACCESS: ((address >> 15) & 0x01) (This is bit 15 of the physical address)
    ret = ap0202at_write_reg_direct(sensor, AP0202AT_REG_ACCESS_CTL_STAT, (physical_address >> 15) & 0x01);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("Failed to set access control state register\n");
        return ret;
    }
    // Set the lower 15 bits of the physical address.
    // Note: Writing to this register will automatically set the PHYSICAL_ACCESS_STATE to 0x03 (physical access).
    ret = ap0202at_write_reg_direct(sensor, AP0202AT_REG_PHYSICAL_ADDRESS_ACCESS, physical_address & 0x7FFF);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("Failed to set physical address access\n");
        return ret;
    }
    // Write the entire contents of the patch to the patch RAM.
    int omv_return_code = omv_i2c_write_bytes(&sensor->i2c_bus, sensor->slv_addr, (uint8_t*)patch->data, patch->len, OMV_I2C_XFER_NO_FLAGS);
    if (omv_return_code != 0) {
        LOG_ERROR("Failed to write patch data with code %d\n", omv_return_code);
        return STATUS_SOURCE_EXTERNAL | omv_return_code;
    }
    // Reset the access control state.
    ret = ap0202at_write_reg_direct(sensor, AP0202AT_REG_LOGICAL_ADDRESS_ACCESS, 0x0000);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("Failed to set logical address access\n");
        return ret;
    }

    return ret;
}

ap0202at_status_t ap0202at_write_sensor_u16(sensor_t *sensor, uint16_t port_address, uint16_t data, uint16_t timeout_start_ms, uint16_t timeout_finish_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;

    uint8_t buf[2];
    buf[0] = data >> 8;
    buf[1] = data & 0xFF;

    ret = ap0202at_cci_manager_write(sensor, port_address, buf, 2, timeout_start_ms, timeout_start_ms);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }

    return ret;
}

/**
 * @brief 
 * 
 * APA0202AT-REV2_AR0147-REV3.ini line 1430
 * 
 * @param sensor the sensor struct.
 * @param port_address
 * @param sequencer_data 
 * @param sequencer_words the length of the sequencer data
 *  in words. 
 * @return ap0202at_status_t
 */
ap0202at_status_t ap0202at_write_sensor_sequencer(sensor_t* sensor, uint16_t port_address, const uint16_t* sequencer_data, size_t sequencer_words) {
    // this function is not implemented.
    // there are some serious questions left to answer such as:
    // - what address are we writing to within the sensor?
    // - is that data encoded in the sequencer format?
    // - how can we make this work with the ap0202at_cci_manager_write function?
    return -1;

    // ap0202at_status_t ret = STATUS_SUCCESS;
    // uint16_t host_command_result;

    // size_t burst_words = 6;
    // size_t num_bursts = sequencer_words / burst_words;
    // size_t leftover_words = sequencer_words % burst_words;

    // // The full params pool contains the port address
    // //   followed by the legnth (u16) followed by the
    // //   burst data.
    // size_t param_pool_words = 1 + 1 + burst_words;
    // size_t leftover_pool_words = 1 + 1 + leftover_words;
    // uint16_t params_pool[param_pool_words];

    // // Loop over all complete bursts of 6 words.
    // for (size_t idx = 0; idx < num_bursts; idx++) {

    //     // Prepare the params pool.
    //     params_pool[0] = port_address;
    //     params_pool[1] = (burst_words << 9);
    //     for (size_t i = 0; i < burst_words; i++) {
    //         params_pool[2 + i] = sequencer_data[idx * burst_words + i];
    //     }

    //     // Load the parameters pool into the AP0202AT.
    //     if (omv_i2c_write_bytes(&sensor->i2c_bus, sensor->slv_addr, (uint8_t*)&params_pool[0], 2*param_pool_words, OMV_I2C_XFER_NO_FLAGS) != 0) {
    //         return -1;
    //     }

    //     // Write the data through the CCIMGR interface
    //     if  (ap0202at_host_command(sensor, AP0202AT_HC_CMD_CCIMGR_WRITE, &host_command_result, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_READ_POLL_TIMEOUT_MS) != 0) {
    //         return -1;
    //     }
    //     if (host_command_result != AP0202AT_HC_RESP_ENOERR) {
    //         return -1;
    //     }

    //     // Poll the CCIMGR_STATUS command until the write is
    //     //   complete as indicated by a response of ENOERR.
    //     for (;;) {
    //         if  (ap0202at_host_command(sensor, AP0202AT_HC_CMD_CCIMGR_STATUS, &host_command_result, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_READ_POLL_TIMEOUT_MS) != 0) {
    //             return -1;
    //         }
    //         if (host_command_result == AP0202AT_HC_RESP_ENOERR) {
    //             break;
    //         }
    //     }

    // }

    // // Handle the remaining data, if any.
    // if (leftover_words != 0) {

    //     // Prepare the params pool.
    //     params_pool[0] = port_address;
    //     params_pool[1] = (leftover_words << 9);
    //     for (size_t i = 0; i < leftover_words; i++) {
    //         params_pool[2 + i] = sequencer_data[sequencer_words - leftover_words + i];
    //     }

    //     // Load the parameters pool into the AP0202AT.
    //     if (omv_i2c_write_bytes(&sensor->i2c_bus, sensor->slv_addr, (uint8_t*)&params_pool[0], 2*leftover_pool_words, OMV_I2C_XFER_NO_FLAGS) != 0) {
    //         return -1;
    //     }

    //     // Write the data through the CCIMGR interface
    //     if  (ap0202at_host_command(sensor, AP0202AT_HC_CMD_CCIMGR_WRITE, &host_command_result, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_READ_POLL_TIMEOUT_MS) != 0) {
    //         return -1;
    //     }
    //     if (host_command_result != AP0202AT_HC_RESP_ENOERR) {
    //         return -1;
    //     }

    //     // Poll the CCIMGR_STATUS command until the write is
    //     //   complete as indicated by a response of ENOERR.
    //     for (;;) {
    //         if  (ap0202at_host_command(sensor, AP0202AT_HC_CMD_CCIMGR_STATUS, &host_command_result, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_READ_POLL_TIMEOUT_MS) != 0) {
    //             return -1;
    //         }
    //         if (host_command_result == AP0202AT_HC_RESP_ENOERR) {
    //             break;
    //         }
    //     }
    // }

    // return ret;
}

/**
 * @brief Issues 'command' through the host command interface command register.
 * 
 * @param sensor the sensor struct.
 * @param command the command to be issued.
 * @return ap0202at_status_t.
 */
static ap0202at_status_t ap0202at_host_command_issue_command(sensor_t *sensor, uint16_t command) {
    ap0202at_status_t ret = STATUS_SUCCESS;

    // Issue the command.
    command |= AP0202AT_SYSCTL_COMMAND_REGISTER_DOORBELL_BIT_MASK;
    ret = ap0202at_write_reg_direct(sensor, AP0202AT_REG_SYSCTL_COMMAND_REGISTER, command);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }

    return ret;
}

/**
 * @brief Place u8 parameter in the parameter pool at 'offset'.
 * Use 'ap0202at_host_command_load_parameter_pool' to load the
 *  paramter pool into the device.
 * 
 * @param pool pointer to the parameter pool.
 * @param offset the offset (bytes) at which to place the
 *  parameter.
 * @param param the parameter to be placed. 
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_host_command_emplace_parameter_offset_u8(uint8_t* pool, size_t pool_len, size_t offset, uint8_t param) {
    if (NULL == pool) {
        return STATUS_ERROR_EINVAL;
    }
    if (pool_len < 1) {
        return STATUS_ERROR_EINVAL;
    }
    if (offset > (pool_len - 1)) {
        return STATUS_ERROR_EINVAL;
    }
    pool[offset] = param;
    return 0;
}

/**
 * @brief Place u16 parameter in the parameter pool at 'offset'.
 * @details Uses big-endian byte order.
 * 
 * @param pool pointer to the parameter pool.
 * @param offset the offset (bytes) at which to place the
 * parameter.
 * @param param the parameter to be placed.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_host_command_emplace_parameter_offset_u16(uint8_t* pool, size_t pool_len, size_t offset, uint16_t param) {
    if (NULL == pool) {
        return STATUS_ERROR_EINVAL;
    }
    if (pool_len < 2) {
        return STATUS_ERROR_EINVAL;
    }
    if (offset > (pool_len - 2)) {
        return STATUS_ERROR_EINVAL;
    }
    pool[offset] = param >> 8;
    pool[offset + 1] = param & 0xFF;
    return STATUS_SUCCESS;
}

/**
 * @brief Place u32 parameter in the parameter pool at 'offset'.
 * @details Uses big-endian byte order.
 * 
 * @param pool pointer to the parameter pool.
 * @param offset the offset (bytes) at which to place the
 * parameter.
 * @param param the parameter to be placed.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_host_command_emplace_parameter_offset_u32(uint8_t* pool, size_t pool_len, size_t offset, uint32_t param) {
    if (NULL == pool) {
        return STATUS_ERROR_EINVAL;
    }
    if (pool_len < 4) {
        return STATUS_ERROR_EINVAL;
    }
    if (offset > (pool_len - 4)) {
        return STATUS_ERROR_EINVAL;
    }
    pool[offset] = param >> 24;
    pool[offset + 1] = (param >> 16) & 0xFF;
    pool[offset + 2] = (param >> 8) & 0xFF;
    pool[offset + 3] = param & 0xFF;
    return STATUS_SUCCESS;
}

/**
 * @brief Extract a u8 parameter from the parameter pool at 'offset'.
 * 
 * @param pool pointer to the parameter pool.
 * @param pool_len the length of the parameter pool.
 * @param offset the offset (bytes) at which to extract the
 * parameter.
 * @param param [output] pointer to the extracted parameter.
 * @return ap0202at_status_t 
 */
ap0202at_status_t ap0202at_host_command_extract_parameter_offset_u8(uint8_t* pool, size_t pool_len, size_t offset, uint8_t* param) {
    if (NULL == pool) {
        return STATUS_ERROR_EINVAL;
    }
    if (pool_len < 1) {
        return STATUS_ERROR_EINVAL;
    }
    if (offset > (pool_len - 1)) {
        return STATUS_ERROR_EINVAL;
    }
    *param = pool[offset];
    return STATUS_SUCCESS;
}

/**
 * @brief Extract a u16 parameter from the parameter pool at 'offset'.
 * 
 * @param pool pointer to the parameter pool.
 * @param pool_len the length of the parameter pool.
 * @param offset the offset (bytes) at which to extract the
 * @param param [output] pointer to the extracted parameter.
 * @return ap0202at_status_t 
 */
ap0202at_status_t ap0202at_host_command_extract_parameter_offset_u16(uint8_t* pool, size_t pool_len, size_t offset, uint16_t* param) {
    if (NULL == pool) {
        return STATUS_ERROR_EINVAL;
    }
    if (pool_len < 2) {
        return STATUS_ERROR_EINVAL;
    }
    if (offset > (pool_len - 2)) {
        return STATUS_ERROR_EINVAL;
    }
    *param = (pool[offset] << 8) | pool[offset + 1];
    return STATUS_SUCCESS;
}

/**
 * @brief Extract a u32 parameter from the parameter pool at 'offset'.
 * 
 * @param pool pointer to the parameter pool.
 * @param pool_len the length of the parameter pool.
 * @param offset the offset (bytes) at which to extract the
 * @param param [output] pointer to the extracted parameter.
 * @return ap0202at_status_t 
 */
ap0202at_status_t ap0202at_host_command_extract_parameter_offset_u32(uint8_t* pool, size_t pool_len, size_t offset, uint32_t* param) {
    if (NULL == pool) {
        return STATUS_ERROR_EINVAL;
    }
    if (pool_len < 4) {
        return STATUS_ERROR_EINVAL;
    }
    if (offset > (pool_len - 4)) {
        return STATUS_ERROR_EINVAL;
    }
    *param = (pool[offset] << 24) | (pool[offset + 1] << 16) | (pool[offset + 2] << 8) | pool[offset + 3];
    return STATUS_SUCCESS;
}

/**
 * @brief Check the Doorbell Bit of the COMMAND_REGISTER.
 * 
 * @param sensor the sensor struct.
 * @param result [output] pointer to the data read from the
 *  COMMAND_REGISTER.
 * @param doorbell [output] pointer to the doorbell bit.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_host_command_get_doorbell_bit(sensor_t *sensor, uint16_t *result, bool *doorbell) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t reg_data;

    // Read the COMMAND_REGISTER.
    ret = ap0202at_read_reg_direct(sensor, AP0202AT_REG_SYSCTL_COMMAND_REGISTER, &reg_data);
    if(ret != STATUS_SUCCESS) {
        return ret;
    }

    // Return the data if the output parameter is not NULL.
    if (result != NULL) {
        *result = reg_data;
    }

    // Return the doorbell bit if the output parameter is not NULL.
    if (doorbell != NULL) {
        *doorbell = ((reg_data & AP0202AT_SYSCTL_COMMAND_REGISTER_DOORBELL_BIT_MASK) != 0);
    }

    return ret;
}

/**
 * @brief Check the Doorbell Bit of the COMMAND_REGISTER.
 * 
 * @param sensor the sensor struct.
 * @param result [output] pointer to the data read from the
 *  COMMAND_REGISTER.
 * @param timeout_ms the maximum time to wait for the
 *  Doorbell Bit to clear in milliseconds.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_host_command_poll_doorbell_bit_clear(sensor_t *sensor, uint16_t *result, uint16_t timeout_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    for (mp_uint_t start = mp_hal_ticks_ms();;) {
        // Check the Doorbell Bit.
        bool doorbell;
        ret = ap0202at_host_command_get_doorbell_bit(sensor, result, &doorbell);
        if (STATUS_SUCCESS != ret) {
            return ret;
        }

        // Break if the Doorbell Bit is clear.
        if (!doorbell) {
            break;
        }

        // Return error if the timeout has elapsed.
        mp_uint_t now = mp_hal_ticks_ms();
        mp_uint_t delta = (now - start);
        if (delta >= timeout_ms) {
            return STATUS_ERROR_TIMEOUT;
        }

        // debug
        static size_t count = 0;
        count++;
        if (count % 500 == 0) {
            LOG_INFO("Doorbell Bit is set. Waiting... %d\n", delta);
        }
    }

    return ret;
}

/**
 * @brief Loads the parameter pool with the given parameters.
 * 
 * @param sensor the sensor struct.
 * @param params pointer to the parameters array.
 * @param params_len length of the parameters array. must be
 * even to ensure that the parameters are written in pairs.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_host_command_load_parameter_pool(sensor_t *sensor, size_t offset, uint8_t* params, size_t params_len) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    if (NULL == params) {
        return STATUS_ERROR_EINVAL;
    }
    if (params_len > 122) {
        return STATUS_ERROR_EINVAL;
    }
    
    // Write the parameters to the parameter pool.
    size_t pairs = params_len / 2;
    for (size_t i = 0; i < pairs; i++) {
        ret = ap0202at_write_reg_direct(sensor, AP0202AT_VAR_CMD_HANDLER_PARAMS_POOL_0 + offset + i*2, (params[2*i] << 8) | params[2*i + 1]);
        if (STATUS_SUCCESS != ret) {
            return ret;
        }
    }

    // handle the last parameter if the length is odd.
    if ((params_len % 2) != 0) {
        ret = ap0202at_write_reg_direct(sensor, AP0202AT_VAR_CMD_HANDLER_PARAMS_POOL_0 + offset + pairs*2, params[params_len - 1]);
        if (STATUS_SUCCESS != ret) {
            return ret;
        }
    }

    return ret;
}

/**
 * @brief Unloads the parameter pool into the given parameters.
 * 
 * @param sensor the sensor struct.
 * @param params [output] pointer to the parameters array.
 * @param params_len length of the parameters array. must be
 * even to ensure that the parameters are read in pairs.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_host_command_unload_parameter_pool(sensor_t *sensor, size_t offset, uint8_t* params, size_t params_len) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    if (NULL == params) {
        return STATUS_ERROR_EINVAL;
    }
    if (params_len > 122) {
        return STATUS_ERROR_EINVAL;
    }

    // Read the parameters from the parameter pool.
    size_t pairs = params_len / 2;
    for (size_t i = 0; i < pairs; i++) {
        uint16_t param;
        ret = ap0202at_read_reg_direct(sensor, AP0202AT_VAR_CMD_HANDLER_PARAMS_POOL_0 + offset + i, &param);
        if (STATUS_SUCCESS != ret) {
            return -1;
        }
        params[2*i] = param >> 8;
        params[2*i + 1] = param & 0xFF;
    }

    // handle the last parameter if the length is odd.
    if ((params_len % 2) != 0) {
        uint16_t param;
        ret = ap0202at_read_reg_direct(sensor, AP0202AT_VAR_CMD_HANDLER_PARAMS_POOL_0 + offset + pairs, &param);
        if (STATUS_SUCCESS != ret) {
            return -1;
        }
        params[params_len - 1] = param & 0xFF;
    }

    return ret;
}

/**
 * @brief Synchronous Host Command Flow.
 * @details Executes a synchronous host command and returns the result.
 *  * Doorbell bit must be clear before issuing the command, or the
 *    command will not be issued and an error code will be returned.
 * 
 * This function does not validate that the command is a synchronous
 * command!
 * 
 * AND9930/D HOST COMMAND PROCESSING :: Synchronous Command Flow
 * 
 * @param sensor the sensor struct.
 * @param command the command to be executed.
 * @param result [output] pointer to the result of the command.
 * @return int the status of the command.
 */
ap0202at_status_t ap0202at_host_command_execute_command_synchronous(sensor_t *sensor, uint16_t command, uint16_t* result, uint16_t timeout_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    bool doorbell;

    // Ensure that the Doorbell Bit is clear.
    ret = ap0202at_host_command_get_doorbell_bit(sensor, NULL, &doorbell);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (doorbell) {
        return STATUS_ERROR_DOORBELL;
    }

    // Issue the command.
    ret = ap0202at_host_command_issue_command(sensor, command);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }

    // Wait for the Doorbell Bit to be cleared.
    ret = ap0202at_host_command_poll_doorbell_bit_clear(sensor, result, timeout_ms);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }

    return ret;
}

/**
 * @brief Start an asynchronous host command.
 * 
 * This function is mostly used symbolically to indicate that
 * the command is asynchronous.
 * 
 * Currently no validation is done to ensure that the command
 * is actually an aync command.
 * 
 * @param sensor the sensor struct.
 * @param command the command to be executed.
 * @param result [output] pointer to the result of the command.
 * @param timeout_ms the maximum time to wait for the command to
 * complete.
 * @return int the status of the command.
 */
ap0202at_status_t ap0202at_host_command_start_command_asynchronous(sensor_t *sensor, uint16_t command, uint16_t* result, uint16_t timeout_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;

    // Launching an async command is the same as executing a synchronous
    // command.
    // The only difference is in the meaning of the response.
    ret = ap0202at_host_command_execute_command_synchronous(sensor, command, result, timeout_ms);

    return ret;
}

/**
 * @brief Issues 'status_command' repeatedly until either the
 *  result is not EBUSY or a timeout occurs.
 * 
 * See AND9930/D Table 1. "Asynchronous Commands and their Get Status Partner"
 * for a list of asynchronous commands and their corresponding status commands.
 * 
 * @param sensor the sensor struct.
 * @param status_command the status command to be issued.
 * @param result [output] pointer to the result of the status command.
 * @param timeout_ms the maximum time to wait for the command to
 * @return int the status of the command.
 */
ap0202at_status_t ap0202at_host_command_finish_command_asynchronous(sensor_t *sensor, uint16_t status_command, uint16_t *result, uint16_t timeout_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;

    // This loop repeatedly checks the result of the status command
    // until the result is not EBUSY.
    // If the result is not EBUSY then the command has completed and
    // the function exits.
    // If the result is EBUSY then the function will continue polling
    // until the timeout has elapsed.
    for (mp_uint_t start = mp_hal_ticks_ms();;) {
        // check the doorbell bit, as some other command 
        // may be running.
        bool doorbell;
        ret = ap0202at_host_command_get_doorbell_bit(sensor, NULL, &doorbell);
        if (STATUS_SUCCESS != ret) {
            return ret;
        }

        // Issue the status command only if the doorbell bit
        // is clear.
        if (!doorbell) {

            // Note: we do not use "ap0202at_host_command_execute_command_synchronous"
            // because formally we want to differentiate between different
            // cases where that function might return an error.
            // 1. error in communication, this function should return -1
            // 2. doorbell bit is not clear, this function should continue
            //    polling for an opportunity to issue the status command.
            ret = ap0202at_host_command_issue_command(sensor, status_command);
            if (STATUS_SUCCESS != ret) {
                return ret;
            }

            // Wait for the response of the status command.
            ret = ap0202at_host_command_poll_doorbell_bit_clear(sensor, result, timeout_ms);
            if (STATUS_SUCCESS != ret) {
                return ret;
            }

            // Return if the result is not EBUSY.
            if (*result != AP0202AT_HC_RESP_EBUSY) {
                return ret;
            }
        }

        // Return error if the timeout has elapsed.
        mp_uint_t now = mp_hal_ticks_ms();
        mp_uint_t delta = (now - start);
        if (delta >= timeout_ms) {
            return STATUS_ERROR_TIMEOUT;
        }
    }

    return STATUS_ERROR;
}

/**
 * @brief Loads and applies the specified patch.
 * 
 * "This asynchronous command requests that the firmware
 * loads and applies a firmware patch stored in NVM."
 * 
 * @param sensor the sensor struct.
 * @param patch_index the index of the patch to be loaded.
 * @param result [output] pointer to the result of the command.
 * @param timeout_start_ms the maximum time to wait for the
 * command to start.
 * @param timeout_finish_ms the maximum time to wait for the
 * command to finish.
 * @return ap0202at_status_t the status of the command.
 */
ap0202at_status_t ap0202at_patch_manager_load_patch(sensor_t *sensor, const uint16_t patch_index, uint16_t timeout_start_ms, uint16_t timeout_finish_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;
    bool doorbell;

    // ensure that the doorbell bit is clear.
    ret = ap0202at_host_command_get_doorbell_bit(sensor, NULL, &doorbell);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_load_patch: get doorbell bit failed\n");
        return ret;
    }
    if (doorbell) {
        LOG_WARNING("ap0202at_patch_manager_load_patch: doorbell bit is set\n");
        return STATUS_ERROR_DOORBELL;
    }

    // prepare the parameter pool.
    uint8_t pool[2];
    ret = ap0202at_host_command_emplace_parameter_offset_u16(pool, sizeof(pool)/sizeof(pool[0]), 0, patch_index);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_load_patch: emplace parameter failed\n");
        return ret;
    }

    // load the parameter pool.
    ret = ap0202at_host_command_load_parameter_pool(sensor, 0, pool, sizeof(pool)/sizeof(pool[0]));
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_load_patch: load parameter pool failed\n");
        return ret;
    }

    // start the command asynchronously.
    ret = ap0202at_host_command_start_command_asynchronous(sensor, AP0202AT_HC_CMD_PATCHLDR_LOAD_PATCH, &host_command_result, timeout_start_ms);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_load_patch: start command failed\n");
        return ret;
    }
    if ((AP0202AT_HC_RESP_ENOERR != host_command_result)
        && (AP0202AT_HC_RESP_EALREADY != host_command_result)) {
        LOG_WARNING("ap0202at_patch_manager_load_patch: start command returned: %d\n", host_command_result);
        return ret;
    }

    // finish the command.
    ret = ap0202at_host_command_finish_command_asynchronous(sensor, AP0202AT_HC_CMD_PATCHLDR_STATUS, &host_command_result, timeout_finish_ms);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_load_patch: finish command failed\n");
        return ret;
    }
    if (AP0202AT_HC_RESP_ENOERR != host_command_result) {
        LOG_WARNING("ap0202at_patch_manager_load_patch: finish command returned: %d\n", host_command_result);
        return ret;
    }

    return ret;
}

/**
 * @brief Get the status of in-progress patch loader operations.
 * 
 * @param sensor the sensor struct.
 * @param result [output] pointer to the result of the command.
 * @return ap0202at_status_t the status of the command.
 */
ap0202at_status_t ap0202at_patch_manager_get_status(sensor_t *sensor, uint16_t* result, uint16_t timeout_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;

    // execute the command synchronously.
    ret = ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_PATCHLDR_STATUS, &host_command_result, timeout_ms);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_get_status: execute command failed\n");
        return ret;
    }
    if (AP0202AT_HC_RESP_ENOERR != host_command_result) {
        LOG_WARNING("ap0202at_patch_manager_get_status: command returned: %d\n", host_command_result);
        return STATUS_SOURCE_EXTERNAL | host_command_result;
    }

    // return the result.
    if (NULL != result) {
        *result = host_command_result;
    }

    return ret;
}

/**
 * @brief Requests the patch loader to reserve a region of
 * Patch RAM to contain a patch.
 * 
 * @param sensor 
 * @param start_address 
 * @param size_bytes 
 * @param timeout_ms 
 * @return ap0202at_status_t 
 */
ap0202at_status_t ap0202at_patch_manager_reserve_ram(sensor_t *sensor, const ap0202at_patch_t* patch, uint16_t timeout_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;
    bool doorbell;

    uint16_t start_address = patch->ram_address;
    uint16_t size_bytes = patch->ram_size;

    // ensure that the doorbell bit is clear.
    ret = ap0202at_host_command_get_doorbell_bit(sensor, NULL, &doorbell);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_reserve_ram: get doorbell bit failed\n");
        return ret;
    }
    if (doorbell) {
        LOG_WARNING("ap0202at_patch_manager_reserve_ram: doorbell bit is set\n");
        return STATUS_ERROR_DOORBELL;
    }

    // prepare the parameter pool.
    uint8_t pool[4];
    ret = ap0202at_host_command_emplace_parameter_offset_u16(pool, sizeof(pool)/sizeof(pool[0]), 0, start_address);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_reserve_ram: emplace start address parameter failed\n");
        return ret;
    }
    ret = ap0202at_host_command_emplace_parameter_offset_u16(pool, sizeof(pool)/sizeof(pool[0]), 2, size_bytes);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_reserve_ram: emplace size parameter failed\n");
        return ret;
    }

    // load the parameter pool.
    ret = ap0202at_host_command_load_parameter_pool(sensor, 0, pool, sizeof(pool)/sizeof(pool[0]));
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_reserve_ram: load parameter pool failed\n");
        return ret;
    }

    // execute the command synchronously.
    ret = ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_PATCHLDR_RESERVE_RAM, &host_command_result, timeout_ms);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_reserve_ram: execute command failed\n");
        return ret;
    }
    if (AP0202AT_HC_RESP_ENOERR != host_command_result) {
        LOG_WARNING("ap0202at_patch_manager_reserve_ram: command returned: %d\n", host_command_result);
        return STATUS_SOURCE_EXTERNAL | host_command_result;
    }

    return ret;
}

/**
 * @brief Requests that the Patch Loader applies a patch stored in RAM.
 * 
 * @param sensor pointer to the sensor struct.
 * @param loader_address the address of the
 * loader in RAM.
 * @param patch_id the patch ID.
 * @param firmware_id the firmware ID.
 * @param ram_addr the address in RAM where the patch is stored.
 * @return ap0202at_status_t 
 */
ap0202at_status_t ap0202at_patch_manager_apply_patch(sensor_t *sensor, const ap0202at_patch_t* patch, uint16_t timeout_start_ms, uint16_t timeout_finish_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;
    bool doorbell;

    uint16_t loader_address = patch->ram_address;
    uint16_t patch_id = patch->patch_id;
    uint32_t firmware_id = patch->firmware_id;
    uint16_t patch_size = patch->len;

    // ensure that the doorbell bit is clear.
    ret = ap0202at_host_command_get_doorbell_bit(sensor, NULL, &doorbell);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_apply_patch: get doorbell bit failed\n");
        return ret;
    }
    if (doorbell) {
        LOG_WARNING("ap0202at_patch_manager_apply_patch: doorbell bit is set\n");
        return STATUS_ERROR_DOORBELL;
    }

    // prepare the parameter pool.
    uint8_t pool[10];
    ret = ap0202at_host_command_emplace_parameter_offset_u16(pool, sizeof(pool)/sizeof(pool[0]), 0, loader_address);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_apply_patch: emplace loader address parameter failed\n");
        return ret;
    }
    ret = ap0202at_host_command_emplace_parameter_offset_u16(pool, sizeof(pool)/sizeof(pool[0]), 2, patch_id);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_apply_patch: emplace patch id parameter failed\n");
        return ret;
    }
    ret = ap0202at_host_command_emplace_parameter_offset_u32(pool, sizeof(pool)/sizeof(pool[0]), 4, firmware_id);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_apply_patch: emplace firmware id parameter failed\n");
        return ret;
    }
    ret = ap0202at_host_command_emplace_parameter_offset_u16(pool, sizeof(pool)/sizeof(pool[0]), 8, patch_size);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_apply_patch: emplace patch size parameter failed\n");
        return ret;
    }

    // load the parameter pool.
    ret = ap0202at_host_command_load_parameter_pool(sensor, 0, pool, sizeof(pool)/sizeof(pool[0]));
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_apply_patch: load parameter pool failed\n");
        return ret;
    }

    // start the command asynchronously.
    ret = ap0202at_host_command_start_command_asynchronous(sensor, AP0202AT_HC_CMD_PATCHLDR_APPLY_PATCH, &host_command_result, timeout_start_ms);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_apply_patch: start command failed\n");
        return ret;
    }
    if ((AP0202AT_HC_RESP_ENOERR != host_command_result)
        && (AP0202AT_HC_RESP_EALREADY != host_command_result)) {
        LOG_WARNING("ap0202at_patch_manager_apply_patch: start command returned: %d\n", host_command_result);
        return ret;
    }

    // finish the command.
    ret = ap0202at_host_command_finish_command_asynchronous(sensor, AP0202AT_HC_CMD_PATCHLDR_STATUS, &host_command_result, timeout_finish_ms);
    if (STATUS_SUCCESS != ret) {
        LOG_ERROR("ap0202at_patch_manager_apply_patch: finish command failed\n");
        return ret;
    }
    if (AP0202AT_HC_RESP_ENOERR != host_command_result) {
        LOG_WARNING("ap0202at_patch_manager_apply_patch: finish command returned: %d\n", host_command_result);
        return ret;
    }

    return ret;
}

/**
 * @brief Get CCI Manager Lock.
 * 
 * AND9930/D Table 219. CCI MANAGER HOST COMMANDS
 * 
 * @param sensor the sensor struct.
 * @param timeout_start_ms timeout for starting the async command.
 * @param timeout_finish_ms timeout for finishing the async command.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_cci_manager_get_lock(sensor_t *sensor, uint16_t timeout_start_ms, uint16_t timeout_finish_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;

    // start the command asynchronously.
    ret = ap0202at_host_command_start_command_asynchronous(sensor, AP0202AT_HC_CMD_CCIMGR_GET_LOCK, &host_command_result, timeout_start_ms);
    if (STATUS_SUCCESS != ret) {
        // LOG_ERROR("ap0202at_cci_manager_get_lock: start command failed\n");
        return ret;
    }
    if ((AP0202AT_HC_RESP_ENOERR != host_command_result)
        && (AP0202AT_HC_RESP_EALREADY != host_command_result)) {
        // LOG_WARNING("ap0202at_cci_manager_get_lock: start command returned: %d\n", host_command_result);
        return ret;
    }

    // finish the command.
    ret = ap0202at_host_command_finish_command_asynchronous(sensor, AP0202AT_HC_CMD_CCIMGR_LOCK_STATUS, &host_command_result, timeout_finish_ms);
    if (STATUS_SUCCESS != ret) {
        // LOG_ERROR("ap0202at_cci_manager_get_lock: finish command failed\n");
        return ret;
    }
    if (AP0202AT_HC_RESP_ENOERR != host_command_result) {
        // LOG_WARNING("ap0202at_cci_manager_get_lock: finish command returned: %d\n", host_command_result);
        return ret;
    }

    return ret;
}

/**
 * @brief Release CCI Manager Lock.
 * 
 * @param sensor the sensor struct.
 * @param timeout_ms timeout for the command.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_cci_manager_release_lock(sensor_t *sensor, uint16_t timeout_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;

    ret = ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_CCIMGR_RELEASE_LOCK, &host_command_result, timeout_ms);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (AP0202AT_HC_RESP_ENOERR != host_command_result) {
        return STATUS_SOURCE_EXTERNAL | host_command_result;
    }

    return ret;
}

/**
 * @brief Configure the CCI Manager.
 * 
 * @param sensor the sensor struct.
 * @param cci_speed_hz the cci bus speed in hertz.
 * @param timeout_ms timeout for the command.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_cci_manager_config(sensor_t *sensor, uint32_t cci_speed_hz, uint16_t timeout_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;
    bool doorbell;

    // ensure that the doorbell bit is clear.
    ret = ap0202at_host_command_get_doorbell_bit(sensor, NULL, &doorbell);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (doorbell) {
        return STATUS_ERROR_DOORBELL;
    }

    // prepare the parameter pool.
    uint8_t pool[4];
    ret = ap0202at_host_command_emplace_parameter_offset_u32(pool, sizeof(pool)/sizeof(pool[0]), 0, cci_speed_hz);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }

    // load the parameter pool.
    ret = ap0202at_host_command_load_parameter_pool(sensor, 0, pool, sizeof(pool)/sizeof(pool[0]));
    if (STATUS_SUCCESS != ret) {
        return ret;
    }

    // execute the config command.
    ret = ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_CCIMGR_CONFIG, &host_command_result, timeout_ms);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (AP0202AT_HC_RESP_ENOERR != host_command_result) {
        return STATUS_SOURCE_EXTERNAL | host_command_result;
    }

    return ret;
}

/**
 * @brief 
 * 
 * @param sensor 
 * @param device_address 
 * @param timeout_ms 
 * @return int 
 */
ap0202at_status_t ap0202at_cci_manager_set_device(sensor_t *sensor, uint8_t device_address, uint16_t timeout_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;
    bool doorbell;

    // ensure that the doorbell bit is clear.
    ret = ap0202at_host_command_get_doorbell_bit(sensor, NULL, &doorbell);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (doorbell) {
        return STATUS_ERROR_DOORBELL;
    }

    // prepare the parameter pool.
    uint8_t pool[1];
    ret = ap0202at_host_command_emplace_parameter_offset_u8(pool, sizeof(pool)/sizeof(pool[0]), 0, device_address);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }

    // load the parameter pool.
    ret = ap0202at_host_command_load_parameter_pool(sensor, 0, pool, sizeof(pool)/sizeof(pool[0]));
    if (STATUS_SUCCESS != ret) {
        return ret;
    }

    // execute the command.
    ret = ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_CCIMGR_SET_DEVICE, &host_command_result, timeout_ms);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (AP0202AT_HC_RESP_ENOERR != host_command_result) {
        return STATUS_SOURCE_EXTERNAL | host_command_result;
    }

    return ret;
}

ap0202at_status_t ap0202at_cci_manager_read(sensor_t *sensor, uint16_t register_address, uint8_t* data, uint8_t data_len, uint16_t timeout_start_ms, uint16_t timeout_finish_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    bool doorbell;
    uint16_t host_command_result;

    // ensure that the doorbell bit is clear.
    ret = ap0202at_host_command_get_doorbell_bit(sensor, NULL, &doorbell);
    if (STATUS_SUCCESS != ret) {
        // LOG_ERROR("ap0202at_cci_manager_read: get doorbell bit failed\n");
        return ret;
    }
    if (doorbell) {
        // LOG_WARNING("ap0202at_cci_manager_read: doorbell bit is set\n");
        return STATUS_ERROR_DOORBELL;
    }

    // prepare the parameter pool.
    uint8_t pool[3];
    ret = ap0202at_host_command_emplace_parameter_offset_u16(pool, sizeof(pool)/sizeof(pool[0]), 0, register_address);
    if (STATUS_SUCCESS != ret) {
        // LOG_ERROR("ap0202at_cci_manager_read: emplace parameter failed\n");
        return ret;
    }
    ret = ap0202at_host_command_emplace_parameter_offset_u8(pool, sizeof(pool)/sizeof(pool[0]), 2, data_len);
    if (STATUS_SUCCESS != ret) {
        // LOG_ERROR("ap0202at_cci_manager_read: emplace parameter failed\n");
        return ret;
    }

    // load the parameter pool.
    ret = ap0202at_host_command_load_parameter_pool(sensor, 0, pool, sizeof(pool)/sizeof(pool[0]));
    if (STATUS_SUCCESS != ret) {
        // LOG_ERROR("ap0202at_cci_manager_read: load parameter pool failed\n");
        return ret;
    }

    // start the command asynchronously.
    ret = ap0202at_host_command_start_command_asynchronous(sensor, AP0202AT_HC_CMD_CCIMGR_READ, &host_command_result, timeout_start_ms);
    if (STATUS_SUCCESS != ret) {
        // LOG_ERROR("ap0202at_cci_manager_read: start command failed\n");
        return ret;
    }
    if (AP0202AT_HC_RESP_ENOERR != host_command_result) {
        // LOG_WARNING("ap0202at_cci_manager_read: start command returned: %d\n", host_command_result);
        return ret;
    }

    // finish the command.
    ret = ap0202at_host_command_finish_command_asynchronous(sensor, AP0202AT_HC_CMD_CCIMGR_STATUS, &host_command_result, timeout_finish_ms);
    if (STATUS_SUCCESS != ret) {
        // LOG_ERROR("ap0202at_cci_manager_read: finish command failed\n");
        return ret;
    }
    if (AP0202AT_HC_RESP_ENOERR != host_command_result) {
        // LOG_ERROR("ap0202at_cci_manager_read: finish command returned: %d\n", host_command_result);
        return STATUS_SOURCE_EXTERNAL | host_command_result;
    }

    // unload the parameter pool.
    ret = ap0202at_host_command_unload_parameter_pool(sensor, 0, data, data_len);
    if (STATUS_SUCCESS != ret) {
        // LOG_ERROR("ap0202at_cci_manager_read: unload parameter pool failed\n");
        return ret;
    }

    return ret;
}

ap0202at_status_t ap0202at_cci_manager_write(sensor_t *sensor, uint16_t register_address, uint8_t* data, uint8_t data_len, uint16_t timeout_start_ms, uint16_t timeout_finish_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;
    bool doorbell;

    // ensure that the doorbell bit is clear.
    ret = ap0202at_host_command_get_doorbell_bit(sensor, NULL, &doorbell);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (doorbell) {
        return STATUS_ERROR_DOORBELL;
    }

    // prepare the parameter pool.
    uint8_t pool[3];
    ret = ap0202at_host_command_emplace_parameter_offset_u16(pool, sizeof(pool)/sizeof(pool[0]), 0, register_address);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    ret = ap0202at_host_command_emplace_parameter_offset_u8(pool, sizeof(pool)/sizeof(pool[0]), 2, data_len);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }

    // load the parameter pool.
    ret = ap0202at_host_command_load_parameter_pool(sensor, 0, pool, sizeof(pool)/sizeof(pool[0]));
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    ret = ap0202at_host_command_load_parameter_pool(sensor, 3, data, data_len);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }

    // start the command asynchronously.
    ret = ap0202at_host_command_start_command_asynchronous(sensor, AP0202AT_HC_CMD_CCIMGR_WRITE, &host_command_result, timeout_start_ms);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (AP0202AT_HC_RESP_ENOERR != host_command_result) {
        return STATUS_SOURCE_EXTERNAL | host_command_result;
    }

    // finish the command.
    ret = ap0202at_host_command_finish_command_asynchronous(sensor, AP0202AT_HC_CMD_CCIMGR_STATUS, &host_command_result, timeout_finish_ms);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (AP0202AT_HC_RESP_ENOERR != host_command_result) {
        return STATUS_SOURCE_EXTERNAL | host_command_result;
    }

    return ret;
}

/**
 * @brief Perform the sensor discovery routine.
 * 
 * AND9930/D DISCOVER SENSOR HOST COMMAND.
 * 
 * @param sensor 
 * @param cci_address 
 * @param revision 
 * @param model_id 
 * @param timeout_ms 
 * @return int 
 */
ap0202at_status_t ap0202at_sensor_manager_discover_sensor(sensor_t *sensor, uint8_t *cci_address, uint8_t *revision, uint16_t *model_id, uint16_t timeout_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;
    bool doorbell;

    // ensure that the doorbell bit is clear.
    ret = ap0202at_host_command_get_doorbell_bit(sensor, NULL, &doorbell);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (doorbell) {
        return STATUS_ERROR_DOORBELL;
    }

    // execute the command.
    ret = ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_SENSOR_MGR_DISCOVER_SENSOR, &host_command_result, timeout_ms);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (AP0202AT_HC_RESP_ENOERR != host_command_result) {
        return STATUS_SOURCE_EXTERNAL | host_command_result;
    }

    // unload the parameter pool.
    uint8_t pool[4];
    ret = ap0202at_host_command_unload_parameter_pool(sensor, 0, pool, sizeof(pool)/sizeof(pool[0]));
    if (STATUS_SUCCESS != ret) {
        return ret;
    }

    // return the results.
    LOG_INFO("pool: 0x%0X 0x%0X 0x%0X 0x%0X\n", pool[0], pool[1], pool[2], pool[3]);

    if (cci_address != NULL) {
        *cci_address = pool[0];
    }
    if (revision != NULL) {
        *revision = pool[1];
    }
    if (model_id != NULL) {
        *model_id = (pool[2] << 8) | pool[3];
    }

    return ret;
}

ap0202at_status_t ap0202at_sensor_manager_initialize_sensor(sensor_t *sensor, uint16_t timeout_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;
    bool doorbell;

    // ensure that the doorbell bit is clear.
    ret = ap0202at_host_command_get_doorbell_bit(sensor, NULL, &doorbell);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (doorbell) {
        return STATUS_ERROR_DOORBELL;
    }

    // execute the command.
    ret = ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_SENSOR_MGR_INITIALIZE_SENSOR, &host_command_result, timeout_ms);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (AP0202AT_HC_RESP_ENOERR != host_command_result) {
        return STATUS_SOURCE_EXTERNAL | host_command_result;
    }

    return ret;
}

/**
 * @brief Enter configuration mode via hardware reset.
 * @details Toggles the RESET_BAR pin for the required
 *   duration to perform a reset. Immediately exits
 *   reset state in order to enter configuration mode.
 * 
 * The state of the SPI_SDI pin will determine the
 *   flow of control through the configuration process
 *   after reset.
 * 
 * The minimum duration of the LOW RESET_BAR pulse is
 *   20 clock cycles. (Datasheet Table 7. HARD RESET)
 * 
 * @param sensor the sensor struct.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_enter_configuration_mode_hardware(sensor_t *sensor) {
    ap0202at_status_t ret = STATUS_SUCCESS;

    // Hardware reset is not implemented.
    ret = -1;

    return ret;
}

/**
 * @brief Enter configuration mode via software reset.
 * @details Both enters and exits the software reset state
 *   in order to enter configuration mode.
 * 
 * The state of the SPI_SDI pin will determine the
 *   flow of control through the configuration process
 *   after reset.
 * 
 * @param sensor the sensor struct.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_enter_configuration_mode_software(sensor_t *sensor) {
    ap0202at_status_t ret = STATUS_SUCCESS;

    // Enter software reset.
    ret = ap0202at_write_reg_masked(
            sensor,
            AP0202AT_REG_SYSCTL_RESET_AND_MISC_CONTROL,
            AP0202AT_SYSCTL_RESET_AND_MISC_CONTROL_RESET_SOFT,
            AP0202AT_SYSCTL_RESET_AND_MISC_CONTROL_RESET_SOFT_MASK
        );
    if (STATUS_SUCCESS != ret) {
        return ret;
    }

    return ret;
}

/**
 * @brief Reset the AP0202AT ISP into the default state.
 * 
 * This function also resets the attached image sensor.
 * 
 * @param sensor Pointer to sensor struct.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_reset(sensor_t *sensor) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;

    // Use software reset to enter configuration mode.
    ret = ap0202at_enter_configuration_mode_software(sensor);
    if (STATUS_SUCCESS != ret) {
        // LOG_ERROR("ap0202at_reset: enter configuration mode failed\n");
        return ret;
    }

    // Wait for the Doorbell Bit to clear.
    // This is standard practice when entering
    //   configuration mode and is particularly called
    //   out if the Flash-Config mode is used to
    //   indicate that the flash download is complete.
    ret = ap0202at_host_command_poll_doorbell_bit_clear(sensor, NULL, 250);
    if (STATUS_SUCCESS != ret) {
        // LOG_ERROR("ap0202at_reset: poll doorbell bit clear failed\n");
        return ret;
    }

    // Issue SYSMGR_GET_STATE repeatedly until the result
    //   is not EBUSY.
    for (mp_uint_t start = mp_hal_ticks_ms();;) {
        ret = ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_SYSMGR_GET_STATE, &host_command_result, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS);
        if  (ret != STATUS_SUCCESS) {
            // LOG_ERROR("ap0202at_reset: get state failed\n");
            return ret;
        }
        if (host_command_result != AP0202AT_HC_RESP_EBUSY) {
            break;
        }

        // Return error if the timeout has elapsed.
        if ((mp_hal_ticks_ms() - start) >= AP0202AT_RESET_GET_STATE_TIMEOUT_MS) {
            // LOG_WARNING("ap0202at_reset: get state timeout\n");
            return STATUS_ERROR_TIMEOUT;
        }
    }

    return ret;
}

/**
 * @brief Issue the SYSMGR SET_STATE command to enter 'state'.
 * 
 * @param sensor 
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_sysmgr_set_state(sensor_t *sensor, uint8_t state, uint16_t timeout_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;
    bool doorbell;

    // ensure that the doorbell bit is clear.
    ret = ap0202at_host_command_get_doorbell_bit(sensor, NULL, &doorbell);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (doorbell) {
        return STATUS_ERROR_DOORBELL;
    }

    // prepare the parameter pool.
    uint8_t pool[1];
    ret = ap0202at_host_command_emplace_parameter_offset_u8(pool, sizeof(pool)/sizeof(pool[0]), 0, state);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    
    // load the parameter pool.
    ret = ap0202at_host_command_load_parameter_pool(sensor, 0, pool, sizeof(pool)/sizeof(pool[0]));
    if (STATUS_SUCCESS != ret) {
        return ret;
    }

    // execute the command.
    ret = ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_SYSMGR_SET_STATE, &host_command_result, timeout_ms);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (AP0202AT_HC_RESP_ENOERR != host_command_result) {
        return STATUS_SOURCE_EXTERNAL | host_command_result;
    }

    return ret;
}

/**
 * @brief Issue the SYSMGR GET_STATE command to read the current state.
 * 
 * @param sensor the sensor struct.
 * @param state [output] the current state.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_sysmgr_get_state(sensor_t *sensor, uint8_t *state, uint16_t timeout_ms) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;
    bool doorbell;
    if (NULL == state) {
        return STATUS_ERROR_EINVAL;
    }

    // check the doorbell bit, as some other command
    // may be running.
    ret = ap0202at_host_command_get_doorbell_bit(sensor, NULL, &doorbell);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (doorbell) {
        return STATUS_ERROR_DOORBELL;
    }

    // Issue the command.
    ret = ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_SYSMGR_GET_STATE, &host_command_result, timeout_ms);
    if (STATUS_SUCCESS != ret) {
        return ret;
    }
    if (host_command_result != AP0202AT_HC_RESP_ENOERR) {
        return ret;
    }

    // Unload the parameter pool.
    uint8_t pool[1];
    ret = ap0202at_host_command_unload_parameter_pool(sensor, 0, pool, sizeof(pool)/sizeof(pool[0]));
    if (STATUS_SUCCESS != ret) {
        return ret;
    }

    // Return the state.
    *state = pool[0];

    return ret;
}

/**
 * @brief Read a register from the AP0202AT ISP.
 * @details Uses the two-wire interface to read a 16-bit
 *   wide register from the AP0202AT ISP.
 *   This function is meant to be used in the `read_reg`
 *   field of the sensor_t struct.
 * 
 * @param sensor the sensor struct.
 * @param reg_addr the register address.
 * @return int -1 in case of error, else register contents.
 */
static int read_reg(sensor_t *sensor, uint16_t reg_addr) {
    uint16_t reg_data;

    // Translate OpenMV return code.
    if (ap0202at_read_reg_direct(sensor, reg_addr, &reg_data) != 0) {
        return -1;
    }

    return reg_data;
}

/**
 * @brief Write data to a register in the AP0202AT ISP.
 * @details Uses the two-wire interface to write a 16-bit
 *   wide register in the AP0202AT ISP.
 *   This function is meant to be used in the `write_reg`
 *   field of the sensor_t struct.
 * 
 * @param sensor the sensor struct.
 * @param reg_addr the register address at which the data
 *   will be written.
 * @param data the data to be written to the register.
 * @return ap0202at_status_t.
 */
static int write_reg(sensor_t *sensor, uint16_t reg_addr, uint16_t data) {
    ap0202at_status_t ret = STATUS_SUCCESS;

    // Translate OpenMV return code.
    if(ap0202at_write_reg_direct(sensor, reg_addr, data) != 0) {
        ret = -1;
    }

    return ret;
}

/**
 * @brief Initialize the sensor structure for the AP0202AT.
 * @details Additional fields may be set on the sensor
 *  struct but these fields should be kept.
 * 
 * @param sensor the sensor struct.
 * @return ap0202at_status_t.
 */
ap0202at_status_t ap0202at_init(sensor_t *sensor) {
    sensor->read_reg = read_reg;
    sensor->write_reg = write_reg;

    return 0;
}

#endif // (OMV_AP0202AT_ENABLE == 1)
