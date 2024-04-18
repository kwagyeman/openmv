/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2023-2024 oclyke <oclyke@oclyke.dev>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * AR0147 w/ AP0202AT ISP driver.
 */
#include "omv_boardconfig.h"
#if (OMV_AP0202AT_AR0147_ENABLE == 1)

#include "py/mphal.h"

#include "sensor.h"
#include "omv_log.h"

#include "ap0202at.h"
#include "ap0202at_regs.h"
#include "ap0202at_ar0231at_patches.h"

// Maximum time in milliseconds before host command
//   polling times out.
#define AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS (100)
#define AP0202AT_HOST_COMMAND_READ_POLL_TIMEOUT_MS (100)

/**
 * @brief Loads and applies patch 0056.
 * 
 * AP0202AT-REV2_AR0231AT-REV7.ini line 1135
 * 
 * @param sensor the sensor struct.
 * @return int 0 on success, -1 on error.
 */
static int load_apply_patch_0056(sensor_t *sensor) {
    (void)patch_0056;
    return STATUS_ERROR;
    // ap0202at_status_t ret = STATUS_SUCCESS;
    // uint16_t patch_addr = 0x0;
    // uint16_t patch_size = 0x614;

    // // AP0202AT-REV2_AR0231AT-REV7.ini line 1141
    // ret = ap0202at_patch_manager_reserve_ram(sensor, patch_addr, patch_size, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS);
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }
    
    // // AP0202AT-REV2_AR0231AT-REV7.ini line 1149
    // ret = ap0202at_patch_manager_write_patch_to_ram(sensor, 0x4750, patch_0056_data, sizeof(patch_0056_data) / sizeof(patch_0056_data[0]));
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // // AP0202AT-REV2_AR0231AT-REV7.ini line 1188
    // ret = ap0202at_patch_manager_apply_patch(
    //     sensor,
    //     0x049c, 0x0056, PATCHLDR_MAGIC_FIRMWARE_ID, patch_size,
    //     AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS
    // );
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // return ret;
}

static int load_apply_patch_0156(sensor_t *sensor) {
    (void)patch_0156;
    return STATUS_ERROR;
    // ap0202at_status_t ret = STATUS_SUCCESS;
    // uint16_t patch_addr = 0x614;
    // uint16_t patch_size = 0x54;

    // ret = ap0202at_patch_manager_reserve_ram(sensor, patch_addr, patch_size, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS);
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // ret = ap0202at_patch_manager_write_patch_to_ram(sensor, 0x4d64, patch_0156_data, sizeof(patch_0156_data) / sizeof(patch_0156_data[0]));
    // if (ret != STATUS_SUCCESS) {
    //     return -1;
    // }
    
    // ret = ap0202at_patch_manager_apply_patch(
    //     sensor,
    //     0x049c, 0x0156, PATCHLDR_MAGIC_FIRMWARE_ID, patch_size,
    //     AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS
    // );
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // return ret;
}

static int load_apply_patch_0256(sensor_t *sensor) {
    (void)patch_0256;
    return STATUS_ERROR;
    // ap0202at_status_t ret = STATUS_SUCCESS;
    // uint16_t patch_addr = 0x668;
    // uint16_t patch_size = 0x620;

    // ret = ap0202at_patch_manager_reserve_ram(sensor, patch_addr, patch_size, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS);
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // ret = ap0202at_patch_manager_write_patch_to_ram(sensor, 0x4db8, patch_0256_data, sizeof(patch_0256_data) / sizeof(patch_0256_data[0]));
    // if (ret != STATUS_SUCCESS) {
    //     return -1;
    // }

    // ret = ap0202at_patch_manager_apply_patch(
    //     sensor,
    //     0x0c20, 0x0256, PATCHLDR_MAGIC_FIRMWARE_ID, patch_size,
    //     AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS
    // );
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // return ret;
}

static int load_apply_patch_0356(sensor_t *sensor) {
    (void)patch_0356;
    return STATUS_ERROR;
    // ap0202at_status_t ret = STATUS_SUCCESS;
    // uint16_t patch_addr = 0xc88;
    // uint16_t patch_size = 0x98;

    // ret = ap0202at_patch_manager_reserve_ram(sensor, patch_addr, patch_size, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS);
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // ret = ap0202at_patch_manager_write_patch_to_ram(sensor, 0x53d8, patch_0356_data, sizeof(patch_0356_data) / sizeof(patch_0356_data[0]));
    // if (ret != STATUS_SUCCESS) {
    //     return -1;
    // }

    // ret = ap0202at_patch_manager_apply_patch(
    //     sensor,
    //     0x0cfc, 0x0356, PATCHLDR_MAGIC_FIRMWARE_ID, patch_size,
    //     AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS
    // );
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // return ret;
}

static int load_apply_patch_0456(sensor_t *sensor) {
    (void)patch_0456;
    return STATUS_ERROR;
//     ap0202at_status_t ret = STATUS_SUCCESS;
//     uint16_t patch_addr = 0xd20;
//     uint16_t patch_size = 0x12d0;

//     ret = ap0202at_patch_manager_reserve_ram(sensor, patch_addr, patch_size, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS);
//     if (ret != STATUS_SUCCESS) {
//         return ret;
//     }

//     ret = ap0202at_patch_manager_write_patch_to_ram(sensor, 0x5470, patch_0456_data, sizeof(patch_0456_data) / sizeof(patch_0456_data[0]));
//     if (ret != STATUS_SUCCESS) {
//         return -1;
//     }

//     ret = ap0202at_patch_manager_apply_patch(
//         sensor,
//         0x1d40, 0x0456, PATCHLDR_MAGIC_FIRMWARE_ID, patch_size,
//         AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS
//     );
//     if (ret != STATUS_SUCCESS) {
//         return ret;
//     }

//     return ret;
}

static int load_apply_patch_1156(sensor_t *sensor) {
    (void)patch_1156;
    return STATUS_ERROR;
    // ap0202at_status_t ret = STATUS_SUCCESS;
    // uint16_t patch_addr = 0x1ff0;
    // uint16_t patch_size = 0x118;

    // ret = ap0202at_patch_manager_reserve_ram(sensor, patch_addr, patch_size, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS);
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // ret = ap0202at_patch_manager_write_patch_to_ram(sensor, 0x6740, patch_1156_data, sizeof(patch_1156_data) / sizeof(patch_1156_data[0]));
    // if (ret != STATUS_SUCCESS) {
    //     return -1;
    // }

    // ret = ap0202at_patch_manager_apply_patch(
    //     sensor,
    //     0x20a0, 0x1156, PATCHLDR_MAGIC_FIRMWARE_ID, patch_size,
    //     AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS
    // );
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // return ret;
}

static int load_apply_patch_1356(sensor_t *sensor) {
    (void)patch_1356;
    return STATUS_ERROR;
    // ap0202at_status_t ret = STATUS_SUCCESS;
    // uint16_t patch_addr = 0x2108;
    // uint16_t patch_size = 0x414;

    // ret = ap0202at_patch_manager_reserve_ram(sensor, patch_addr, patch_size, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS);
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }     

    // ret = ap0202at_patch_manager_write_patch_to_ram(sensor, 0x6858, patch_1356_data, sizeof(patch_1356_data) / sizeof(patch_1356_data[0]));
    // if (ret != STATUS_SUCCESS) {
    //     return -1;
    // }

    // ret = ap0202at_patch_manager_apply_patch(
    //     sensor,
    //     0x2108, 0x1356, PATCHLDR_MAGIC_FIRMWARE_ID, patch_size,
    //     AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS
    // );
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // return ret;
}

static int load_apply_patch_1456(sensor_t *sensor) {
    (void)patch_1456;
    return STATUS_ERROR;
    // ap0202at_status_t ret = STATUS_SUCCESS;
    // uint16_t patch_addr = 0x25c1;
    // uint16_t patch_size = 0xdc;

    // ret = ap0202at_patch_manager_reserve_ram(sensor, patch_addr, patch_size, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS);
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // ret = ap0202at_patch_manager_write_patch_to_ram(sensor, 0x6c6c, patch_1456_data, sizeof(patch_1456_data) / sizeof(patch_1456_data[0]));
    // if (ret != STATUS_SUCCESS) {
    //     return -1;
    // }

    // ret = ap0202at_patch_manager_apply_patch(
    //     sensor,
    //     0x25cc, 0x1456, PATCHLDR_MAGIC_FIRMWARE_ID, patch_size,
    //     AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS
    // );
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // return ret;
}

static int load_apply_patch_1556(sensor_t *sensor) {
    (void)patch_1556;
    return STATUS_ERROR;
    // ap0202at_status_t ret = STATUS_SUCCESS;
    // uint16_t patch_addr = 0x25f8;
    // uint16_t patch_size = 0x1f8;

    // ret = ap0202at_patch_manager_reserve_ram(sensor, patch_addr, patch_size, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS);
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // ret = ap0202at_patch_manager_write_patch_to_ram(sensor, 0x6d48, patch_1556_data, sizeof(patch_1556_data) / sizeof(patch_1556_data[0]));
    // if (ret != STATUS_SUCCESS) {
    //     return -1;
    // }

    // ret = ap0202at_patch_manager_apply_patch(
    //     sensor,
    //     0x2750, 0x1556, PATCHLDR_MAGIC_FIRMWARE_ID, patch_size,
    //     AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS
    // );
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // return ret;
}

static int load_apply_patch_1756(sensor_t *sensor) {
    (void)patch_1756;
    return STATUS_ERROR;
    // ap0202at_status_t ret = STATUS_SUCCESS;
    // uint16_t patch_addr = 0x27f0;
    // uint16_t patch_size = 0x60;

    // ret = ap0202at_patch_manager_reserve_ram(sensor, patch_addr, patch_size, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS);
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // ret = ap0202at_patch_manager_write_patch_to_ram(sensor, 0x6f40, patch_1756_data, sizeof(patch_1756_data) / sizeof(patch_1756_data[0]));
    // if (ret != STATUS_SUCCESS) {
    //     return -1;
    // }

    // ret = ap0202at_patch_manager_apply_patch(
    //     sensor,
    //     0x283c, 0x1756, PATCHLDR_MAGIC_FIRMWARE_ID, patch_size,
    //     AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS
    // );
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // return ret;
}

static int load_apply_patch_1956(sensor_t *sensor) {
    (void)patch_1956;
    return STATUS_ERROR;
    // ap0202at_status_t ret = STATUS_SUCCESS;
    // uint16_t patch_addr = 0x2850;
    // uint16_t patch_size = 0xa0;

    // ret = ap0202at_patch_manager_reserve_ram(sensor, patch_addr, patch_size, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS);
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // ret = ap0202at_patch_manager_write_patch_to_ram(sensor, 0x6fa0, patch_1956_data, sizeof(patch_1956_data) / sizeof(patch_1956_data[0]));
    // if (ret != STATUS_SUCCESS) {
    //     return -1;
    // }

    // ret = ap0202at_patch_manager_apply_patch(
    //     sensor,
    //     0x28cc, 0x1956, PATCHLDR_MAGIC_FIRMWARE_ID, patch_size,
    //     AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS
    // );
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // return ret;
}

static int load_apply_patch_2156(sensor_t *sensor) {
    (void)patch_2156;
    return STATUS_ERROR;
    // ap0202at_status_t ret = STATUS_SUCCESS;
    // uint16_t patch_addr = 0x28f0;
    // uint16_t patch_size = 0xb8;

    // ret = ap0202at_patch_manager_reserve_ram(sensor, patch_addr, patch_size, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS);
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // ret = ap0202at_patch_manager_write_patch_to_ram(sensor, 0x7040, patch_2156_data, sizeof(patch_2156_data) / sizeof(patch_2156_data[0]));
    // if (ret != STATUS_SUCCESS) {
    //     return -1;
    // }

    // ret = ap0202at_patch_manager_apply_patch(
    //     sensor,
    //     0x297c, 0x2156, PATCHLDR_MAGIC_FIRMWARE_ID, patch_size,
    //     AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS
    // );
    // if (ret != STATUS_SUCCESS) {
    //     return ret;
    // }

    // return ret;
}

/**
 * @brief Loads all applicable patches for the combination
 *   of the AP0202AT ISP and the image sensor.
 * 
 * @param sensor the sensor struct.
 * @return int 0 on success, -1 on error.
 */
static int load_patches(sensor_t *sensor) {
    ap0202at_status_t ret = STATUS_SUCCESS;

    // AP0202AT-REV2_AR0231AT-REV7.ini line 1118

    if (load_apply_patch_0056(sensor) != 0) {
        return -1;
    }
    if (load_apply_patch_0156(sensor) != 0) {
        return -1;
    }
    if (load_apply_patch_0256(sensor) != 0) {
        return -1;
    }
    if (load_apply_patch_0356(sensor) != 0) {
        return -1;
    }
    if (load_apply_patch_0456(sensor) != 0) {
        return -1;
    }
    if (load_apply_patch_1156(sensor) != 0) {
        return -1;
    }
    if (load_apply_patch_1356(sensor) != 0) {
        return -1;
    }
    if (load_apply_patch_1456(sensor) != 0) {
        return -1;
    }
    if (load_apply_patch_1556(sensor) != 0) {
        return -1;
    }
    if (load_apply_patch_1756(sensor) != 0) {
        return -1;
    }
    if (load_apply_patch_1956(sensor) != 0) {
        return -1;
    }
    if (load_apply_patch_2156(sensor) != 0) {
        return -1;
    }

    return ret;
}

static int sensor_do_sequencer(sensor_t *sensor) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;

    // AP0202AT-REV2_AR0231AT-REV7.ini line 1042
    if  (ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_CCIMGR_GET_LOCK, &host_command_result, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (host_command_result != AP0202AT_HC_RESP_ENOERR) {
        return -1;
    }

    // AP0202AT-REV2_AR0231AT-REV7.ini line 1043
    if  (ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_CCIMGR_LOCK_STATUS, &host_command_result, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    LOG_WARNING("UNCHECKED HOST COMMAND RESULT: host_command_result: %d\n", host_command_result);
    // if (host_command_result != AP0202AT_HC_CMD_CCIMGR_LOCK_STATUS_RESP_LOCKED) {
    //     return -1;
    // }

    // AP0202AT-REV2_AR0231AT-REV7.ini line 1047
    if (ap0202at_write_sensor_u16(sensor, 0x2512,0x8000, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }

    // AP0202AT-REV2_AR0231AT-REV7.ini line 1066
    if (ap0202at_write_sensor_sequencer(sensor, 0x2510, ar0231at_sequencer_data, sizeof(ar0231at_sequencer_data)/ sizeof(uint16_t)) != 0) {
        return -1;
    }

    // AP0202AT-REV2_AR0231AT-REV7.ini line 1068
    if  (ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_CCIMGR_RELEASE_LOCK, &host_command_result, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (host_command_result != AP0202AT_HC_RESP_ENOERR) {
        return -1;
    }

    return ret;
}

static int sensor_post_initialization(sensor_t *sensor) {
    ap0202at_status_t ret = STATUS_SUCCESS;
    uint16_t host_command_result;

    // AP0202AT-REV2_AR0231AT-REV7.ini line 83

    // Get the CCIM lock
    if  (ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_CCIMGR_GET_LOCK, &host_command_result, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (host_command_result != AP0202AT_HC_RESP_ENOERR) {
        return -1;
    }

    // AP0202AT-REV2_AR0231AT-REV7.ini line 985
    if (ap0202at_write_sensor_u16(sensor, 0x318E, 0x0200, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3092, 0x0C24, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x337A, 0x0C80, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3520, 0x1288, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3522, 0x880C, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3524, 0x0C12, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x352C, 0x1212, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x354A, 0x007F, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x350C, 0x055C, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    } 
    if (ap0202at_write_sensor_u16(sensor, 0x3506, 0x3333, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3508, 0x3333, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3100, 0x4000, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3280, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3282, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3284, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3286, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3288, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x328A, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x328C, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x328E, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3290, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3292, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3294, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3296, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3298, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x329A, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x329C, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x329E, 0x0FA0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS,  AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }

    if (ap0202at_write_sensor_u16(sensor, 0x32E6, 0xE0, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x1008, 0x036F, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x100C, 0x058F, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x100E, 0x07AF, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x1010, 0x014F, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3230, 0x0312, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3232, 0x0532, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3234, 0x0752, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x3236, 0x0F2, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }

    if (ap0202at_write_sensor_u16(sensor, 0x32D0, 0x3A02, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x32D2, 0x3508, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x32D4, 0x3702, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x32D6, 0x3C04, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x32DC, 0x370A, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (ap0202at_write_sensor_u16(sensor, 0x0566, 0x3328, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }

    // Release the CCIM lock
    if  (ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_CCIMGR_RELEASE_LOCK, &host_command_result, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
        return -1;
    }
    if (host_command_result != AP0202AT_HC_RESP_ENOERR) {
        return -1;
    }

    return ret;
}

// /**
//  * @brief Reset the image sensor into the default state.
//  * 
//  * @param sensor the sensor struct.
//  * @return int 0 on success, -1 on error.
//  */
// static int reset_image_sensor(sensor_t *sensor) {
//     ap0202at_status_t ret = STATUS_SUCCESS;
// 
//     if (ap0202at_write_reg_masked(
//             sensor,
//             AP0202AT_VAR_SENSOR_MGR_MODE,
//             AP0202AT_VAR_SENSOR_MGR_SENSOR_DEFAULT_SEQUENCER_LOAD_INHIBIT_TRUE,
//             AP0202AT_VAR_SENSOR_MGR_SENSOR_DEFAULT_SEQUENCER_LOAD_INHIBIT_MASK
//         ) != 0) {
//         return -1;
//     }

//     if (sensor_do_sequencer(sensor) != 0) {
//         return -1;
//     }

//     // Sensor initialization
//     // APA0202AT-REV2.ini line 304
//     if  (ap0202at_host_command_execute_command_synchronous(sensor, AP0202AT_HC_CMD_SENSOR_MGR_INITIALIZE_SENSOR, &host_command_result, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS) != 0) {
//         return -1;
//     }
//     if (host_command_result != AP0202AT_HC_RESP_ENOERR) {
//         return -1;
//     }

//     if (sensor_post_initialization(sensor) != 0) {
//         return -1;
//     }

//     return ret;
// }

/**
 * @brief Reset the AP0202AT ISP and the AR0147 into the
 *   default state.
 * 
 * - 
 * 
 * @param sensor Pointer to sensor struct.
 * @return int 0 on success, -1 on error.
 */
static int reset(sensor_t *sensor) {
    return -1;

    // some unused functions that cause errors
    (void)sensor_do_sequencer;
    (void)sensor_post_initialization;
    (void)load_patches;
    
    // ap0202at_status_t ret = STATUS_SUCCESS;
    // uint8_t state;

    // // Reset the AP0202AT
    // // APA0202AT-REV2.ini line 298
    // ret = ap0202at_reset(sensor);
    // if (ret != 0) {
    //     return -1;
    // }

    // // Load patches for the AP0202AT ISP.
    // if(load_patches(sensor) != 0) {
    //     return -1;
    // }

    // // Perform sensor discovery and fail if no sensor is
    // //   found.
    // uint8_t cci_address;
    // uint8_t revision;
    // uint16_t model_id;
    // ret = ap0202at_sensor_manager_discover_sensor(sensor, &cci_address, &revision, &model_id, 100);
    // if (ret != 0) {
    //     // printf("Failed to discover sensor\n");
    //     return -1;
    // }
    // // printf("Discovered sensor at 0x%0X\n", cci_address);
    // // printf("Sensor revision: 0x%0X\n", revision);
    // // printf("Sensor model ID: 0x%04X\n", model_id);

    // // Perform sensor configuration.
    // if(reset_image_sensor(sensor) != 0) {
    //     return -1;
    // }

    // // Issue the Change Config command to enter streaming.
    // ret = ap0202at_sysmgr_set_state(sensor, AP0202AT_HCI_SYS_STATE_STREAMING, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS);
    // if (ret != 0) {
    //     // printf("Failed to issue set state command.\n");
    //     return -1;
    // }

    // ret = ap0202at_sysmgr_get_state(sensor, &state, AP0202AT_HOST_COMMAND_ISSUE_POLL_TIMEOUT_MS);
    // if (ret != 0) {
    //     // printf("Failed to issue get state command\n");
    //     return -1;
    // }
    // if (state != AP0202AT_HCI_SYS_STATE_STREAMING) {
    //     // printf("Failed to enter streaming state. Found state 0x%X instead\n", state);
    //     return -1;
    // }

    // return ret;
}

/**
 * @brief Initialize the sensor structure for the AP0202AT.
 * 
 * @param sensor the sensor struct.
 * @return int 0 on success, -1 on error.
 */
int ap0202at_ar0231_init(sensor_t *sensor) {
    ap0202at_status_t ret = STATUS_SUCCESS;

    // Initialize the AP0202AT portions of the sensor
    //   structure.
    ret = ap0202at_init(sensor);
    if (ret != 0) {
        return -1;
    }

    sensor->reset = reset;

    return 0;
}

#endif // (OMV_AP0202AT_AR0147_ENABLE == 1)
