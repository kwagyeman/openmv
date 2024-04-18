/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2024 oclyke <oclyke@oclyke.dev>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * AP0202AT generic sensor utilities.
 */
#ifndef __AP0202AT_GENERIC_H__
#define __AP0202AT_GENERIC_H__

// #include <stdint.h>

#include "sensor.h"

int ap0202at_detect_self(sensor_t *sensor, bool *detected);
int ap0202at_detect_sensor_ar0147(sensor_t *sensor, bool *detected);
int ap0202at_detect_sensor_ar0231at(sensor_t *sensor, bool *detected);

#endif // __AP0202AT_H__
