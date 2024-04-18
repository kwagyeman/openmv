/*
 * This file is part of the OpenMV project.
 *
 * Copyright (c) 2023-2024 oclyke <oclyke@oclyke.dev>
 *
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * AR0147 w/ AP0202AT ISP driver.
 */
#ifndef __AP0202AT_AR0147_H__
#define __AP0202AT_AR0147_H__

#include "ap0202at.h"

int ap0202at_ar0147_init(sensor_t *sensor);
int ap0202at_ar0147_init0(sensor_t *sensor);

#endif // __AP0202AT_AR0147_H__
