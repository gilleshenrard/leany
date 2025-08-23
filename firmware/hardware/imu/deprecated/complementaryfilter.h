/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef HARDWARE_IMU_COMPLEMENTARY_FILTER_H
#define HARDWARE_IMU_COMPLEMENTARY_FILTER_H
#include "sensorfusion.h"

void complementaryFilter(const IMUsample* sample, float filtered_angles_rad[]);
#endif
