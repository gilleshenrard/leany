/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef HARDWARE_IMU_LSM6DSR_LSM6DSR_REGISTERS_H_INCLUDED
#define HARDWARE_IMU_LSM6DSR_LSM6DSR_REGISTERS_H_INCLUDED
#include "lsm6_registers.inc"  // NOLINT(unused-includes)

// WhoAmI register (0x0F) values
#undef LSM6_WHOAMI
#define LSM6_WHOAMI 0x6BU  ///< LSM6DSR Who Am I constant value

typedef enum {
    kS4S_TPH_L = 0x04U,        ///< RW - Sensor synchronization time frame register (LSB)
    kS4S_TPH_H = 0x05U,        ///< RW - Sensor synchronization time frame register (MSB)
    kS4S_RR = 0x06U,           ///< RW - Sensor synchronization resolution ratio register
    kS4S_ST_CMD_CODE = 0x60U,  ///< RW - Sensor synchronization resolution ratio register
    kS4S_DT_REG = 0x61U,       ///< RW - Sensor synchronization resolution ratio register
} LSM6DSRregister;

typedef enum {
    kADV_PEDO = 0x03U,  ///< RW - Enable/disable pedometer advanced features register
} LSM6DSRembeddedFunction;

#endif
