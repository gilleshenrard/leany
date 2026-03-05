/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file task_gpio.h
 * @author Gilles Henrard
 */
#ifndef HARDWARE_GPIO_GPIO_H
#define HARDWARE_GPIO_GPIO_H
#include <stdint.h>

void createGPIOtask(void);
uint8_t getInternalTemperatureCelsius(int32_t* temperature_celsius);
uint8_t getADCreference_mV(uint32_t* voltage_mv);
#endif
