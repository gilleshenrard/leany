/*
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef HARDWARE_GPIO_USER_ADC_H
#define HARDWARE_GPIO_USER_ADC_H
#include <stdint.h>

void ADCinterruptTriggered(void);
void initialiseUserADC(void);
void runUserADCstateMachine(void);
uint8_t getInternalTemperatureCelsius(int32_t* temperature_celsius);

#endif
