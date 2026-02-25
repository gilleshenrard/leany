/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file hal_adc.h
 * @author Gilles Henrard
 */
#ifndef HARDWARE_HAL_HAL_ADC_H
#define HARDWARE_HAL_HAL_ADC_H
#include <stdint.h>

typedef enum { kADCchannelTemperature = 0, kADCnbChannels } ADCchannel;

typedef struct {
    uint16_t value;
    uint8_t updated;
} ADCresult;

void ADCinterruptTriggered(void);
void initialiseHALadc(void);
void runADCstateMachine(void);
uint8_t requestADCmeasurement(ADCchannel channel);
uint8_t getADCvalue(ADCchannel channel, ADCresult* value);
int32_t adcToInternalTemperature(uint16_t adc_raw);

#endif
