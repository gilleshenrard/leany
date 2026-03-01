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

enum {
    kAdcVref_mV = 3300,  ///< ADC reference voltage: 3.3V (VDDA voltage)
};

/**
 * ADC channels used in the application
 */
typedef enum {
    kADCchannelTemperature = 0,  ///< MCU internal temperature
    kADCchannelBattery,          ///< Battery voltage
    kADCnbChannels               ///< Number of ADC channels implemented
} ADCchannel;

/**
 * ADC read result
 */
typedef struct {
    uint16_t value;   ///< Latest ADC value
    uint8_t updated;  ///< Flag indicating whether the value has been updated
} ADCresult;

void ADCinterruptTriggered(void);
void initialiseHALadc(void);
void runADCstateMachine(void);
uint8_t requestADCmeasurement(ADCchannel channel);
uint8_t getADCvalue(ADCchannel channel, ADCresult* value);

#endif
