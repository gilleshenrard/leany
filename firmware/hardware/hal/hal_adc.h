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

/**
 * ADC devices implemented
 */
typedef enum {
    kADC1 = 0,      ///< ADC1
    kADCnbDevices,  ///< Number of ADC devices
} ADCdevice;

/**
 * ADC channels used in the application
 */
typedef enum {
    kADCchannelTemperature = 0,  ///< MCU internal temperature
    kADCchannelBattery,          ///< Battery voltage
    kADCchannelVrefInt,          ///< MCU VrefInt
    kADC1nbChannels              ///< Number of ADC channels implemented
} ADC1channel;

void ADCinterruptTriggered(void);
void initialiseHALadc(void);
uint8_t requestADCmeasurement(ADCdevice device);
uint8_t getADCvalue(ADCdevice device, uint8_t channel, uint16_t* value);
void runADCstateMachine(void);

#endif
