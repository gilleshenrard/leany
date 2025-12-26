/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HARDWARE_GPIO_LED_H
#define HARDWARE_GPIO_LED_H
#include <stdint.h>

/**
 * Hex value of each colour channel
 */
typedef struct {
    uint8_t blue;    ///< Blue HEX value
    uint8_t green;   ///< Red HEX value
    uint8_t red;     ///< Green HEX value
    uint8_t unused;  ///< Unused leading byte
} Channels;

/**
 * Structure representing a HEX RGB colour value
 */
typedef union {
    Channels channels;   ///< Independant colour channels
    uint32_t hex_value;  ///< Whole colour HEX value
} Colour;

/**
 * Enumeration of the LED effects
 */
typedef enum {
    kOFF = 0,   ///< OFF
    kSOLID,     ///< Solidly ON
    kBLINKING,  ///< Blinking
} EffectState;

static const Colour kWhite = {.hex_value = 0xFFFFFFUL};        ///< 0xFFFFFF : white LED hex colour value
static const Colour kWhiteDimmed = {.hex_value = 0x333333UL};  ///< 0x333333 : dimmed white LED hex colour value
static const Colour kRed = {.hex_value = 0xFF0000UL};          ///< 0xFF0000 : red LED hex colour value
static const Colour kGreen = {.hex_value = 0x00FF00UL};        ///< 0x00FF00 : green LED hex colour value
static const Colour kBlue = {.hex_value = 0x0000FFUL};         ///< 0x0000FF : blue LED hex colour value
static const Colour kOrange = {.hex_value = 0xFFA500UL};       ///< 0xFFA500 : orange LED hex colour value
static const Colour kYellow = {.hex_value = 0xFFFF00UL};       ///< 0xFFFF00 : yellow LED hex colour value
static const Colour kBlack = {.hex_value = 0x000000UL};        ///< 0x000000 : LED off

static const uint16_t kSlowblinkPeriod_ms = 2000U;

void initialiseLED(void);
void runLEDstateMachine(void);
void LEDsetEffect(EffectState new_effect, uint16_t period_ms);
void LEDsetColour(const Colour* colour);
void LEDsetColourHex(uint32_t hexa_code);

#endif
