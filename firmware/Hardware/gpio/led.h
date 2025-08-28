/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef HARDWARE_GPIO_LED_H
#define HARDWARE_GPIO_LED_H
#include <stdint.h>

enum {
    kColourAlignment = 4U,  ///< Memory alignment of the colour internal structure
};

/**
 * Structure representing a HEX RGB colour value
 */
typedef union {
    struct {
        uint8_t blue;   ///< Blue HEX value
        uint8_t green;  ///< Red HEX value
        uint8_t red;    ///< Green HEX value
    } __attribute__((aligned(kColourAlignment)));

    uint32_t hex_value;  ///< Whole colour HEX value
} Colour;

static const Colour kWhite = {.hex_value = 0xFFFFFFUL};   ///< #FFFFFF : white LED hex colour value
static const Colour kRed = {.hex_value = 0xFF0000UL};     ///< #FF0000 : red LED hex colour value
static const Colour kGreen = {.hex_value = 0x00FF00UL};   ///< #00FF00 : green LED hex colour value
static const Colour kBlue = {.hex_value = 0x0000FFUL};    ///< #0000FF : blue LED hex colour value
static const Colour kOrange = {.hex_value = 0xFFA500UL};  ///< #FFA500 : orange LED hex colour value
static const Colour kYellow = {.hex_value = 0xFFFF00UL};  ///< #FFFF00 : yellow LED hex colour value
static const Colour kBlack = {.hex_value = 0x000000UL};   ///< #000000 : LED off

void initialiseLED(void);
void runLEDstateMachine(void);
void LEDsolid(const Colour* colour);
void LEDoff(void);
void LEDblink(const Colour* colour, uint16_t new_blink_period_ms);

#endif
