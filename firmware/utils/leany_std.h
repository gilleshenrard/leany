/*
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file leany_std.h
 * @brief Lightweight printf implementation for embedded systems
 * 
 * Provides leany_snprintf and leany_vsnprintf functions that pass strict linting.
 * 
 * Supported format specifiers:
 * - %d, %i: signed decimal integer
 * - %u: unsigned decimal integer
 * - %x: unsigned hexadecimal (lowercase)
 * - %X: unsigned hexadecimal (uppercase)
 * - %s: string
 * - %c: character
 * - %p: pointer (as 0xHEXADECIMAL)
 * - %%: literal percent sign
 * 
 * Supported flags:
 * - 0: zero padding
 * - -: left justify
 * - +: always show sign
 * - (space): space for positive numbers
 * - width: minimum field width (e.g., %5d)
 * - .precision: precision for strings (e.g., %.10s)
 * 
 * @note Does not support:
 * - Floating-point formats (%f, %e, %g)
 * - Length modifiers beyond default int/uint32_t
 * - Dynamic width/precision (*) 
 */

#ifndef HARDWARE_CUSTOM_PRINTF_H
#define HARDWARE_CUSTOM_PRINTF_H

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

enum {
    kFloatBufferSize = 20U,  ///< Number of bytes a float to string buffer can hold
    kHexaUint32Size = 8U,    ///< Number of bytes a hexadecimal 32-bits number can hold
};

int32_t leany_vsnprintf(char* buffer, size_t size, const char* format, va_list args);
int32_t leany_snprintf(char* buffer, size_t size, const char* format, ...);
void floatToString(float value, char out_buffer[], uint8_t buffer_size, uint8_t precision);
void intToString(uint32_t value, char out_buffer[], uint8_t buffer_size);
float stringToFloat(const char string[]);
uint32_t stringToInt(const char string[]);
uint32_t stringHexToInt(const char string[]);
uint8_t toLowerAscii(uint8_t character);
int8_t compareString(const char* first, size_t first_size, const char* second, size_t second_size);
size_t getStringLength(const char* string, size_t max_length);

/**
 * Check if a character is a number
 *
 * @param character Character to check
 * @retval 1 Character is a number
 * @retval 0 Character is not
 */
static inline uint8_t isnumber(char character) { return (((character >= '0') && (character <= '9'))); }

/**
 * Check if a character is a sign ('+' or '-')
 *
 * @param character Character to check
 * @retval 1 Character is a sign
 * @retval 0 Character is not
 */
static inline uint8_t issign(char character) { return ((character == '+') || (character == '-')); }

/**
 * Check if a character is an upper case alpha character
 *
 * @param character Character to check
 * @retval 1 The character is upper case
 * @retval 0 The character is not upper case
 */
static inline uint8_t isAlphaUppercase(char character) {
    return ((character >= (uint8_t)'A') && (character <= (uint8_t)'Z'));
}

/**
 * Check if a character is a lower case alpha character
 *
 * @param character Character to check
 * @retval 1 The character is lower case
 * @retval 0 The character is not lower case
 */
static inline uint8_t isAlphaLowercase(char character) {
    return ((character >= (uint8_t)'a') && (character <= (uint8_t)'z'));
}

#endif /* CUSTOM_PRINTF_H */
