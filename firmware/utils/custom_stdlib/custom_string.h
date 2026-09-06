/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 *
 * @file custom_string.h
 * @brief Lightweight standard functions implementation for embedded systems
 */

#ifndef UTILS_CUSTOM_STDLIB_CUSTOM_STRING_H
#define UTILS_CUSTOM_STDLIB_CUSTOM_STRING_H
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

// clang-format off
__attribute__((format(printf, 3, 4)))
int32_t custom_snprintf(char* buffer, size_t size, const char* format, ...);

__attribute__((format(printf, 3, 0)))
int32_t custom_vsnprintf(char* buffer, size_t size, const char* format, va_list* args);
// clang-format on

float custom_strtof(const char string[]);
uint32_t custom_strtoi(const char string[]);
uint32_t custom_strtoi_hexa(const char string[]);
char toLowerAscii(char character);
bool isnumber(char character);
bool issign(char character);
bool isAlphaUppercase(char character);
bool isAlphaLowercase(char character);
size_t getStringLength(const char* string, size_t max_length);
int8_t compareString(const char* first, size_t first_size, const char* second, size_t second_size);

#endif
