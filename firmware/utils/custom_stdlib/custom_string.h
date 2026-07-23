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

int32_t custom_vsnprintf(char* buffer, size_t size, const char* format, va_list args);
int32_t custom_snprintf(char* buffer, size_t size, const char* format, ...);
bool isnumber(char character);
bool issign(char character);
bool isAlphaUppercase(char character);
bool isAlphaLowercase(char character);

#endif
