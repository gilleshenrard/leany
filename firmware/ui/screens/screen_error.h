/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file screen_error.h
 * @author Gilles Henrard
 */
#ifndef UI_SCREENS_SCREEN_ERROR_H
#define UI_SCREENS_SCREEN_ERROR_H
#include "errorstack.h"

ErrorCode setupErrorScreen(const ErrorCode* error);
ErrorCode updateErrorAnimation(void);

#endif
