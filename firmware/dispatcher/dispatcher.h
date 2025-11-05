/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef CONTROLLER_DISPATCHER_H
#define CONTROLLER_DISPATCHER_H
#include <stdint.h>

#include "errorstack.h"

enum {
    kMessageStructAlignment = 8U,  ///< Optimised memory alignment for the Message structure
};

/**
 * Enumeration of all the possible application messages
 */
typedef enum {
    kMessageXValue = 0,  ///< Message : X value update
    kMessageYValue,      ///< Message : Y value update
    kMessageZero,        ///< Message : Zero down measurements
    kMessageCancelZero,  ///< Message : Cancel measurements zeroing
    kNbMessages,         ///< Number of messages types
} MessageID;

/**
 * Application message descriptor
 */
typedef struct {
    MessageID type;  ///< Message type
    int16_t value;   ///< Value of the message
} __attribute__((aligned(kMessageStructAlignment))) Message;

ErrorCode createMessageDispatchertask(void);
uint8_t getUImessage(Message* message_to_get);

#endif
