/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#include "scpi_commands.h"

#include "generic_command.inc"

static const Node kRootNode;
const Node* getRootNode(void) { return &kRootNode; }

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

static const Node kLogCommands[] = {
    {.scpi =
         {.code = kCmdLogLevel, .short_name = "LEV", .long_name = "LEVEL", .mode = kRW, .param_type = kParamInteger}},
};

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

static const Node kIMUCommands[] = {
    {.scpi = {.code = kCmdKI, .short_name = "KI", .long_name = "KI", .mode = kRW, .param_type = kParamFloat}},
    {.scpi = {.code = kCmdKP, .short_name = "KP", .long_name = "KP", .mode = kRW, .param_type = kParamFloat}},
    {.scpi = {.code = kCmdAlignmentEnable,
              .short_name = "ENALI",
              .long_name = "ENABLEALIGNMENT",
              .mode = kRW,
              .param_type = kParamInteger}},
};

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

static const Node kRootCommands[] = {
    {.scpi = {.short_name = "IMU", .long_name = "IMU", .mode = kNA},
     .children = kIMUCommands,
     .nb_children = (sizeof(kIMUCommands) / sizeof(Node))},
    {.scpi = {.short_name = "LOG", .long_name = "LOG", .mode = kNA},
     .children = kLogCommands,
     .nb_children = (sizeof(kLogCommands) / sizeof(Node))},

    /*
    //mandatory SCPI commands
    {.scpi = {.short_name = "CLS", .long_name = "CLS", .mode = kWO}},
    {.scpi = {.short_name = "ESE", .long_name = "ESE", .mode = kRW}},
    {.scpi = {.short_name = "ESR", .long_name = "ESR", .mode = kRO}},
    {.scpi = {.short_name = "IDN", .long_name = "IDN", .mode = kRO}},
    {.scpi = {.short_name = "OPC", .long_name = "OPC", .mode = kRW}},
    {.scpi = {.short_name = "RST", .long_name = "RST", .mode = kWO}},
    {.scpi = {.short_name = "SRE", .long_name = "SRE", .mode = kRW}},
    {.scpi = {.short_name = "STB", .long_name = "STB", .mode = kRO}},
    {.scpi = {.short_name = "TST", .long_name = "TST", .mode = kRO}},
    {.scpi = {.short_name = "WAI", .long_name = "WAI", .mode = kWO}},
    */
};

static const Node kRootNode = {
    .children = kRootCommands,
    .nb_children = (sizeof(kRootCommands) / sizeof(Node)),
};
