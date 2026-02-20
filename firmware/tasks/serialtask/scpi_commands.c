/**
 * SPDX-FileCopyrightText: 2026 Gilles Henrard <contact@gilleshenrard.com>
 * SPDX-License-Identifier: MIT
 * 
 * @file scpi_commands.c
 * @brief Implement SCPI commands
 * @author Gilles Henrard
 */
#include "scpi_commands.h"

#include <stddef.h>

#include "generic_command.inc"

static const Node kRootNode;  ///< Root node of the SCPI commands tree

/**
 * Get the root node of the SCPI commands tree
 * 
 * @return Tree root node 
 */
const Node* getRootNode(void) { return &kRootNode; }

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Log commands descriptors
 */
static const Node kLogCommands[] = {
    {.scpi =
         {.code = kCmdLogLevel, .short_name = "LEV", .long_name = "LEVel", .mode = kRW, .param_type = kParamInteger}},
};

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * IMU commands descriptors
 */
static const Node kIMUCommands[] = {
    {.scpi = {.code = kCmdKI, .short_name = "KI", .long_name = "KI", .mode = kRW, .param_type = kParamFloat}},
    {.scpi = {.code = kCmdKP, .short_name = "KP", .long_name = "KP", .mode = kRW, .param_type = kParamFloat}},
    {.scpi = {.code = kCmdToggleHold,
              .short_name = "TOGHOL",
              .long_name = "TOGgleHOLd",
              .mode = kRW,
              .param_type = kParamInteger}},
    {.scpi = {.code = kCmdToggleZero,
              .short_name = "TOGZERO",
              .long_name = "TOGgleZERO",
              .mode = kRW,
              .param_type = kParamInteger}},
    {.scpi = {.code = kCmdAlignmentEnable,
              .short_name = "ENALI",
              .long_name = "ENableALIgnment",
              .mode = kRW,
              .param_type = kParamInteger}},
};

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Display commands descriptors
 */
static const Node kDisplayCommands[] = {
    {.scpi = {.code = kCmdOrientation,
              .short_name = "ORI",
              .long_name = "ORIentation",
              .mode = kRW,
              .param_type = kParamInteger}},
    {.scpi = {.code = kCmdToggleScreen,
              .short_name = "TOGSCR",
              .long_name = "TOGgleSCReen",
              .mode = kWO,
              .param_type = kParamInteger}},
};

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Battery commands descriptors
 */
static const Node kBatteryCommands[] = {
    {.scpi = {.code = kCmdBatteryPercent,
              .short_name = "PER",
              .long_name = "PERcentage",
              .mode = kRW,
              .param_type = kParamInteger}},
    {.scpi = {.code = kCmdBatteryCharge,
              .short_name = "CHA",
              .long_name = "CHArge",
              .mode = kRW,
              .param_type = kParamInteger}},
};

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * LED commands descriptors
 */
static const Node kLedCommands[] = {
    {.scpi =
         {.code = kCmdLedColour, .short_name = "COL", .long_name = "COLour", .mode = kWO, .param_type = kParamHexa}},
    {.scpi =
         {.code = kCmdLedEffect, .short_name = "EFF", .long_name = "EFFect", .mode = kWO, .param_type = kParamInteger}},
};

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Error commands descriptors
 */
static const Node kErrorCommands[] = {
    {.scpi = {.code = kCmdErrorCode, .short_name = "HEX", .long_name = "HEX", .mode = kWO, .param_type = kParamHexa}},
};

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Root commands descriptors
 */
static const Node kRootCommands[] = {
    {.scpi = {.short_name = "IMU", .long_name = "IMU", .mode = kNA},
     .children = kIMUCommands,
     .nb_children = (sizeof(kIMUCommands) / sizeof(Node))},
    {.scpi = {.short_name = "LOG", .long_name = "LOG", .mode = kNA},
     .children = kLogCommands,
     .nb_children = (sizeof(kLogCommands) / sizeof(Node))},
    {.scpi = {.short_name = "DIS", .long_name = "DISplay", .mode = kNA},
     .children = kDisplayCommands,
     .nb_children = (sizeof(kDisplayCommands) / sizeof(Node))},
    {.scpi = {.short_name = "BAT", .long_name = "BATtery", .mode = kNA},
     .children = kBatteryCommands,
     .nb_children = (sizeof(kBatteryCommands) / sizeof(Node))},
    {.scpi = {.short_name = "LED", .long_name = "LED", .mode = kNA},
     .children = kLedCommands,
     .nb_children = (sizeof(kLedCommands) / sizeof(Node))},
    {.scpi = {.short_name = "ERR", .long_name = "ERRor", .mode = kNA},
     .children = kErrorCommands,
     .nb_children = (sizeof(kErrorCommands) / sizeof(Node))},
    {.scpi = {.code = kCmdHelp, .short_name = "HELP", .long_name = "HELP", .mode = kRO},
     .children = NULL,
     .nb_children = 0},

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

/**
 * Tree root node
 */
static const Node kRootNode = {
    .children = kRootCommands,
    .nb_children = (sizeof(kRootCommands) / sizeof(Node)),
};
