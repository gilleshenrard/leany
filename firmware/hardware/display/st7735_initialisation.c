/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#include "st7735_initialisation.h"

#include <stddef.h>

#include "st7735_registers.inc"

/**
 * @brief Arguments sent when configuring the frame rate
 */
static const Register kFramerageControlArgs[3] = {
    kOneLinePeriod1,
    kFrontPorchDefault,
    kBackPorchDefault,
};

/**
 * @brief Arguments sent when configuring the frame rate in partial mode
 */
static const Register kFramerateControlPartialArgs[6] = {
    kOneLinePeriod1, kFrontPorchDefault, kBackPorchDefault, kOneLinePeriod1, kFrontPorchDefault, kBackPorchDefault,
};

/**
 * @brief Arguments sent when configuring the inversion
 */
static const Register kInversionControlArg = kAllModesNoInversion;

/**
 * @brief Arguments sent when configuring the power 1
 */
static const Register kPowerControl1Args[3] = {
    kAVDD_5v | kGVDD_4_6v,  // NOLINT (hicpp-signed-bitwise)
    kGVCL_NEG_4_6V,
    kPowerModeAuto,
};

/**
 * @brief Arguments sent when configuring the power 2
 */
static const Register powerControl2_arg = kVGH25_2_4C | kVGL_NEG_10 | kVGH_3ADD;  // NOLINT (hicpp-signed-bitwise)

/**
 * @brief Arguments sent when configuring the power 3
 */
static const Register kPowerControl3args[2] = {
    kBoostMax | kOpAmpSAPASmallCurrent | kOpAmpAPAMedLowCurrent,  // NOLINT (hicpp-signed-bitwise)
    kBoostMax,
};

/**
 * @brief Arguments sent when configuring the power 4
 */
static const Register kPowerControl4args[2] = {
    (kBoost_BCLK_DIV2 << 6U) | kOpAmpSAPASmallCurrent | kOpAmpAPAMedLowCurrent,  // NOLINT (hicpp-signed-bitwise)
    kBoostLSB_BCLK_DIV2,
};

/**
 * @brief Arguments sent when configuring the power 5
 */
static const Register kPowerControl5args[2] = {
    (kBoost_BCLK_DIV2 << 6U) | kOpAmpSAPASmallCurrent | kOpAmpAPAMedLowCurrent,  // NOLINT (hicpp-signed-bitwise)
    kBoostLSBidleMode,
};

/**
 * @brief Arguments sent when configuring the VCOM voltage
 */
static const Register kVmCtr1arg = kVCOM_NEG_0_775V;

const Register kOrientations[kNBorientations] = {
    // NOLINTBEGIN(misc-redundant-expression, hicpp-signed-bitwise)
    [kPortrait] =
        kRefreshTopToBottom | kRefreshLeftToRight | kOrientPortrait | kColourOrderRGB,  // NOLINT (hicpp-signed-bitwise)
    [kPortrait180] = kRefreshTopToBottom | kRefreshLeftToRight | kOrientPortrait180     // NOLINT (hicpp-signed-bitwise)
                     | kColourOrderRGB,                                                 // NOLINT (hicpp-signed-bitwise)
    [kLandscape] = kRefreshTopToBottom | kRefreshLeftToRight | kOrientLandscape |
                   kColourOrderRGB,                                                    // NOLINT (hicpp-signed-bitwise)
    [kLandscape180] = kRefreshTopToBottom | kRefreshLeftToRight | kOrientLandscape180  // NOLINT (hicpp-signed-bitwise)
                      | kColourOrderRGB,                                               // NOLINT (hicpp-signed-bitwise)
    // NOLINTEND(misc-redundant-expression, hicpp-signed-bitwise)
};

/**
 * @brief Arguments sent when configuring the colour mode
 */
static const Register kColorModeArg = kColour16bits;

/**
 * @brief Arguments sent when configuring the Gamma positive
 */
static const Register kGammaControlPositiveArgs[16] = {0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d,
                                                       0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10};

/**
 * @brief Arguments sent when configuring the Gamma negative
 */
static const Register kGammaControlNegativeArgs[16] = {0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
                                                       0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10};

/**
 * @brief Configuration commands list
 */
const ST7735command kST7735configurationScript[kST7735nbCommands] = {
    {kFRMCTR1, 3, kFramerageControlArgs},         //set frame rate in normal mode
    {kFRMCTR2, 3, kFramerageControlArgs},         //set frame rate in idle mode
    {kFRMCTR3, 3, kFramerateControlPartialArgs},  //set frame rate in partial mode
    {kINVCTR, 1, &kInversionControlArg},          //disable inversion
    {kPWCTR1, 3, kPowerControl1Args},             //set power control 1
    {kPWCTR2, 1, &powerControl2_arg},             //set power control 2
    {kPWCTR3, 2, kPowerControl3args},             //set power control 3 in normal mode
    {kPWCTR4, 2, kPowerControl4args},             //set power control 4 in idle mode
    {kPWCTR5, 2, kPowerControl5args},             //set power control 5 in partial mode
    {kVMCTR1, 1, &kVmCtr1arg},                    //set VCOM voltage
    {kINVOFF, 0, NULL},                           //no display inversion
    {kCOLMOD, 1, &kColorModeArg},                 //color mode
    {kGAMCTRP1, 16, kGammaControlPositiveArgs},   //Gamma adjustments (positive polarity)
    {kGAMCTRN1, 16, kGammaControlNegativeArgs},   //Gamma adjustments (negative polarity)
    {kNORON, 0, NULL},                            //normal mode ON
    {kDISPON, 0, NULL},                           //display ON
};
