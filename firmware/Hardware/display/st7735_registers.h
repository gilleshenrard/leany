/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef INC_HARDWARE_SCREEN_ST7735S_REGISTERS_H
#define INC_HARDWARE_SCREEN_ST7735S_REGISTERS_H

//  Frame Rate Control (In normal mode/ Full colors) (register 0xB1) values
enum {
    kOneLinePeriod1 = 0x01U,     ///< 1 line period value of 1
    kFrontPorchDefault = 0x2CU,  ///< Default front porch value after sleep out
    kBackPorchDefault = 0x2DU,   ///< Default back porch value after sleep out
};

//  Display Inversion Control (register 0xB4) values
enum {
    kAllModesNoInversion = 0x07U,  ///< Disable inversion in all screen modes
};

// Power Control 1 (register 0xC0) values
enum {
    kAVDD_5v = 0xA0U,        ///< AVDD voltage = 5V
    kGVDD_4_6v = 0x02U,      ///< GVDD voltage = 4.6V
    kGVCL_NEG_4_6V = 0x02U,  ///< GVCL voltage = -4.6V
    kPowerModeAuto = 0x84U,  ///< Power mode auto
};

// Power control 2 (register 0xC1) values
enum {
    kVGH25_2_4C = 0xC0U,  ///< VGH25 = 2.4C
    kVGL_NEG_10 = 0x04U,  ///< VGL = -10
    kVGH_3ADD = 0x01U,    ///< VGH = 3*ADD
};

// Power control 3, 4 and 5 (registers 0xC2 to 0xC4) values
enum {
    kOpAmpSAPASmallCurrent = 0x08U,  ///< OpAmp SAPA small current
    kOpAmpAPAMedLowCurrent = 0x02U,  ///< OpAmp APA medium low current
    kBoostLSBmax = 0x00U,            ///< Maximum booster value for bits 0 to 7
    kBoostLSB_BCLK_DIV2 = 0x2AU,     ///< BCLK/2 booster value for bits 0 to 7
    kBoost_76_BCLK_DIV2 = 0x2AU,     ///< BCLK/2 booster value for bits 0 to 7
    kBoostMax = 0x00U,               ///< Maximum booster value for 2 bits
    kBoost_BCLK_DIV2 = 0x02U,        ///< BCLK/2 booster value for 2 bits
    kBoostLSBidleMode = 0xEEU,       ///< Boost LSB value for idle mode
};

//  VCOM Control 1 (register 0xC5) values
enum {
    kVCOM_NEG_0_775V = 0x0EU,  ///< VCOM voltage = -0.775V
};

// Address control (register 0x36) values
enum {
    kRefreshTopToBottom = 0x00U,  ///< refresh top to bottom
    kRefreshBottomToTop = 0x10U,  ///< refresh bottom to top
    kRefreshLeftToRight = 0x00U,  ///< refresh left to right
    kRefreshRightToLeft = 0x04U,  ///< refresh right to left
    kColourOrderRGB = 0x00U,      ///< RGB instead of BGR
    kColourOrderBGR = 0x08U,      ///< BGR instead of RGB
    kOrientPortrait = 0x00U,      ///< MV, MX and MY values for portrait orientation
    kOrientPortrait180 = 0xC0U,   ///< MV, MX and MY values for portrait orientation (rotated 180°)
    kOrientLandscape = 0x60U,     ///< MV, MX and MY values for landscape orientation
    kOrientLandscape180 = 0xA0U,  ///< MV, MX and MY values for landscape orientation (rotated 180°)
};

//  Colour mode (register 0x3A) values
enum {
    kColour16bits = 0x05U,  ///< 16 bits colours
};

/**
 * @brief ST7735 System Function command List
 */
typedef enum {
    kNOP = 0x00U,        ///< No Operation
    kSWRESET = 0x01U,    ///< Software reset
    kRDDID = 0x04U,      ///< Read Display ID
    kRDDST = 0x09U,      ///< Read Display Status
    kRDDPM = 0x0AU,      ///< Read Display Power
    kRDDMADCTL = 0x0BU,  ///< Read Display
    kRDDCOLMOD = 0x0CU,  ///< Read Display Pixel
    kRDDIM = 0x0DU,      ///< Read Display Image
    kRDDSM = 0x0EU,      ///< Read Display Signal
    kSLPIN = 0x10U,      ///< Sleep in & booster off
    kSLPOUT = 0x11U,     ///< Sleep out & booster on
    kPTLON = 0x12U,      ///< Partial mode on
    kNORON = 0x13U,      ///< Partial off (Normal)
    kINVOFF = 0x20U,     ///< Display inversion off
    kINVON = 0x21U,      ///< Display inversion on
    kGAMSET = 0x26U,     ///< Gamma curve select
    kDISPOFF = 0x28U,    ///< Display off
    kDISPON = 0x29U,     ///< Display on
    kCASET = 0x2AU,      ///< Column address set
    kRASET = 0x2BU,      ///< Row address set
    kRAMWR = 0x2CU,      ///< Memory write
    kRAMRD = 0x2EU,      ///< Memory read
    kPTLAR = 0x30U,      ///< Partial start/end address set
    kTEOFF = 0x34U,      ///< Tearing effect line off
    kTEON = 0x35U,       ///< Tearing effect mode set & on
    kMADCTL = 0x36U,     ///< Memory data access control
    kIDMOFF = 0x38U,     ///< Idle mode off
    kIDMON = 0x39U,      ///< Idle mode on
    kCOLMOD = 0x3AU,     ///< Interface pixel format
    kFRMCTR1 = 0xB1U,    ///< Framerate control in normal mode (Full colors)
    kFRMCTR2 = 0xB2U,    ///< Framerate control in Idle mode (8-colors)
    kFRMCTR3 = 0xB3U,    ///< Framerate control in partial mode + Full colors
    kINVCTR = 0xB4U,     ///< Display inversion control
    kDISSET5 = 0xB6U,    ///< Display function setting
    kPWCTR1 = 0xC0U,     ///< Power control setting 1
    kPWCTR2 = 0xC1U,     ///< Power control setting 2
    kPWCTR3 = 0xC2U,     ///< In normal mode (Full colors)
    kPWCTR4 = 0xC3U,     ///< In Idle mode (8-colors)
    kPWCTR5 = 0xC4U,     ///< In partial mode + Full colors
    kVMCTR1 = 0xC5U,     ///< VCOM control 1
    kVMOFCTR = 0xC7U,    ///< Set VCOM offset control
    kWRID2 = 0xD1U,      ///< Set LCM version code
    kWRID3 = 0xD2U,      ///< Customer Project code
    kNVCTR1 = 0xD9U,     ///< EEPROM control status
    kNVCTR2 = 0xDEU,     ///< EEPROM Read Command
    kRDID1 = 0xDAU,      ///< Read ID1
    kRDID2 = 0xDBU,      ///< Read ID2
    kRDID3 = 0xDCU,      ///< Read ID3
    kNVCTR3 = 0xDFU,     ///< EEPROM Write Command
    kGAMCTRP1 = 0xE0U,   ///< Set Gamma adjustment (+ polarity)
    kGAMCTRN1 = 0xE1U,   ///< Set Gamma adjustment (- polarity)
    kEXTCTRL = 0xF0U,    ///< Extension Command Control
    kPWCTR6 = 0xFCU,     ///< In partial mode + Idle
    kVCOM4L = 0xFFU,     ///< Vcom 4 Level control
} ST7735register;

#endif
