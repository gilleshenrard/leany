/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef INC_HARDWARE_SCREEN_ST7735S_REGISTERS_H_
#define INC_HARDWARE_SCREEN_ST7735S_REGISTERS_H_

//  Frame Rate Control (In normal mode/ Full colors) (register 0xB1) values
enum {
    ONELINEPERIOD_1    = 0x01U,  ///< 1 line period value of 1
    FRONTPORCH_DEFAULT = 0x2CU,  ///< Default front porch value after sleep out
    BACKPORCH_DEFAULT  = 0x2DU,  ///< Default back porch value after sleep out
};

//  Display Inversion Control (register 0xB4) values
enum {
    ALL_MODES_NO_INVERSION = 0x07U,  ///< Disable inversion in all screen modes
};

// Power Control 1 (register 0xC0) values
enum {
    AVDD_5V         = 0xA0U,  ///< AVDD voltage = 5V
    GVDD_4_6V       = 0x02U,  ///< GVDD voltage = 4.6V
    GVCL_NEG_4_6V   = 0x02U,  ///< GVCL voltage = -4.6V
    POWER_MODE_AUTO = 0x84U,  ///< Power mode auto
};

// Power control 2 (register 0xC1) values
enum {
    VGH25_2_4C = 0xC0U,  ///< VGH25 = 2.4C
    VGL_10     = 0x04U,  ///< VGL = -10
    VGH_3ADD   = 0x01U,  ///< VGH = 3*ADD
};

// Power control 3, 4 and 5 (registers 0xC2 to 0xC4) values
enum {
    OPAMP_HIGH_SMALL_CUR = 0x08U,  ///< OpAmp SAPA small current
    OPAMP_LOW_MEDLOW_CUR = 0x02U,  ///< OpAmp APA medium low current
    BOOST_LSB_MAX        = 0x00U,  ///< Maximum booster value for bits 0 to 7
    BOOST_LSB_BCLK_2     = 0x2AU,  ///< BCLK/2 booster value for bits 0 to 7
    BOOST_76_BCLK_2      = 0x2AU,  ///< BCLK/2 booster value for bits 0 to 7
    BOOST_MAX            = 0x00U,  ///< Maximum booster value for 2 bits
    BOOST_BCLK_2         = 0x02U,  ///< BCLK/2 booster value for 2 bits
    BOOST_LSB_IDLE_MODE  = 0xEEU,  ///< Boost LSB value for idle mode
};

//  VCOM Control 1 (register 0xC5) values
enum {
    VCOM_NEG_0_775V = 0x0EU,  ///< VCOM voltage = -0.775V
};

// Address control (register 0x36) values
enum {
    REFRESH_TOP_BOTTOM   = 0x00U,  ///< refresh top to bottom
    REFRESH_BOTTOM_TOP   = 0x10U,  ///< refresh bottom to top
    REFRESH_LEFT_RIGHT   = 0x00U,  ///< refresh left to right
    REFRESH_RIGHT_LEFT   = 0x04U,  ///< refresh right to left
    COLORS_ORDER_RGB     = 0x00U,  ///< RGB instead of BGR
    COLORS_ORDER_BGR     = 0x08U,  ///< BGR instead of RGB
    ORIENT_PORTRAIT      = 0x00U,  ///< MV, MX and MY values for portrait orientation
    ORIENT_PORTRAIT_180  = 0xC0U,  ///< MV, MX and MY values for portrait orientation (180°)
    ORIENT_LANDSCAPE     = 0x60U,  ///< MV, MX and MY values for landscape orientation
    ORIENT_LANDSCAPE_180 = 0xA0U,  ///< MV, MX and MY values for landscape orientation (180°)
};

//  Colour mode (register 0x3A) values
enum {
    COLOUR_16BITS = 0x05U,  ///< 16 bits colours
};

/**
 * @brief ST7735 System Function command List
 */
typedef enum {
    NOP       = 0x00U,  ///< No Operation
    SWRESET   = 0x01U,  ///< Software reset
    RDDID     = 0x04U,  ///< Read Display ID
    RDDST     = 0x09U,  ///< Read Display Status
    RDDPM     = 0x0AU,  ///< Read Display Power
    RDDMADCTL = 0x0BU,  ///< Read Display
    RDDCOLMOD = 0x0CU,  ///< Read Display Pixel
    RDDIM     = 0x0DU,  ///< Read Display Image
    RDDSM     = 0x0EU,  ///< Read Display Signal
    SLPIN     = 0x10U,  ///< Sleep in & booster off
    SLPOUT    = 0x11U,  ///< Sleep out & booster on
    PTLON     = 0x12U,  ///< Partial mode on
    NORON     = 0x13U,  ///< Partial off (Normal)
    INVOFF    = 0x20U,  ///< Display inversion off
    INVON     = 0x21U,  ///< Display inversion on
    GAMSET    = 0x26U,  ///< Gamma curve select
    DISPOFF   = 0x28U,  ///< Display off
    DISPON    = 0x29U,  ///< Display on
    CASET     = 0x2AU,  ///< Column address set
    RASET     = 0x2BU,  ///< Row address set
    RAMWR     = 0x2CU,  ///< Memory write
    RAMRD     = 0x2EU,  ///< Memory read
    PTLAR     = 0x30U,  ///< Partial start/end address set
    TEOFF     = 0x34U,  ///< Tearing effect line off
    TEON      = 0x35U,  ///< Tearing effect mode set & on
    MADCTL    = 0x36U,  ///< Memory data access control
    IDMOFF    = 0x38U,  ///< Idle mode off
    IDMON     = 0x39U,  ///< Idle mode on
    COLMOD    = 0x3AU,  ///< Interface pixel format
    FRMCTR1   = 0xB1U,  ///< Framerate control in normal mode (Full colors)
    FRMCTR2   = 0xB2U,  ///< Framerate control in Idle mode (8-colors)
    FRMCTR3   = 0xB3U,  ///< Framerate control in partial mode + Full colors
    INVCTR    = 0xB4U,  ///< Display inversion control
    DISSET5   = 0xB6U,  ///< Display function setting
    PWCTR1    = 0xC0U,  ///< Power control setting 1
    PWCTR2    = 0xC1U,  ///< Power control setting 2
    PWCTR3    = 0xC2U,  ///< In normal mode (Full colors)
    PWCTR4    = 0xC3U,  ///< In Idle mode (8-colors)
    PWCTR5    = 0xC4U,  ///< In partial mode + Full colors
    VMCTR1    = 0xC5U,  ///< VCOM control 1
    VMOFCTR   = 0xC7U,  ///< Set VCOM offset control
    WRID2     = 0xD1U,  ///< Set LCM version code
    WRID3     = 0xD2U,  ///< Customer Project code
    NVCTR1    = 0xD9U,  ///< EEPROM control status
    NVCTR2    = 0xDEU,  ///< EEPROM Read Command
    RDID1     = 0xDAU,  ///< Read ID1
    RDID2     = 0xDBU,  ///< Read ID2
    RDID3     = 0xDCU,  ///< Read ID3
    NVCTR3    = 0xDFU,  ///< EEPROM Write Command
    GAMCTRP1  = 0xE0U,  ///< Set Gamma adjustment (+ polarity)
    GAMCTRN1  = 0xE1U,  ///< Set Gamma adjustment (- polarity)
    EXTCTRL   = 0xF0U,  ///< Extension Command Control
    PWCTR6    = 0xFCU,  ///< In partial mode + Idle
    VCOM4L    = 0xFFU,  ///< Vcom 4 Level control
} ST7735register_e;

#endif
