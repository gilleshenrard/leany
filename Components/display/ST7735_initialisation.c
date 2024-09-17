#include "ST7735_initialisation.h"
#include <stddef.h>
#include "ST7735_registers.h"


/**
 * @brief Arguments sent when configuring the frame rate
 */
static const registerValue_t framerateControl_args[3] = {
    ONELINEPERIOD_1,
    FRONTPORCH_DEFAULT,
    BACKPORCH_DEFAULT,
};

/**
 * @brief Arguments sent when configuring the frame rate in partial mode
 */
static const registerValue_t framerateControlPartial_args[6] = {
    ONELINEPERIOD_1, FRONTPORCH_DEFAULT, BACKPORCH_DEFAULT, ONELINEPERIOD_1, FRONTPORCH_DEFAULT, BACKPORCH_DEFAULT,
};

/**
 * @brief Arguments sent when configuring the inversion
 */
static const registerValue_t inversionControl_arg = ALL_MODES_NO_INVERSION;

/**
 * @brief Arguments sent when configuring the power 1
 */
static const registerValue_t powerControl1_args[3] = {
    AVDD_5V | GVDD_4_6V,
    GVCL_NEG_4_6V,
    POWER_MODE_AUTO,
};

/**
 * @brief Arguments sent when configuring the power 2
 */
static const registerValue_t powerControl2_arg = VGH25_2_4C | VGL_10 | VGH_3ADD;

/**
 * @brief Arguments sent when configuring the power 3
 */
static const registerValue_t powerControl3_args[2] = {
    BOOST_MAX | OPAMP_HIGH_SMALL_CUR | OPAMP_LOW_MEDLOW_CUR,
    BOOST_MAX,
};

/**
 * @brief Arguments sent when configuring the power 4
 */
static const registerValue_t powerControl4_args[2] = {
    (BOOST_BCLK_2 << 6U) | OPAMP_HIGH_SMALL_CUR | OPAMP_LOW_MEDLOW_CUR,
    BOOST_LSB_BCLK_2,
};

/**
 * @brief Arguments sent when configuring the power 5
 */
static const registerValue_t powerControl5_args[2] = {
    (BOOST_BCLK_2 << 6U) | OPAMP_HIGH_SMALL_CUR | OPAMP_LOW_MEDLOW_CUR,
    BOOST_LSB_IDLE_MODE,
};

/**
 * @brief Arguments sent when configuring the VCOM voltage
 */
static const registerValue_t vmCtr1_arg = VCOM_NEG_0_775V;

// NOLINTBEGIN(misc-redundant-expression)
const registerValue_t orientations[NB_ORIENTATION] = {
    [PORTRAIT]      = REFRESH_TOP_BOTTOM | REFRESH_LEFT_RIGHT | ORIENT_PORTRAIT | COLORS_ORDER_RGB,
    [PORTRAIT_180]  = REFRESH_TOP_BOTTOM | REFRESH_LEFT_RIGHT | ORIENT_PORTRAIT_180 | COLORS_ORDER_RGB,
    [LANDSCAPE]     = REFRESH_TOP_BOTTOM | REFRESH_LEFT_RIGHT | ORIENT_LANDSCAPE | COLORS_ORDER_RGB,
    [LANDSCAPE_180] = REFRESH_TOP_BOTTOM | REFRESH_LEFT_RIGHT | ORIENT_LANDSCAPE_180 | COLORS_ORDER_RGB,
};
// NOLINTEND(misc-redundant-expression)

/**
 * @brief Arguments sent when configuring the colour mode
 */
static const registerValue_t colorMode_arg = COLOUR_16BITS;

/**
 * @brief Arguments sent when configuring the Gamma positive
 */
static const registerValue_t gammaControlPositive_args[16] = {0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d,
                                                              0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10};

/**
 * @brief Arguments sent when configuring the Gamma negative
 */
static const registerValue_t gammaControlNegative_args[16] = {0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D,
                                                              0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10};

/**
 * @brief Configuration commands list
 */
const st7735_command_t st7735configurationScript[ST7735_NB_COMMANDS] = {
    { FRMCTR1,  3,        framerateControl_args}, //set frame rate in normal mode
    { FRMCTR2,  3,        framerateControl_args}, //set frame rate in idle mode
    { FRMCTR3,  3, framerateControlPartial_args}, //set frame rate in partial mode
    {  INVCTR,  1,        &inversionControl_arg}, //disable inversion
    {  PWCTR1,  3,           powerControl1_args}, //set power control 1
    {  PWCTR2,  1,           &powerControl2_arg}, //set power control 2
    {  PWCTR3,  2,           powerControl3_args}, //set power control 3 in normal mode
    {  PWCTR4,  2,           powerControl4_args}, //set power control 4 in idle mode
    {  PWCTR5,  2,           powerControl5_args}, //set power control 5 in partial mode
    {  VMCTR1,  1,                  &vmCtr1_arg}, //set VCOM voltage
    {  INVOFF,  0,                         NULL}, //no display inversion
    {  COLMOD,  1,               &colorMode_arg}, //color mode
    {GAMCTRP1, 16,    gammaControlPositive_args}, //Gamma adjustments (positive polarity)
    {GAMCTRN1, 16,    gammaControlNegative_args}, //Gamma adjustments (negative polarity)
    {   NORON,  0,                         NULL}, //normal mode ON
    {  DISPON,  0,                         NULL}, //display ON
};
