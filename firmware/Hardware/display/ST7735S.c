/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file ST7735S.c
 * @brief Implement the functioning of the ST7735S TFT screen via SPI and DMA
 * @author Gilles Henrard
 * @date 12/06/2025
 *
 * @note Datasheet : https://cdn-shop.adafruit.com/datasheets/ST7735R_V0.2.pdf
 */
#include "ST7735S.h"
#include <stddef.h>
#include <stdint.h>
#include "ST7735_initialisation.h"
#include "ST7735_registers.h"
#include "errorstack.h"
#include "halspi.h"
#include "main.h"
#include "projdefs.h"
#include "stm32f103xb.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"

/**
 * @brief Enumeration of the function IDs of the SSD1306
 */
typedef enum {
    SET_WINDOW = 0,  ///< setWindow()
    SEND_DATA,       ///< sendScreenData()
    PRT_CHAR,        ///< printCharacter()
    PRT_MEAS,        ///< printMeasurements()
    ORIENT,          ///< st7735sSetOrientation()
    STARTUP,         ///< restartScreen()
    CONFIG,          ///< writeConfiguration()
    SET_CONFIG,      ///< configureST7735S()
} SSD1306functionCodes_e;

/**
 * Structure describing the backporch offsets of the display
 */
typedef struct {
    uint8_t xOffset_px;  ///< X backporch offset in [pixels]
    uint8_t yOffset_px;  ///< Y backporch offset in [pixels]
} __attribute((aligned(2))) backporch_t;

//state machine
static errorCode_u restartScreen(void);
static errorCode_u writeConfiguration(void);

//State variables
static errorCode_u       result;                               ///< Buffer used to store function return codes
static orientation_e     currentOrientation = NB_ORIENTATION;  ///< Current display orientation
static const backporch_t offsets[]          =                  ///< default backporch offsets
    {
        {1, 2},
};
static dma_t dmaDescriptor =  ///< Descriptor of the DMA channel used to send data to the display
    {
        .dma     = DMA1,
        .Channel = LL_DMA_CHANNEL_5,
        .spi     = {.handle                = SPI2,
                    .CSport                = ST7735S_CS_GPIO_Port,
                    .pin                   = ST7735S_CS_Pin,
                    .dataCommandPort       = ST7735S_DC_GPIO_Port,
                    .dataCommandPin        = ST7735S_DC_Pin,
                    .highestRegisterNumber = VCOM4L}
};

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Send the coordinates of the window in which the following data should be displayed on the screen
 *
 * @param Xstart    X left-most coordinate of the window in [pixels]
 * @param Ystart    X top-most coordinate of the window in [pixels]
 * @param width     Width of the window in [pixels]
 * @param height    Height of the window in [pixels]
 * @retval 0 Success
 * @retval 1 Error while sending the X coordinates of the window
 * @retval 2 Error while sending the Y coordinates of the window
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
errorCode_u setWindow(uint8_t Xstart, uint8_t Ystart, uint8_t width, uint8_t height) {
    //set the data window columns count
    const uint8_t columns[4] = {0, Xstart + offsets[0].xOffset_px, 0,
                                (uint8_t)(Xstart + offsets[0].xOffset_px + width - (uint8_t)1U)};
    result                   = writeRegisters(&dmaDescriptor.spi, CASET, columns, 4);
    if(isError(result)) {
        return pushErrorCode(result, SET_WINDOW, 1);
    }

    //set the data window rows count
    const uint8_t rows[4] = {0, Ystart + offsets[0].yOffset_px, 0,
                             (uint8_t)(Ystart + offsets[0].yOffset_px + height - (uint8_t)1U)};
    result                = writeRegisters(&dmaDescriptor.spi, RASET, rows, 4);
    if(isError(result)) {
        return pushErrorCode(result, SET_WINDOW, 2);
    }

    return ERR_SUCCESS;
}

/**
 * @brief Set the display orientation
 * 
 * @param orientation New display orientation
 * @return Success
 * @retval 1 Invalid orientation code provided
 * @retval 2 Error while sending the command to the screen
 */
errorCode_u st7735sSetOrientation(orientation_e orientation) {
    //if orientation does not change, exit
    if(currentOrientation == orientation) {
        return (ERR_SUCCESS);
    }

    //if incorrect orientation, error
    if(orientation >= NB_ORIENTATION) {
        return createErrorCode(ORIENT, 1, ERR_CRITICAL);
    }

    //send the command to the display

    result = writeRegisters(&dmaDescriptor.spi, MADCTL, &orientations[orientation], 1);
    if(isError(result)) {
        return pushErrorCode(result, ORIENT, 2);
    }

    //update the current orientation
    currentOrientation = orientation;
    return (ERR_SUCCESS);
}

/**
 * @brief Turn the display TFT backlight ON
 */
void turnBacklightON(void) {
    LL_GPIO_ResetOutputPin(ST7735S_BL_GPIO_Port, ST7735S_BL_Pin);
}

/**
 * Attach a FreeRTOS UI task to the screen
 * @details This is used because the task should sleep on DMA send and wake up at DMA done
 *
 * @param handle FreeRTOS task handle
 */
void attachUItask(TaskHandle_t handle) {
    dmaDescriptor.task = handle;
}

/**
 * Send the configuration registers and value to the ST7735S
 *
 * @retval 0 Success
 * @retval 1 Error while restarting the display
 * @retval 2 Error while writing the configuration
 */
errorCode_u configureST7735S(void) {
    //make sure to disable ST7735S SPI communication
    LL_SPI_Enable(dmaDescriptor.spi.handle);

    result = restartScreen();
    if(isError(result)) {
        return pushErrorCode(result, SET_CONFIG, 1);
    }

    result = writeConfiguration();
    if(isError(result)) {
        return pushErrorCode(result, SET_CONFIG, 2);
    }

    return (ERR_SUCCESS);
}

/**
 * Restart the screen module
 * 
 * @return Success
 * @retval 1 Error while transmitting the software reset command
 * @retval 2 Error while transmitting the sleep out command
 */
static errorCode_u restartScreen(void) {
    static const uint8_t RESET_DELAY_MS    = 150U;  ///< Number of milliseconds to wait after reset
    static const uint8_t SLEEPOUT_DELAY_MS = 255U;  ///< Number of milliseconds to wait sleep out

    //send the reset command and, if error, exit
    result = writeRegisters(&dmaDescriptor.spi, SWRESET, NULL, 0);
    if(isError(result)) {
        return pushErrorCode(result, STARTUP, 1);
    }

    //wait for a while after software reset
    vTaskDelay(pdMS_TO_TICKS(RESET_DELAY_MS));

    //send the reset command and, if error, exit
    result = writeRegisters(&dmaDescriptor.spi, SLPOUT, NULL, 0);
    if(isError(result)) {
        return pushErrorCode(result, STARTUP, 2);
    }

    //wait for a while after sleepout
    vTaskDelay(pdMS_TO_TICKS(SLEEPOUT_DELAY_MS));

    //save the current systick and get to waiting state
    return (ERR_SUCCESS);
}

/**
 * Configure the screen properly
 * 
 * @return Success
 * @retval 1 Error while sending a command
 * @retval 2 Error while setting the screen orientation
 */
static errorCode_u writeConfiguration(void) {
    //execute all configuration commands

    for(uint8_t command = 0; command < (uint8_t)ST7735_NB_COMMANDS; command++) {
        result = writeRegisters(&dmaDescriptor.spi, (spiregister_t)st7735configurationScript[command].registerNumber,
                                st7735configurationScript[command].parameters,
                                st7735configurationScript[command].nbParameters);
        if(isError(result)) {
            return pushErrorCode(result, CONFIG, 1);
        }
    }

    //set screen orientation
    result = st7735sSetOrientation(LANDSCAPE);
    if(isError(result)) {
        return pushErrorCode(result, CONFIG, 2);
    }

    return (ERR_SUCCESS);
}

/**
 * Send a data chunk to the display
 *
 * @param data          Data to send
 * @param nbBytes       Number of bytes to send
 * @param maxBytes      Maximum number of bytes allowed (usually size of a display buffer)
 * @param screenArea    Display window which will contain the data on the display
 * @retval 0 Success
 * @retval 1 Error while setting the window
 * @retval 2 Error while sending RAM write command
 * @retval 3 Error while sending the data via DMA
 */
errorCode_u sendScreenData(const uint16_t data[], size_t nbBytes, size_t maxBytes, const area_t* screenArea) {
    result = setWindow(screenArea->x0, screenArea->y0, getAreaWidth(screenArea), getAreaHeight(screenArea));
    if(isError(result)) {
        return pushErrorCode(result, SEND_DATA, 1);
    }

    result = writeRegistersAndContinue(&dmaDescriptor.spi, RAMWR, NULL, 0);
    if(isError(result)) {
        return pushErrorCode(result, SEND_DATA, 2);
    }

    result = sendDMA(&dmaDescriptor, (uint8_t*)data, nbBytes, maxBytes);
    if(isError(result)) {
        return pushErrorCode(result, SEND_DATA, 3);
    }

    return ERR_SUCCESS;
}
