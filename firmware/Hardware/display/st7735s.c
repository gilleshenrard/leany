/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file st7735s.c
 * @brief Implement the functioning of the ST7735S TFT screen via SPI and DMA
 * @author Gilles Henrard
 * @date 25/07/2025
 *
 * @note Datasheet : https://cdn-shop.adafruit.com/datasheets/ST7735R_V0.2.pdf
 */
#include "st7735s.h"

#include <main.h>
#include <projdefs.h>
#include <stddef.h>
#include <stdint.h>
#include <stm32f103xb.h>
#include <stm32f1xx_ll_dma.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_spi.h>

#include "errorstack.h"
#include "halspi.h"
#include "st7735_initialisation.h"
#include "st7735_registers.h"

/**
 * @brief Enumeration of the function IDs of the SSD1306
 */
typedef enum {
    kSetWindow = 0,  ///< setWindow()
    kSendData,       ///< sendScreenData()
    kPrintChar,      ///< printCharacter()
    kPrintMeasure,   ///< printMeasurements()
    kOrient,         ///< st7735sSetOrientation()
    kStartup,        ///< restartScreen()
    kConfig,         ///< writeConfiguration()
    kSetConfig,      ///< configureST7735S()
} SSD1306functionCode;

/**
 * Structure describing the backporch offsets of the display
 */
typedef struct {
    uint8_t x_offset_px;  ///< X backporch offset in [pixels]
    uint8_t y_offset_px;  ///< Y backporch offset in [pixels]
} __attribute((aligned(2))) Backporch;

//state machine
static ErrorCode restartScreen(void);
static ErrorCode writeConfiguration(void);

//State variables
static ErrorCode result;                                   ///< Buffer used to store function return codes
static Orientation current_orientation = kNBorientations;  ///< Current display orientation
static const Backporch kOffsets[] =                        ///< default backporch offsets
    {
        {1, 2},
};
static DMA dma_descriptor =  ///< Descriptor of the DMA channel used to send data to the display
    {.dma = DMA1,
     .channel = LL_DMA_CHANNEL_5,
     .spi = {.handle = SPI2,
             .cs_port = ST7735S_CS_GPIO_Port,
             .pin = ST7735S_CS_Pin,
             .data_command_port = ST7735S_DC_GPIO_Port,
             .data_command_pin = ST7735S_DC_Pin,
             .highest_register_number = kVCOM4L}};

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * Send the coordinates of the window in which the following data should be displayed on the screen
 *
 * @param x_start    X left-most coordinate of the window in [pixels]
 * @param y_start    X top-most coordinate of the window in [pixels]
 * @param width     Width of the window in [pixels]
 * @param height    Height of the window in [pixels]
 * @retval 0 Success
 * @retval 1 Error while sending the X coordinates of the window
 * @retval 2 Error while sending the Y coordinates of the window
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
ErrorCode setWindow(uint8_t x_start, uint8_t y_start, uint8_t width, uint8_t height) {
    //set the data window columns count
    const uint8_t columns[4] = {0, x_start + kOffsets[0].x_offset_px, 0,
                                (uint8_t)(x_start + kOffsets[0].x_offset_px + width - (uint8_t)1U)};
    result = writeRegisters(&dma_descriptor.spi, kCASET, columns, 4);
    if (isError(result)) {
        return pushErrorCode(result, kSetWindow, 1);
    }

    //set the data window rows count
    const uint8_t rows[4] = {0, y_start + kOffsets[0].y_offset_px, 0,
                             (uint8_t)(y_start + kOffsets[0].y_offset_px + height - (uint8_t)1U)};
    result = writeRegisters(&dma_descriptor.spi, kRASET, rows, 4);
    if (isError(result)) {
        return pushErrorCode(result, kSetWindow, 2);
    }

    return kSuccessCode;
}

/**
 * @brief Set the display orientation
 * 
 * @param orientation New display orientation
 * @return Success
 * @retval 1 Invalid orientation code provided
 * @retval 2 Error while sending the command to the screen
 */
ErrorCode st7735sSetOrientation(Orientation orientation) {
    //if orientation does not change, exit
    if (current_orientation == orientation) {
        return (kSuccessCode);
    }

    //if incorrect orientation, error
    if (orientation >= kNBorientations) {
        return createErrorCode(kOrient, 1, kErrorCritical);
    }

    //send the command to the display

    result = writeRegisters(&dma_descriptor.spi, kMADCTL, &kOrientations[orientation], 1);
    if (isError(result)) {
        return pushErrorCode(result, kOrient, 2);
    }

    //update the current orientation
    current_orientation = orientation;
    return (kSuccessCode);
}

/**
 * @brief Turn the display TFT backlight ON
 */
void turnBacklightON(void) { LL_GPIO_ResetOutputPin(ST7735S_BL_GPIO_Port, ST7735S_BL_Pin); }

/**
 * Attach a FreeRTOS UI task to the screen
 * @details This is used because the task should sleep on DMA send and wake up at DMA done
 *
 * @param handle FreeRTOS task handle
 */
void attachUItask(TaskHandle_t handle) { dma_descriptor.task = handle; }

/**
 * Send the configuration registers and value to the ST7735S
 *
 * @retval 0 Success
 * @retval 1 Error while restarting the display
 * @retval 2 Error while writing the configuration
 */
ErrorCode configureST7735S(void) {
    //make sure to disable ST7735S SPI communication
    LL_SPI_Enable(dma_descriptor.spi.handle);

    result = restartScreen();
    if (isError(result)) {
        return pushErrorCode(result, kSetConfig, 1);
    }

    result = writeConfiguration();
    if (isError(result)) {
        return pushErrorCode(result, kSetConfig, 2);
    }

    return (kSuccessCode);
}

/**
 * Restart the screen module
 * 
 * @return Success
 * @retval 1 Error while transmitting the software reset command
 * @retval 2 Error while transmitting the sleep out command
 */
static ErrorCode restartScreen(void) {
    static const uint8_t kResetDelayMS = 150U;     ///< Number of milliseconds to wait after reset
    static const uint8_t kSleepoutDelayMS = 255U;  ///< Number of milliseconds to wait sleep out

    //send the reset command and, if error, exit
    result = writeRegisters(&dma_descriptor.spi, kSWRESET, NULL, 0);
    if (isError(result)) {
        return pushErrorCode(result, kStartup, 1);
    }

    //wait for a while after software reset
    vTaskDelay(pdMS_TO_TICKS(kResetDelayMS));

    //send the reset command and, if error, exit
    result = writeRegisters(&dma_descriptor.spi, kSLPOUT, NULL, 0);
    if (isError(result)) {
        return pushErrorCode(result, kStartup, 2);
    }

    //wait for a while after sleepout
    vTaskDelay(pdMS_TO_TICKS(kSleepoutDelayMS));

    //save the current systick and get to waiting state
    return (kSuccessCode);
}

/**
 * Configure the screen properly
 * 
 * @return Success
 * @retval 1 Error while sending a command
 * @retval 2 Error while setting the screen orientation
 */
static ErrorCode writeConfiguration(void) {
    //execute all configuration commands

    for (uint8_t command = 0; command < (uint8_t)kST7735nbCommands; command++) {
        result = writeRegisters(&dma_descriptor.spi, (SPIregister)kST7735configurationScript[command].register_number,
                                kST7735configurationScript[command].parameters,
                                kST7735configurationScript[command].nb_parameters);
        if (isError(result)) {
            return pushErrorCode(result, kConfig, 1);
        }
    }

    //set screen orientation
    result = st7735sSetOrientation(kLandscape);
    if (isError(result)) {
        return pushErrorCode(result, kConfig, 2);
    }

    return (kSuccessCode);
}

/**
 * Send a data chunk to the display
 *
 * @param data          Data to send
 * @param nb_bytes       Number of bytes to send
 * @param max_bytes      Maximum number of bytes allowed (usually size of a display buffer)
 * @param screen_area    Display window which will contain the data on the display
 * @retval 0 Success
 * @retval 1 Error while setting the window
 * @retval 2 Error while sending RAM write command
 * @retval 3 Error while sending the data via DMA
 */
ErrorCode sendScreenData(const uint16_t data[], size_t nb_bytes, size_t max_bytes, const Area* screen_area) {
    result = setWindow(screen_area->x0, screen_area->y0, getAreaWidth(screen_area), getAreaHeight(screen_area));
    if (isError(result)) {
        return pushErrorCode(result, kSendData, 1);
    }

    result = writeRegistersAndContinue(&dma_descriptor.spi, kRAMWR, NULL, 0);
    if (isError(result)) {
        return pushErrorCode(result, kSendData, 2);
    }

    result = sendDMA(&dma_descriptor, (uint8_t*)data, nb_bytes, max_bytes);
    if (isError(result)) {
        return pushErrorCode(result, kSendData, 3);
    }

    return kSuccessCode;
}
