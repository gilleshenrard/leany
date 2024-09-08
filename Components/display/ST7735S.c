/**
 * @file ST7735S.c
 * @brief Implement the functioning of the ST7735S TFT screen via SPI and DMA
 * @author Gilles Henrard
 * @date 16/09/2024
 *
 * @note Datasheet : https://cdn-shop.adafruit.com/datasheets/ST7735R_V0.2.pdf
 */
#include "ST7735S.h"
#include <stdint.h>
#include "ST7735_initialisation.h"
#include "ST7735_registers.h"
#include "errorstack.h"
#include "icons.h"
#include "main.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"
#include "systick.h"

enum {
    DISPLAY_WIDTH     = 160U,  ///< Number of pixels in width
    DISPLAY_HEIGHT    = 128U,  ///< Number of pixels in height
    RESET_DELAY_MS    = 150U,  ///< Number of milliseconds to wait after reset
    SLEEPOUT_DELAY_MS = 255U,  ///< Number of milliseconds to wait sleep out
    SPI_TIMEOUT_MS    = 10U,   ///< Number of milliseconds beyond which SPI is in timeout
    FRAME_BUFFER_SIZE = (DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(pixel_t)) / 5U,  ///< Size of the frame buffer in bytes
};

/**
 * @brief Enumeration of the function IDs of the SSD1306
 */
typedef enum {
    INIT = 0,         ///< st7735sInitialise()
    SEND_CMD,         ///< sendCommand()
    FILL_BACKGROUND,  ///< printBackground()
    PRINT_CHAR,       ///< printCharacter()
    ORIENT,           ///< st7735sSetOrientation()
    RESETTING,        ///< stateResetting()
    WAKING,           ///< stateExitingSleep()
    CONFIG,           ///< stateConfiguring()
    WAITING_DMA_RDY,  ///< stateWaitingForTXdone()
} SSD1306functionCodes_e;

/**
 * @brief SPI Data/command pin status enumeration
 */
typedef enum {
    COMMAND = 0,  ///< Command is to be sent
    DATA,         ///< Data is to be sent
} DCgpio_e;

/**
 * @brief Screen state machine state prototype
 *
 * @return Return code of the state
 */
typedef errorCode_u (*screenState)(void);

//utility functions
static inline void setDataCommandGPIO(DCgpio_e function);
static inline void turnBacklightON(void);
// static inline void turnBacklightOFF(void);
static errorCode_u sendCommand(ST7735register_e regNumber, const uint8_t parameters[], uint8_t nbParameters);
static errorCode_u printBackground(void);
static errorCode_u printCharacter(verdanaCharacter_e character, uint8_t Xstart, uint8_t Ystart);

//state machine
static errorCode_u stateResetting(void);
static errorCode_u stateConfiguring(void);
static errorCode_u stateExitingSleep(void);
static errorCode_u stateStartingDMATX(void);
static errorCode_u stateIdle(void);
static errorCode_u stateWaitingForTXdone(void);
static errorCode_u stateError(void);

//State variables
static SPI_TypeDef*    spiHandle      = (void*)0;         ///< SPI handle used with the SSD1306
static DMA_TypeDef*    dmaHandle      = (void*)0;         ///< DMA handle used with the SSD1306
static uint32_t        dmaChannelUsed = 0x00000000U;      ///< DMA channel used
static screenState     state          = stateResetting;   ///< State machine current state
static registerValue_t displayBuffer[FRAME_BUFFER_SIZE];  ///< Buffer used to send data to the display
static systick_t       previousTick_ms = 0;               ///< Latest system tick value saved (in ms)
static errorCode_u     result;                            ///< Buffer used to store function return codes
static uint8_t         displayHeight      = 0;            ///< Current height of the display (depending on orientation)
static uint8_t         displayWidth       = 0;            ///< Current width of the display (depending on orientation)
static orientation_e   currentOrientation = NB_ORIENTATION;  ///< Current display orientation
static uint32_t        dataTXRemaining    = 0;

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Initialise the ST7735S display
 *
 * @param handle        SPI handle used
 * @param dma           DMA handle used
 * @param dmaChannel    DMA channel used to send data to the ST7735S
 * @return Success
 */
errorCode_u st7735sInitialise(SPI_TypeDef* handle, DMA_TypeDef* dma, uint32_t dmaChannel) {
    spiHandle      = handle;
    dmaHandle      = dma;
    dmaChannelUsed = dmaChannel;

    //make sure to disable ST7735S SPI communication
    LL_SPI_Disable(spiHandle);
    LL_DMA_DisableChannel(dmaHandle, dmaChannelUsed);

    //set the DMA source and destination addresses (will always use the same ones)
    LL_DMA_ConfigAddresses(dmaHandle, dmaChannelUsed, (uint32_t)&displayBuffer, LL_SPI_DMA_GetRegAddr(spiHandle),
                           LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

    return (ERR_SUCCESS);
}

/**
 * brief Set the Data/Command pin
 * 
 * @param function Value of the data/command pin
 */
static inline void setDataCommandGPIO(DCgpio_e function) {
    if(function == COMMAND) {
        LL_GPIO_ResetOutputPin(ST7735S_DC_GPIO_Port, ST7735S_DC_Pin);
    } else {
        LL_GPIO_SetOutputPin(ST7735S_DC_GPIO_Port, ST7735S_DC_Pin);
    }
}

/**
 * @brief Send a command with parameters
 *
 * @param regNumber Register number
 * @param parameters Parameters to write
 * @param nbParameters Number of parameters to write
 * @return Success
 * @retval 1	No SPI handle declared
 * @retval 2	No parameters array provided with a non-zero number of parameters
 * @retval 3	Number of parameters above maximum
 * @retval 4	Timeout while sending the command
 */
static errorCode_u sendCommand(ST7735register_e regNumber, const uint8_t parameters[], uint8_t nbParameters) {
    const uint8_t MAX_PARAMETERS = 16U;  ///< Maximum number of parameters a command can have

    //if no handle declared
    if(!spiHandle) {
        return (createErrorCode(SEND_CMD, 1, ERR_WARNING));
    }

    //if nb of params non-zero and parameters array NULL, error
    if(!parameters && nbParameters) {
        return (createErrorCode(SEND_CMD, 2, ERR_WARNING));
    }

    //if too many parameters, error
    if(nbParameters > MAX_PARAMETERS) {
        return (createErrorCode(SEND_CMD, 3, ERR_WARNING));
    }

    //set command pin and enable SPI
    systick_t tickAtStart_ms = getSystick();
    setDataCommandGPIO(COMMAND);
    LL_SPI_Enable(spiHandle);

    //send the command byte and wait for the transaction to be done
    LL_SPI_TransmitData8(spiHandle, (uint8_t)regNumber);
    while(!LL_SPI_IsActiveFlag_TXE(spiHandle) && !isTimeElapsed(tickAtStart_ms, SPI_TIMEOUT_MS)) {}

    //send the parameters
    setDataCommandGPIO(DATA);
    uint8_t* iterator = (uint8_t*)parameters;
    while(nbParameters && !isTimeElapsed(tickAtStart_ms, SPI_TIMEOUT_MS)) {
        //wait for the previous byte to be done, then send the next one
        while(!LL_SPI_IsActiveFlag_TXE(spiHandle) && !isTimeElapsed(tickAtStart_ms, SPI_TIMEOUT_MS)) {}
        if(!isTimeElapsed(tickAtStart_ms, SPI_TIMEOUT_MS)) {
            LL_SPI_TransmitData8(spiHandle, *iterator);
        }

        iterator++;
        nbParameters--;
    }

    //wait for transaction to be finished and clear Overrun flag
    while(LL_SPI_IsActiveFlag_BSY(spiHandle) && !isTimeElapsed(tickAtStart_ms, SPI_TIMEOUT_MS)) {}
    LL_SPI_ClearFlag_OVR(spiHandle);

    //disable SPI and return status
    LL_SPI_Disable(spiHandle);

    //if timeout, error
    if(isTimeElapsed(tickAtStart_ms, SPI_TIMEOUT_MS)) {
        return (createErrorCode(SEND_CMD, 4, ERR_WARNING));
    }

    return (ERR_SUCCESS);
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
    result = sendCommand(MADCTL, &orientations[orientation], 1);
    if(isError(result)) {
        return pushErrorCode(result, ORIENT, 2);
    }

    //update the current orientation
    currentOrientation = orientation;

    //update the display dimensions
    switch(orientation) {
        case PORTRAIT:
        case PORTRAIT_180:
            displayHeight = DISPLAY_WIDTH;
            displayWidth  = DISPLAY_HEIGHT;
            break;

        case LANDSCAPE:
        case LANDSCAPE_180:
        case NB_ORIENTATION:
        default:
            displayHeight = DISPLAY_HEIGHT;
            displayWidth  = DISPLAY_WIDTH;
            break;
    }

    return (ERR_SUCCESS);
}

/**
 * @brief Turn the display TFT backlight ON
 */
static inline void turnBacklightON(void) {
    LL_GPIO_SetOutputPin(ST7735S_BL_GPIO_Port, ST7735S_BL_Pin);
}

// /**
//  * @brief Turn the display TFT backlight ON
//  */
// static inline void turnBacklightOFF(void) {
//     LL_GPIO_ResetOutputPin(ST7735S_BL_GPIO_Port, ST7735S_BL_Pin);
// }

/**
 * @brief Prepare to fill the screen with background colour pixels
 * 
 * @return Success
 * @retval 1 Error while setting data window columns count
 * @retval 2 Error while setting data window rows count
 * @retval 3 Timeout while sending Write Data command
 */
static errorCode_u printBackground(void) {
    const uint8_t BITE_DOWNSHIFT = 8U;
    const uint8_t BITE_MASK      = 0xFFU;
    uint8_t*      iterator       = displayBuffer;

    //set the data window columns count
    uint8_t columns[4] = {0, 0, 0, displayWidth};
    result             = sendCommand(CASET, columns, 4);
    if(isError(result)) {
        return pushErrorCode(result, FILL_BACKGROUND, 1);
    }

    //set the data window rows count
    uint8_t rows[4] = {0, 0, 0, displayHeight + 1};
    result          = sendCommand(RASET, rows, 4);
    if(isError(result)) {
        return pushErrorCode(result, FILL_BACKGROUND, 2);
    }

    //fill the frame buffer with background pixels
    for(uint16_t pixel = 0; pixel < (uint16_t)FRAME_BUFFER_SIZE; pixel++) {
        *(iterator++) = (registerValue_t)((pixel_t)DARK_CHARCOAL >> BITE_DOWNSHIFT);
        *(iterator++) = (registerValue_t)((pixel_t)DARK_CHARCOAL & BITE_MASK);
    }

    //set command pin and enable SPI
    systick_t tickAtStart_ms = getSystick();
    setDataCommandGPIO(COMMAND);
    LL_SPI_Enable(spiHandle);

    //send the command byte and wait for the transaction to be done
    LL_SPI_TransmitData8(spiHandle, (uint8_t)RAMWR);
    while(!LL_SPI_IsActiveFlag_TXE(spiHandle) && !isTimeElapsed(tickAtStart_ms, SPI_TIMEOUT_MS)) {}
    if(isTimeElapsed(tickAtStart_ms, SPI_TIMEOUT_MS)) {
        return pushErrorCode(result, FILL_BACKGROUND, 3);
    }

    //get to sending data state
    dataTXRemaining = (DISPLAY_HEIGHT + 2) * (DISPLAY_WIDTH + 1) * sizeof(pixel_t);
    state           = stateStartingDMATX;
    return (ERR_SUCCESS);
}

/**
 * @brief Prepare to display a character on screen
 * 
 * @param  character Character to print
 * @param  Xstart X coordinate of the character's upper left corner
 * @param  Ystart Y coordinate of the character's upper left corner
 * @return Success
 * @retval 1 Character partially out of screen bounds
 * @retval 2 Error while setting data window columns count
 * @retval 3 Error while setting data window rows count
 * @retval 4 Timeout while sending Write Data command
 */
static errorCode_u printCharacter(verdanaCharacter_e character, uint8_t Xstart, uint8_t Ystart) {
    uint8_t* iterator = displayBuffer;

    //if character completely out of screen, ignore request
    if((Xstart >= DISPLAY_WIDTH) || (Ystart >= DISPLAY_HEIGHT)) {
        return (ERR_SUCCESS);
    }

    //if character partially out of screen, error (easier)
    if(((Xstart + VERDANA_NB_COLUMNS) >= DISPLAY_WIDTH) || ((Ystart + VERDANA_NB_ROWS) >= DISPLAY_HEIGHT)) {
        return createErrorCode(PRINT_CHAR, 1, ERR_WARNING);
    }

    //set the data window columns count
    uint8_t columns[4] = {0, Xstart + 2U, 0, Xstart + VERDANA_NB_COLUMNS + 1U};
    result             = sendCommand(CASET, columns, 4);
    if(isError(result)) {
        return pushErrorCode(result, PRINT_CHAR, 2);
    }

    //set the data window rows count
    uint8_t rows[4] = {0, Ystart + 2U, 0, Ystart + VERDANA_NB_ROWS + 2U};
    result          = sendCommand(RASET, rows, 4);
    if(isError(result)) {
        return pushErrorCode(result, PRINT_CHAR, 3);
    }

    //fill the frame buffer with background pixels
    for(uint8_t row = 0; row < (uint8_t)VERDANA_NB_ROWS; row++) {
        uncompressIconLine(iterator, character, row);
        iterator += ((uint8_t)VERDANA_NB_COLUMNS << 1U);
    }

    //set command pin and enable SPI
    systick_t tickAtStart_ms = getSystick();
    setDataCommandGPIO(COMMAND);
    LL_SPI_Enable(spiHandle);

    //send the command byte and wait for the transaction to be done
    LL_SPI_TransmitData8(spiHandle, (uint8_t)RAMWR);
    while(!LL_SPI_IsActiveFlag_TXE(spiHandle) && !isTimeElapsed(tickAtStart_ms, SPI_TIMEOUT_MS)) {}
    if(isTimeElapsed(tickAtStart_ms, SPI_TIMEOUT_MS)) {
        return pushErrorCode(result, PRINT_CHAR, 4);
    }

    //get to sending data state
    dataTXRemaining = VERDANA_NB_COLUMNS * VERDANA_NB_ROWS * sizeof(pixel_t);
    state           = stateStartingDMATX;
    return (ERR_SUCCESS);
}

/**
 * @brief Run the state machine
 *
 * @return Return code of the current state
 */
errorCode_u st7735sUpdate(void) {
    return ((*state)());
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief State in which a software reset is requested
 * 
 * @return Success
 * @retval 1 Error while transmitting the command
 */
static errorCode_u stateResetting(void) {
    //send the reset command and, if error, exit
    result = sendCommand(SWRESET, NULL, 0);
    if(isError(result)) {
        state = stateError;
        return pushErrorCode(result, RESETTING, 1);
    }

    //save the current systick and get to waiting state
    previousTick_ms = getSystick();
    state           = stateExitingSleep;
    return (ERR_SUCCESS);
}

/**
 * @brief State in which a sleep out is requested
 * 
 * @return Success
 * @retval 1 Error while transmitting the command
 */
static errorCode_u stateExitingSleep(void) {
    //if reset timer not elapsed yet, exit
    if(!isTimeElapsed(previousTick_ms, RESET_DELAY_MS)) {
        return (ERR_SUCCESS);
    }

    //send the reset command and, if error, exit
    result = sendCommand(SLPOUT, NULL, 0);
    if(isError(result)) {
        state = stateError;
        return pushErrorCode(result, WAKING, 1);
    }

    previousTick_ms = getSystick();
    state           = stateConfiguring;
    return (ERR_SUCCESS);
}

/**
 * @brief State in which the screen is being configured properly
 * 
 * @return Success
 * @retval 1 Error while sending a command
 * @retval 2 Error while setting the screen orientation
 */
static errorCode_u stateConfiguring(void) {
    //if sleep out timer not elapsed yet, exit
    if(!isTimeElapsed(previousTick_ms, SLEEPOUT_DELAY_MS)) {
        return (ERR_SUCCESS);
    }

    //execute all configuration commands
    for(uint8_t command = 0; command < (uint8_t)ST7735_NB_COMMANDS; command++) {
        result =
            sendCommand(st7735configurationScript[command].registerNumber,
                        st7735configurationScript[command].parameters, st7735configurationScript[command].nbParameters);
        if(isError(result)) {
            state = stateError;
            return pushErrorCode(result, CONFIG, 1);
        }
    }

    //set screen orientation
    result = st7735sSetOrientation(LANDSCAPE_180);
    if(isError(result)) {
        state = stateError;
        return pushErrorCode(result, CONFIG, 2);
    }

    //turn on backlight
    turnBacklightON();

    //fill the screen with the background colour
    printBackground();

    state = stateStartingDMATX;
    return (ERR_SUCCESS);
}

/**
 * @brief State in which data is being sent to the display
 * 
 * @return Success
 */
static errorCode_u stateStartingDMATX(void) {
    //if all data sent, close DMA and SPI and get to idle state
    if(!dataTXRemaining) {
        LL_DMA_DisableChannel(dmaHandle, dmaChannelUsed);
        LL_SPI_Disable(spiHandle);
        state = stateIdle;
        return (ERR_SUCCESS);
    }

    //clamp the data to send to max. the frameBuffer
    uint32_t dataToSend = (dataTXRemaining > FRAME_BUFFER_SIZE ? FRAME_BUFFER_SIZE : dataTXRemaining);

    //set data GPIO and enable SPI
    setDataCommandGPIO(DATA);

    //configure the DMA transaction
    LL_DMA_DisableChannel(dmaHandle, dmaChannelUsed);
    LL_DMA_ClearFlag_GI5(dmaHandle);
    LL_DMA_SetDataLength(dmaHandle, dmaChannelUsed, dataToSend);  //must be reset every time
    LL_DMA_EnableChannel(dmaHandle, dmaChannelUsed);

    //send the data
    previousTick_ms = getSystick();
    LL_SPI_EnableDMAReq_TX(spiHandle);

    //get to next
    dataTXRemaining -= dataToSend;
    state = stateWaitingForTXdone;
    return (ERR_SUCCESS);
}

/**
 * @brief State in which the machine waits for a DMA transmission to end
 *
 * @return Success
 * @retval 1	Timeout while waiting for transmission to end
 * @retval 2	Error interrupt occurred during the DMA transfer
 */
static errorCode_u stateWaitingForTXdone(void) {
    //if timer elapsed, stop DMA and error
    if(isTimeElapsed(previousTick_ms, SPI_TIMEOUT_MS)) {
        LL_DMA_DisableChannel(dmaHandle, dmaChannelUsed);
        LL_SPI_Disable(spiHandle);
        state = stateError;
        return createErrorCode(WAITING_DMA_RDY, 1, ERR_ERROR);
    }

    //if DMA error, stop DMA and error
    if(LL_DMA_IsActiveFlag_TE5(dmaHandle)) {
        LL_DMA_DisableChannel(dmaHandle, dmaChannelUsed);
        LL_SPI_Disable(spiHandle);
        state = stateError;
        return createErrorCode(WAITING_DMA_RDY, 2, ERR_ERROR);
    }

    //if transmission complete, get to sending data state
    if(LL_DMA_IsActiveFlag_TC5(dmaHandle)) {
        state = stateStartingDMATX;
    }

    return (ERR_SUCCESS);
}

/**
 * @brief State in which the screen waits for instructions
 * 
 * @return Success
 */
static errorCode_u stateIdle(void) {
    static uint8_t nbChar = 1;

    if(nbChar) {
        nbChar--;
        result = printCharacter(VERDANA_1, 0, 0);
        if(isError(result)) {
            state = stateError;
        }
    }

    return (ERR_SUCCESS);
}

/**
 * @brief State in which the screen is in error
 * 
 * @return Success
 */
static errorCode_u stateError(void) {
    return (ERR_SUCCESS);
}
