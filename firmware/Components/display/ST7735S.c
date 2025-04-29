/**
 * @file ST7735S.c
 * @brief Implement the functioning of the ST7735S TFT screen via SPI and DMA
 * @author Gilles Henrard
 * @date 06/10/2024
 *
 * @note Datasheet : https://cdn-shop.adafruit.com/datasheets/ST7735R_V0.2.pdf
 */
#include "ST7735S.h"
#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "ST7735_initialisation.h"
#include "ST7735_registers.h"
#include "errorstack.h"
#include "icons.h"
#include "main.h"
#include "memsBMI270.h"
#include "portmacro.h"
#include "projdefs.h"
#include "queue.h"
#include "stdbool.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"
#include "task.h"

enum {
    STACK_SIZE        = 160U,  ///< Amount of words in the task stack
    TASK_LOW_PRIORITY = 8U,    ///< FreeRTOS number for a low priority task
    DISPLAY_WIDTH     = 160U,  ///< Number of pixels in width
    DISPLAY_HEIGHT    = 128U,  ///< Number of pixels in height
    SPI_TIMEOUT_MS    = 10U,   ///< Number of milliseconds beyond which SPI is in timeout
    MSG_TIMEOUT_MS    = 2U,    ///< Maximum number of milliseconds to wait for a message in the queue
    NB_QUEUE_ELEM     = 10U,   ///< Maximum number of messages in the queue
    FRAME_BUFFER_SIZE = (DISPLAY_WIDTH * DISPLAY_HEIGHT) / 10U,  ///< Size of the frame buffer in bytes
};

/**
 * @brief Enumeration of the function IDs of the SSD1306
 */
typedef enum {
    SEND_CMD = 0,  ///< sendCommand()
    SET_WINDOW,    ///< setWindow()
    SEND_DATA,     ///< sendData()
    PRT_CHAR,      ///< printCharacter()
    ORIENT,        ///< st7735sSetOrientation()
    STARTUP,       ///< stateStartup()
    CONFIG,        ///< stateConfiguring()
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
static void        taskST7735S(void* argument);
static inline void setDataCommandGPIO(DCgpio_e function);
static errorCode_u setWindow(uint8_t Xstart, uint8_t Ystart, uint8_t width, uint8_t height);
static inline void turnBacklightON(void);
static errorCode_u sendCommand(ST7735register_e regNumber, const uint8_t parameters[], uint8_t nbParameters,
                               uint8_t finalise);
static errorCode_u printCharacter(verdanaCharacter_e character, uint8_t xLeft, uint8_t yTop);

//state machine
static errorCode_u stateStartup(void);
static errorCode_u stateConfiguring(void);
static errorCode_u stateFillingBackground(void);
static errorCode_u stateIdle(void);
static errorCode_u stateError(void);

//State variables
static volatile TaskHandle_t taskHandle     = NULL;             ///< handle of the FreeRTOS task
static SPI_TypeDef*          spiHandle      = (void*)0;         ///< SPI handle used with the SSD1306
static DMA_TypeDef*          dmaHandle      = (void*)0;         ///< DMA handle used with the SSD1306
static uint32_t              dmaChannelUsed = 0x00000000U;      ///< DMA channel used
static screenState           state          = stateStartup;     ///< State machine current state
static pixel_t               displayBuffer[FRAME_BUFFER_SIZE];  ///< Buffer used to send data to the display
static errorCode_u           result;                            ///< Buffer used to store function return codes
static uint8_t               displayHeight      = 0;  ///< Current height of the display (depending on orientation)
static uint8_t               displayWidth       = 0;  ///< Current width of the display (depending on orientation)
static orientation_e         currentOrientation = NB_ORIENTATION;  ///< Current display orientation
static TickType_t            previousTick       = 0;     ///< Variable used to remember the previous systick saved
QueueHandle_t                messageStack       = NULL;  ///< Thread-safe message stack

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Handle DMA Transfer Complete interruptions
 */
void st7735sDMAinterruptHandler(void) {
    BaseType_t hasWoken = 0;
    vTaskNotifyGiveFromISR(taskHandle, &hasWoken);
    portYIELD_FROM_ISR(hasWoken);
}

/**
 * @brief Create the ST7735S FreeRTOS task
 *
 * @param handle        SPI handle used
 * @param dma           DMA handle used
 * @param dmaChannel    DMA channel used to send data to the ST7735S
 * @return Success
 */
errorCode_u createST7735Stask(SPI_TypeDef* handle, DMA_TypeDef* dma, uint32_t dmaChannel) {
    static StackType_t   taskStack[STACK_SIZE] = {0};  ///< Buffer used as the task stack
    static StaticTask_t  taskState             = {0};  ///< Task state variables
    static uint8_t       queueStorage[sizeof(displayMessage_t) * NB_QUEUE_ELEM] = {0};
    static StaticQueue_t queueState                                             = {0};

    //save the SPI handle and DMA channel used by the ST7735S
    spiHandle      = handle;
    dmaHandle      = dma;
    dmaChannelUsed = dmaChannel;

    //make sure to disable ST7735S SPI communication
    LL_SPI_Disable(spiHandle);
    LL_DMA_DisableChannel(dmaHandle, dmaChannelUsed);
    LL_DMA_EnableIT_TC(dmaHandle, dmaChannelUsed);

    //set the DMA source and destination addresses (will always use the same ones)
    LL_DMA_ConfigAddresses(dmaHandle, dmaChannelUsed, (uint32_t)&displayBuffer, LL_SPI_DMA_GetRegAddr(spiHandle),
                           LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

    //create the static task
    taskHandle =
        xTaskCreateStatic(taskST7735S, "ST7735S task", STACK_SIZE, NULL, TASK_LOW_PRIORITY, taskStack, &taskState);
    if(!taskHandle) {
        Error_Handler();
    }

    messageStack = xQueueCreateStatic(NB_QUEUE_ELEM, sizeof(displayMessage_t), queueStorage, &queueState);
    if(!messageStack) {
        Error_Handler();
    }

    return (ERR_SUCCESS);
}

/**
 * @brief Run the state machine
 *
 * @return Return code of the current state
 */
static void taskST7735S(void* argument) {
    UNUSED(argument);

    while(1) {
        result = (*state)();
        if(isError(result)) {
            Error_Handler();
        }
    }
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

//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
static errorCode_u setWindow(uint8_t Xstart, uint8_t Ystart, uint8_t width, uint8_t height) {
    //set the data window columns count
    uint8_t columns[4] = {0, Xstart, 0, Xstart + width};
    result             = sendCommand(CASET, columns, 4, true);
    if(isError(result)) {
        return pushErrorCode(result, SET_WINDOW, 1);
    }

    //set the data window rows count
    uint8_t rows[4] = {0, Ystart, 0, Ystart + height};
    result          = sendCommand(RASET, rows, 4, true);
    if(isError(result)) {
        return pushErrorCode(result, SET_WINDOW, 2);
    }

    return ERR_SUCCESS;
}

/**
 * @brief Send a command with parameters
 *
 * @param regNumber Register number
 * @param parameters Parameters to write
 * @param nbParameters Number of parameters to write
 * @param finalise True if needing to reset CS and disable SPI
 * @return Success
 * @retval 1	No SPI handle declared
 * @retval 2	No parameters array provided with a non-zero number of parameters
 * @retval 3	Number of parameters above maximum
 * @retval 4	Timeout while sending the command
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
static errorCode_u sendCommand(ST7735register_e regNumber, const uint8_t parameters[], uint8_t nbParameters,
                               uint8_t finalise) {
    const uint8_t MAX_PARAMETERS = 16U;

    //if nb of params non-zero and parameters array NULL, error
    if(!parameters && nbParameters) {
        return (createErrorCode(SEND_CMD, 2, ERR_WARNING));
    }

    //if too many parameters, error
    if(nbParameters > MAX_PARAMETERS) {
        return (createErrorCode(SEND_CMD, 3, ERR_WARNING));
    }

    //set command pin and enable SPI
    uint32_t SPItick = HAL_GetTick();
    setDataCommandGPIO(COMMAND);
    LL_SPI_Enable(spiHandle);

    //send the command byte and wait for the transaction to be done
    LL_SPI_TransmitData8(spiHandle, (uint8_t)regNumber);
    while(!LL_SPI_IsActiveFlag_TXE(spiHandle) && ((HAL_GetTick() - SPItick) < SPI_TIMEOUT_MS)) {}

    //send the parameters
    setDataCommandGPIO(DATA);
    uint8_t* iterator = (uint8_t*)parameters;
    while(nbParameters && ((HAL_GetTick() - SPItick) < SPI_TIMEOUT_MS)) {
        //wait for the previous byte to be done, then send the next one
        while(!LL_SPI_IsActiveFlag_TXE(spiHandle) && ((HAL_GetTick() - SPItick) < SPI_TIMEOUT_MS)) {}
        if(LL_SPI_IsActiveFlag_TXE(spiHandle)) {
            LL_SPI_TransmitData8(spiHandle, *iterator);
        }

        iterator++;
        nbParameters--;
    }

    //wait for transaction to be finished and clear Overrun flag
    while(LL_SPI_IsActiveFlag_BSY(spiHandle) && ((HAL_GetTick() - SPItick) < SPI_TIMEOUT_MS)) {}
    LL_SPI_ClearFlag_OVR(spiHandle);

    //if timeout, error
    if(((HAL_GetTick() - SPItick) >= SPI_TIMEOUT_MS)) {
        LL_SPI_Disable(spiHandle);
        return (createErrorCode(SEND_CMD, 4, ERR_WARNING));
    }

    if(finalise) {
        LL_SPI_Disable(spiHandle);
    }
    return (ERR_SUCCESS);
}

/**
 * @brief Send pixel data to the display
 * 
 * @param dataRemaining Number of bytes remaining to send
 * @return Success
 * @retval 1 Timeout while sending write command
 * @retval 2 Timeout while waiting for DMA TX done interrupt
 * @retval 3 Error occurred durint DMA transmission
 */
//NOLINTNEXTLINE(bugprone-easily-swappable-parameters)
static errorCode_u sendData(uint32_t* dataRemaining) {
    static uint8_t WriteCommandSent = 0;

    if(!dataRemaining) {
        return ERR_SUCCESS;
    }

    if(!WriteCommandSent) {
        sendCommand(RAMWR, NULL, 0, false);
        WriteCommandSent = 1;
    }

    //clamp the data to send to max. the frameBuffer
    uint32_t dataToSend =
        (*dataRemaining > (FRAME_BUFFER_SIZE * sizeof(pixel_t)) ? (FRAME_BUFFER_SIZE * sizeof(pixel_t))
                                                                : *dataRemaining);

    //set data GPIO and enable SPI
    setDataCommandGPIO(DATA);

    //configure the DMA transaction
    LL_DMA_DisableChannel(dmaHandle, dmaChannelUsed);
    LL_DMA_ClearFlag_GI5(dmaHandle);
    LL_DMA_SetDataLength(dmaHandle, dmaChannelUsed, dataToSend);  //must be reset every time
    LL_DMA_EnableChannel(dmaHandle, dmaChannelUsed);
    LL_SPI_EnableDMAReq_TX(spiHandle);

    result = ERR_SUCCESS;

    //wait for measurements to be ready
    if(ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SPI_TIMEOUT_MS)) == pdFALSE) {
        result = createErrorCode(SEND_DATA, 2, ERR_ERROR);
        goto finaliseDMA;
    }

    //if DMA error, stop DMA and error
    if(LL_DMA_IsActiveFlag_TE5(dmaHandle)) {
        result = createErrorCode(SEND_DATA, 3, ERR_ERROR);
        goto finaliseDMA;
    }

    *dataRemaining -= dataToSend;
    if(*dataRemaining) {
        return ERR_SUCCESS;
    }

finaliseDMA:
    LL_DMA_DisableChannel(dmaHandle, dmaChannelUsed);
    LL_SPI_Disable(spiHandle);
    if(isError(result)) {
        state = stateError;
    }
    WriteCommandSent = 0;
    return result;
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
    result = sendCommand(MADCTL, &orientations[orientation], 1, true);
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

/**
 * @brief Send a message to the display
 * 
 * @param message Message to send
 * @retval 0 Message sent
 * @retval 1 Message could not be sent in a timely manner
 */
uint8_t sendDisplayMessage(const displayMessage_t* message) {
    return (xQueueSendToFront(messageStack, message, MSG_TIMEOUT_MS) != pdTRUE);
}

/**
 * @brief Print a Verdana character on screen
 * 
 * @param character Character to print
 * @param xLeft Left-most coordinate of the character
 * @param yTop Top-most coordinate of the character
 * @return Success
 * @retval 1 Error while setting the window
 * @retval 2 Error while sending the pixel data to the screen
 */
static errorCode_u printCharacter(verdanaCharacter_e character, uint8_t xLeft, uint8_t yTop) {
    uint32_t characterSize = 0;

    result = setWindow(xLeft, yTop, VERDANA_NB_COLUMNS - 1, VERDANA_NB_ROWS - 1);
    if(isError(result)) {
        state = stateError;
        return pushErrorCode(result, PRT_CHAR, 1);
    }

    //fill the frame buffer with background pixels
    uncompressCharacter(displayBuffer, character);

    characterSize = VERDANA_NB_COLUMNS * VERDANA_NB_ROWS * sizeof(pixel_t);
    result        = sendData(&characterSize);
    if(isError(result)) {
        state = stateError;
        return pushErrorCode(result, PRT_CHAR, 2);
    }

    return ERR_SUCCESS;
}

/**
 * @brief Print measurement values in 0.1°
 * 
 * @param axis Axis of which to print the measurements
 * @return Success
 */
static errorCode_u printMeasurements(axis_e axis) {
    const uint8_t SECOND_LINE_Y = 50U;
    const uint8_t MULTIPLE_10   = 10U;
    const uint8_t MULTIPLE_100  = 100U;
    int16_t       measurement   = getAngleDegreesTenths(axis);
    uint8_t       yTop          = (axis == X_AXIS ? 0 : SECOND_LINE_Y);

    if(measurement >= 0) {
        printCharacter(VERDANA_PLUS, 0, yTop);
    } else {
        measurement = (int16_t)-measurement;
        printCharacter(VERDANA_MIN, 0, yTop);
    }
    printCharacter((verdanaCharacter_e)(measurement / MULTIPLE_100), VERDANA_NB_COLUMNS, yTop);
    printCharacter((verdanaCharacter_e)((measurement / MULTIPLE_10) % MULTIPLE_10), (VERDANA_NB_COLUMNS * 2), yTop);
    printCharacter((verdanaCharacter_e)(measurement % MULTIPLE_10), (VERDANA_NB_COLUMNS * 4), yTop);

    return ERR_SUCCESS;
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief State in which a software reset and a sleepout are requested
 * 
 * @return Success
 * @retval 1 Error while transmitting the software reset command
 * @retval 2 Error while transmitting the sleep out command
 */
static errorCode_u stateStartup(void) {
    const uint8_t RESET_DELAY_MS    = 150U;  ///< Number of milliseconds to wait after reset
    const uint8_t SLEEPOUT_DELAY_MS = 255U;  ///< Number of milliseconds to wait sleep out

    //send the reset command and, if error, exit
    result = sendCommand(SWRESET, NULL, 0, true);
    if(isError(result)) {
        state = stateError;
        return pushErrorCode(result, STARTUP, 1);
    }

    //wait for a while after software reset
    vTaskDelay(pdMS_TO_TICKS(RESET_DELAY_MS));

    //send the reset command and, if error, exit
    result = sendCommand(SLPOUT, NULL, 0, true);
    if(isError(result)) {
        state = stateError;
        return pushErrorCode(result, STARTUP, 2);
    }

    //wait for a while after sleepout
    vTaskDelay(pdMS_TO_TICKS(SLEEPOUT_DELAY_MS));

    //save the current systick and get to waiting state
    state = stateConfiguring;
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
    //execute all configuration commands
    for(uint8_t command = 0; command < (uint8_t)ST7735_NB_COMMANDS; command++) {
        result = sendCommand(st7735configurationScript[command].registerNumber,
                             st7735configurationScript[command].parameters,
                             st7735configurationScript[command].nbParameters, true);
        if(isError(result)) {
            state = stateError;
            return pushErrorCode(result, CONFIG, 1);
        }
    }

    //set screen orientation
    result = st7735sSetOrientation(LANDSCAPE);
    if(isError(result)) {
        state = stateError;
        return pushErrorCode(result, CONFIG, 2);
    }

    previousTick = xTaskGetTickCount();
    state        = stateFillingBackground;
    return (ERR_SUCCESS);
}

static errorCode_u stateFillingBackground(void) {
    //fill the frame buffer with background pixels
    for(uint16_t pixel = 0; pixel < (uint16_t)FRAME_BUFFER_SIZE; pixel += 2) {
        *((uint16_t*)(&displayBuffer[pixel])) = DARK_CHARCOAL_BIGENDIAN;
    }

    result = setWindow(0, 0, displayWidth, displayHeight);
    if(isError(result)) {
        state = stateError;
        return pushErrorCode(result, CONFIG, 3);
    }

    uint32_t dataTXRemaining = (DISPLAY_HEIGHT + 2) * (DISPLAY_WIDTH + 1) * sizeof(pixel_t);
    do {
        result = sendData(&dataTXRemaining);
    } while(dataTXRemaining && !isError(result));

    if(isError(result)) {
        state = stateError;
        return pushErrorCode(result, CONFIG, 4);
    }

    //turn on backlight
    turnBacklightON();

    state = stateIdle;
    return ERR_SUCCESS;
}

/**
 * @brief State in which the screen waits for instructions
 * 
 * @return Success
 */
static errorCode_u stateIdle(void) {
    const uint8_t REFRESH_DELAY_MS = 30U;

    vTaskDelayUntil(&previousTick, pdMS_TO_TICKS(REFRESH_DELAY_MS));

    if(anglesChanged()) {
        printMeasurements(X_AXIS);
        printMeasurements(Y_AXIS);
    }

    // displayMessage_t message = {0};
    // while(xQueueReceive(messageStack, &message, pdMS_TO_TICKS(MSG_TIMEOUT_MS)) == pdTRUE) {
    //     switch(message.ID) {
    //         case MSG_HOLD:
    //         case MSG_ZERO:
    //         case MSG_PWROFF:
    //         case NB_MESSAGES:
    //         default:
    //             break;
    //     }
    // }

    return ERR_SUCCESS;
}

/**
 * @brief State in which the screen is in error
 * 
 * @return Success
 */
static errorCode_u stateError(void) {
    return (ERR_SUCCESS);
}
