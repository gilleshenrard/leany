/**
 * @file SSD1306.c
 * @brief Implement the functioning of the SSD1306 OLED screen via SPI and DMA
 * @author Gilles Henrard
 * @date 26/07/2024
 *
 * @note Datasheet : https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
 */
#include "SSD1306.h"
#include <assert.h>
#include <stdint.h>
#include "SSD1306_registers.h"
#include "errorstack.h"
#include "icons.h"
#include "main.h"
#include "numbersVerdana16.h"
#include "stm32f103xb.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"
#include "systick.h"

//Definitions
enum {
    SSD_SCREEN_WIDTH  = 128U,                               ///< Number of columns on the screen
    SSD_SCREEN_HEIGHT = 64U,                                ///< Number of rows (i.e. COM connections) on the screen
    SSD_NB_PAGES     = ((uint8_t)SSD_SCREEN_HEIGHT >> 3U),  ///< Number of pages (1 page = 8 COM) present to update rows
    REFICON_PAGE     = (SSD_NB_PAGES - 1),
    REFICON_COLUMN   = (SSD_SCREEN_WIDTH - REFERENCETYPE_NB_BYTES - 1),
    HOLDICON_PAGE    = REFICON_PAGE,
    HOLDICON_COLUMN  = (REFICON_COLUMN - REFERENCETYPE_NB_BYTES),
    ANGLE_NB_CHARS   = 6U,   ///< Number of characters in the angle array
    ANGLE_COLUMN     = 40U,  ///< Column number of the first screen line
    ANGLE_ROLL_PAGE  = 1U,   ///< Number of the page at which display the roll axis angle
    ANGLE_PITCH_PAGE = 5U,   ///< Number of the page at which display the pitch axis angle
};

/**
 * @brief Enumeration of the function IDs of the SSD1306
 */
typedef enum {
    INIT = 0,         ///< SSD1306initialise()
    SEND_CMD,         ///< SSD1306sendCommand()
    PRT_ANGLE,        ///< SSD1306_printAngleTenths()
    PRT_REFICON,      ///< SSD1306_printReferentialIcon()
    PRT_HOLDICON,     ///< SSD1306_printHoldIcon()
    SENDING_DATA,     ///< stateSendingData()
    WAITING_DMA_RDY,  ///< stateWaitingForTXdone()
} SSD1306functionCodes_e;

/**
 * @brief SPI Data/command pin status enumeration
 */
typedef enum {
    COMMAND = 0,
    DATA,
} DCgpio_e;

/**
 * @brief Screen state machine state prototype
 *
 * @return Return code of the state
 */
typedef errorCode_u (*screenState)();

//communication functions with the SSD1306
static inline void setDataCommandGPIO(DCgpio_e function);
static errorCode_u sendCommand(SSD1306register_e regNumber, const uint8_t parameters[], uint8_t nbParameters);
static errorCode_u drawBaseScreen();

//state machine
static errorCode_u stateConfiguring();
static errorCode_u stateIdle();
static errorCode_u stateSendingData();
static errorCode_u stateWaitingForTXdone();

//Constant values
static const uint8_t SPI_TIMEOUT_MS = 10U;  ///< Maximum number of milliseconds SPI traffic should last before timeout

//Variables used in interrupts  ///< Timer used to make sure SPI does not time out (in ms)
static systick_t TXtick = 0;

//State variables
static SPI_TypeDef* spiHandle         = (void*)0;          ///< SPI handle used with the SSD1306
static DMA_TypeDef* dmaHandle         = (void*)0;          ///< DMA handle used with the SSD1306
static uint32_t     dmaChannelUsed    = 0x00000000U;       ///< DMA channel used
static screenState  state             = stateConfiguring;  ///< State machine current state
static uint8_t      screenInvalidated = 0;  ///< Flag indicating the screen has been invalidated and should be updated
static uint8_t      screenBuffer[SSD_NB_PAGES][SSD_SCREEN_WIDTH];  ///< Buffer used to send data to the screen

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Initialise the SSD1306
 *
 * @param handle        SPI handle used
 * @param dma           DMA handle used
 * @param dmaChannel    DMA channel used to send data to the SSD1306
 * @return Success
 * @retval 1	        Error while initialising the registers
 * @retval 2	        Error while clearing the screen
 */
errorCode_u ssd1306Initialise(SPI_TypeDef* handle, DMA_TypeDef* dma, uint32_t dmaChannel) {
    spiHandle      = handle;
    dmaHandle      = dma;
    dmaChannelUsed = dmaChannel;

    //make sure to disable SSD1306 SPI communication
    LL_SPI_Disable(spiHandle);
    LL_DMA_DisableChannel(dmaHandle, dmaChannelUsed);

    //set the DMA source and destination addresses (will always use the same ones)
    LL_DMA_ConfigAddresses(dmaHandle, dmaChannelUsed, (uint32_t)&screenBuffer, LL_SPI_DMA_GetRegAddr(spiHandle),
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
        LL_GPIO_ResetOutputPin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin);
    } else {
        LL_GPIO_SetOutputPin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin);
    }
}

/**
 * @brief Send a command with parameters
 *
 * @param regNumber Register number
 * @param parameters Parameters to write
 * @param nbParameters Number of parameters to write
 * @return Success
 * @retval 1	Number of parameters above maximum
 * @retval 2	Timeout while sending the command
 */
errorCode_u sendCommand(SSD1306register_e regNumber, const uint8_t parameters[], uint8_t nbParameters) {
    const uint8_t MAX_PARAMETERS = 6U;  ///< Maximum number of parameters a command can have
    errorCode_u   result         = ERR_SUCCESS;

    //assertions
    assert(spiHandle);                    //handle is not null
    assert(parameters || !nbParameters);  //either 0 parameters, or parameters array not null

    //if too many parameters, error
    if(nbParameters > MAX_PARAMETERS) {
        return (createErrorCode(SEND_CMD, 1, ERR_WARNING));
    }

    //set command pin and enable SPI
    systick_t tickAtStart_ms = getSystick();
    setDataCommandGPIO(COMMAND);
    LL_SPI_Enable(spiHandle);

    //send the command byte
    LL_SPI_TransmitData8(spiHandle, (uint8_t)regNumber);

    //send the parameters
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
        return (createErrorCode(SEND_CMD, 2, ERR_WARNING));
    }

    return (result);
}

/**
 * @brief Wipe the screen blank and draw the separator and icons
 * @note This function invalidates the screen
 *
 * @return Success
 */
errorCode_u drawBaseScreen() {
    uint8_t*       iteratorBuffer = (uint8_t*)screenBuffer;
    const uint8_t* iteratorIcon   = baseScreen;

    //copy the base screen in the buffer
    for(uint16_t counter = 0; counter < (uint16_t)MAX_DATA_SIZE; counter++) {
        *(iteratorBuffer++) = *(iteratorIcon++);
    }

    //invalidate the screen and exit
    screenInvalidated = 1;
    return (ERR_SUCCESS);
}

/**
 * @brief Check if the screen is ready to accept new commands
 *
 * @return 0 Not ready
 * @retval 1 Ready
 */
uint8_t isScreenReady() {
    return (state == stateIdle);
}

/**
 * @brief Print an angle (in degrees, with sign) on the screen
 * @note This function invalidates the screen
 *
 * @param angleTenths	Angle to print
 * @param rotationAxis  Axis around which the rotation angle is to print
 *
 * @return Success
 * @retval 1 Screen busy
 */
errorCode_u ssd1306PrintAngleTenths(int16_t angleTenths, rotationAxis_e rotationAxis) {
    const int16_t MIN_ANGLE_DEG_TENTHS        = -900;  ///< Minimum angle allowed (in tenths of degrees)
    const int16_t MAX_ANGLE_DEG_TENTHS        = 900;   ///< Maximum angle allowed (in tenths of degrees)
    const uint8_t INDEX_SIGN                  = 0;     ///< Index of the sign in the angle indexes array
    const uint8_t INDEX_TENS                  = 1U;    ///< Index of the tens in the angle indexes array
    const uint8_t INDEX_UNITS                 = 2U;    ///< Index of the units in the angle indexes array
    const uint8_t INDEX_TENTHS                = 4U;    ///< Index of the tenths in the angle indexes array
    const uint8_t DIVIDE_10                   = 10U;   ///< 10 Divider (used for magic numbers warnings)
    const uint8_t DIVIDE_100                  = 100U;  ///< 100 Divider (used for magic numbers warnings)
    uint8_t       charIndexes[ANGLE_NB_CHARS] = {INDEX_PLUS, 0, 0, INDEX_DOT, 0, INDEX_DEG};
    uint8_t       anglePage                   = (rotationAxis == ROLL ? ANGLE_ROLL_PAGE : ANGLE_PITCH_PAGE);
    uint8_t*      bytesToUpdate               = (void*)0;

    //if screen busy, error
    if(!isScreenReady()) {
        return (createErrorCode(PRT_ANGLE, 1, ERR_WARNING));
    }

    //clamp the angle to print to the min value
    if(angleTenths < MIN_ANGLE_DEG_TENTHS) {
        angleTenths = MIN_ANGLE_DEG_TENTHS;
    }

    //clamp the angle to print to the max value
    if(angleTenths > MAX_ANGLE_DEG_TENTHS) {
        angleTenths = MAX_ANGLE_DEG_TENTHS;
    }

    //if angle negative, replace plus sign with minus sign
    if(angleTenths < 0) {
        charIndexes[INDEX_SIGN] = INDEX_MINUS;
        angleTenths             = (int16_t)-angleTenths;
    }

    //fill the angle characters indexes array with the float values (tens, units, tenths)
    charIndexes[INDEX_TENS]   = (uint8_t)(angleTenths / DIVIDE_100);
    charIndexes[INDEX_UNITS]  = (uint8_t)((angleTenths / DIVIDE_10) % DIVIDE_10);
    charIndexes[INDEX_TENTHS] = (uint8_t)(angleTenths % DIVIDE_10);

    //fill the buffer with the angle pixels
    for(uint8_t page = 0; page < (uint8_t)VERDANA_NB_PAGES; page++) {
        //point to the beginning of the section in the current page which will be updated
        bytesToUpdate = &screenBuffer[anglePage + page][ANGLE_COLUMN];

        //update each required byte in the page
        for(uint8_t character = 0; character < (uint8_t)ANGLE_NB_CHARS; character++) {
            //save the current character to print
            uint8_t characterToPrint = charIndexes[character];

            //copy each byte from the Verdana BMP in the buffer
            for(uint8_t column = 0; column < (uint8_t)VERDANA_CHAR_WIDTH; column++) {
                bytesToUpdate[(character * VERDANA_CHAR_WIDTH) + column] =
                    verdana_16ptNumbers[characterToPrint][page][column];
            }
        }
    }

    //invalidate the screen and exit
    screenInvalidated = 1;
    return (ERR_SUCCESS);
}

/**
 * @brief Draw the icon representing the type of referential currently used
 * @note This function invalidates the screen
 * 
 * @param type Referential type
 * @return Success
 * @retval 1 Screen busy
 */
errorCode_u ssd1306PrintReferentialIcon(referentialType_e type) {
    uint8_t*       iterator     = &screenBuffer[REFICON_PAGE][REFICON_COLUMN];
    const uint8_t* iconIterator = (type == ABSOLUTE ? absoluteReferentialIcon : relativeReferentialIcon);

    //if screen busy, error
    if(!isScreenReady()) {
        return (createErrorCode(PRT_REFICON, 1, ERR_WARNING));
    }

    for(uint8_t i = 0; i < (uint8_t)REFERENCETYPE_NB_BYTES; i++) {
        *(iterator++) = *(iconIterator++);
    }

    //invalidate the screen and exit
    screenInvalidated = 1;
    return (ERR_SUCCESS);
}

/**
 * @brief Draw/erase the icon representing the hold function
 * @note This function invalidates the screen
 * 
 * @param status 1 to print, 0 to erase
 * @return Success
 * @retval 1 Screen busy
 */
errorCode_u ssd1306PrintHoldIcon(uint8_t status) {
    uint8_t*       iterator     = &screenBuffer[HOLDICON_PAGE][HOLDICON_COLUMN];
    const uint8_t* iconIterator = holdIcon;

    //if screen busy, error
    if(!isScreenReady()) {
        return (createErrorCode(PRT_HOLDICON, 1, ERR_WARNING));
    }

    for(uint8_t i = 0; i < (uint8_t)REFERENCETYPE_NB_BYTES; i++) {
        if(status) {
            *(iterator++) = *(iconIterator++);
        } else {
            *(iterator++) = 0x00;
        }
    }

    //invalidate the screen and exit
    screenInvalidated = 1;
    return (ERR_SUCCESS);
}

/**
 * @brief Turn the screen OFF
 * 
 * @return Return code of the send command instruction
 */
errorCode_u ssd1306TurnDisplayOFF() {
    return (sendCommand(DISPLAY_OFF, (void*)0, 0));
}

/**
 * @brief Run the state machine
 *
 * @return Return code of the current state
 */
errorCode_u ssd1306Update() {
    return ((*state)());
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief State in which the SSD1306 configuration registers are set
 * 
 * @return Success
 * @retval 1	Error while setting a configuration register
 * @retval 2	Error while setting the update window width
 * @retval 3	Error while setting the update window height
 * @retval 4	Error while requesting the screen wipe
 */
static errorCode_u stateConfiguring() {
#define NB_INIT_REGISERS 8U  ///< Number of registers set at initialisation
    const uint8_t limitColumns[2]                   = {0, (SSD_SCREEN_WIDTH - 1)};
    const uint8_t limitPages[2]                     = {0, (SSD_NB_PAGES - 1)};
    const uint8_t initCommands[NB_INIT_REGISERS][3] = {
        ///< Array used to initialise the registers
        {SCAN_DIRECTION_N1_0, 0,                                       0x00},
        {    HARDWARE_CONFIG, 1, SSD_PIN_CONFIG_ALT | SSD_COM_REMAP_DISABLE},
        {  SEGMENT_REMAP_127, 0,                                       0x00},
        {   MEMORY_ADDR_MODE, 1,                        SSD_HORIZONTAL_ADDR},
        {   CONTRAST_CONTROL, 1,                       SSD_CONTRAST_HIGHEST},
        { CLOCK_DIVIDE_RATIO, 1,   SSD_CLOCK_FREQ_MID | SSD_CLOCK_DIVIDER_1},
        { CHG_PUMP_REGULATOR, 1,                        SSD_ENABLE_CHG_PUMP},
        {         DISPLAY_ON, 0,                                       0x00},
    };
    errorCode_u result;

    //reset the chip
    LL_GPIO_ResetOutputPin(SSD1306_RES_GPIO_Port, SSD1306_RES_Pin);
    LL_GPIO_SetOutputPin(SSD1306_RES_GPIO_Port, SSD1306_RES_Pin);

    //initialisation taken from PDF p. 64 (Application Example)
    //	values which don't change from reset values aren't modified
    //TODO test for max oscillator frequency
    for(uint8_t i = 0; i < NB_INIT_REGISERS; i++) {
        result = sendCommand(initCommands[i][0], &initCommands[i][2], initCommands[i][1]);
        if(isError(result)) {
            return (pushErrorCode(result, INIT, 1));
        }
    }

    //screen updates will be done on the whole screen width
    result = sendCommand(COLUMN_ADDRESS, limitColumns, 2);
    if(isError(result)) {
        return (pushErrorCode(result, INIT, 2));
    }

    //screen updates will be done on the whole screen height
    result = sendCommand(PAGE_ADDRESS, limitPages, 2);
    if(isError(result)) {
        return (pushErrorCode(result, INIT, 3));
    }

    result = drawBaseScreen();
    if(isError(result)) {
        screenInvalidated = 0;
        return (pushErrorCode(result, INIT, 4));
    }

    //get to idle state
    state = stateIdle;
    return (ERR_SUCCESS);
}

/**
 * @brief State in which the screen awaits for commands
 *
 * @return Success
 */
errorCode_u stateIdle() {
    if(screenInvalidated) {
        screenInvalidated = 0;
        state             = stateSendingData;
    }

    return (ERR_SUCCESS);
}

/**
 * @brief State in which data is sent to the screen
 *
 * @return Success
 */
errorCode_u stateSendingData() {
    const uint16_t MAX_DATA = (SSD_SCREEN_WIDTH * SSD_NB_PAGES);  ///< Maximum data to send the screen

    //set data GPIO and enable SPI
    setDataCommandGPIO(DATA);
    LL_SPI_Enable(spiHandle);

    //configure the DMA transaction
    LL_DMA_DisableChannel(dmaHandle, dmaChannelUsed);
    LL_DMA_ClearFlag_GI5(dmaHandle);
    LL_DMA_SetDataLength(dmaHandle, dmaChannelUsed, MAX_DATA);  //must be reset every time
    LL_DMA_EnableChannel(dmaHandle, dmaChannelUsed);

    //send the data
    TXtick = getSystick();
    LL_SPI_EnableDMAReq_TX(spiHandle);

    //get to next
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
errorCode_u stateWaitingForTXdone() {
    errorCode_u result = ERR_SUCCESS;

    //if timer elapsed, stop DMA and error
    if(isTimeElapsed(TXtick, SPI_TIMEOUT_MS)) {
        result = createErrorCode(WAITING_DMA_RDY, 1, ERR_ERROR);
        goto finalise;
    }

    //if DMA error, error
    if(LL_DMA_IsActiveFlag_TE5(dmaHandle)) {
        result = createErrorCode(WAITING_DMA_RDY, 2, ERR_ERROR);
        goto finalise;
    }

    //if transmission not complete yet, exit
    if(!LL_DMA_IsActiveFlag_TC5(dmaHandle)) {
        return (ERR_SUCCESS);
    }

finalise:
    LL_DMA_DisableChannel(dmaHandle, dmaChannelUsed);
    LL_SPI_Disable(spiHandle);
    state = stateIdle;
    return result;
}
