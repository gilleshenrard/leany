/**
 * @file SSD1306.c
 * @brief Implement the functioning of the SSD1306 OLED screen via SPI and DMA
 * @author Gilles Henrard
 * @date 04/05/2024
 *
 * @note Datasheet : https://cdn-shop.adafruit.com/datasheets/SSD1306.pdf
 */
#include "SSD1306.h"
#include <assert.h>
#include "SSD1306_registers.h"
#include "icons.h"
#include "numbersVerdana16.h"
#include "stdbool.h"

enum {
    SSD_LAST_COLUMN = 127U,   ///< Index of the highest column
    SSD_LAST_PAGE   = 31U,    ///< Index of the highest page
    MAX_DATA_SIZE   = 1024U,  ///< Maximum SSD1306 data size (128 * 64 pixels / 8 pixels per byte)
    ANGLE_NB_CHARS  = 6U,     ///< Number of characters in the angle array
};

//definitions
#define REFTYPE_PAGE   SSD_LAST_PAGE
#define REFTYPE_COLUMN (SSD_LAST_COLUMN - REFERENCETYPE_NB_BYTES)
static const uint8_t SPI_TIMEOUT_MS = 10U;  ///< Maximum number of milliseconds SPI traffic should last before timeout

//static assertions (ran at compile time)
_Static_assert((bool)((ANGLE_NB_CHARS * VERDANA_NB_BYTES_CHAR) <= MAX_DATA_SIZE), "SSD1306 font chosen uses too much space.");
_Static_assert((bool)((ANGLE_NB_CHARS * VERDANA_CHAR_WIDTH) <= (SSD_LAST_COLUMN + 1)),
               "SSD1306 font chosen has too many columns.");

/**
 * @brief Enumeration of the function IDs of the SSD1306
 */
typedef enum {
    INIT = 0,        ///< SSD1306initialise()
    SEND_CMD,        ///< SSD1306sendCommand()
    PRT_ANGLE,       ///< SSD1306_printAngleTenths()
    SENDING_DATA,    ///< stateSendingData()
    WAITING_DMA_RDY  ///< stateWaitingForTXdone()
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

//state machine
static errorCode_u stateIdle();
static errorCode_u stateConfiguring();
static errorCode_u stateSendingData();
static errorCode_u stateWaitingForTXdone();

//state variables
volatile uint16_t   screenTimer_ms     = 0;                 ///< Timer used with screen SPI transmissions (in ms)
volatile uint16_t   ssd1306SPITimer_ms = 0;                 ///< Timer used to make sure SPI does not time out (in ms)
static SPI_TypeDef* spiHandle          = (void*)0;          ///< SPI handle used with the SSD1306
static DMA_TypeDef* dmaHandle          = (void*)0;          ///< DMA handle used with the SSD1306
static uint32_t     dmaChannel         = 0x00000000U;       ///< DMA channel used
static screenState  state              = stateConfiguring;  ///< State machine current state
static uint8_t      screenBuffer[MAX_DATA_SIZE];            ///< Buffer used to send data to the screen
static uint8_t      limitColumns[2];                        ///< Buffer used to set the first and last column to send
static uint8_t      limitPages[2];                          ///< Buffer used to set the first and last page to send
static uint16_t     size;                                   ///< Number of bytes to send

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief Initialise the SSD1306
 *
 * @param handle SPI handle used
 * @return Success
 * @retval 1	Error while initialising the registers
 * @retval 2	Error while clearing the screen
 */
errorCode_u SSD1306initialise(SPI_TypeDef* handle, DMA_TypeDef* dma, uint32_t dmaChannel) {
    spiHandle  = handle;
    dmaHandle  = dma;
    dmaChannel = dmaChannel;

    //make sure to disable SSD1306 SPI communication
    LL_SPI_Disable(spiHandle);
    LL_DMA_DisableChannel(dmaHandle, dmaChannel);

    //set the DMA source and destination addresses (will always use the same ones)
    LL_DMA_ConfigAddresses(dmaHandle, dmaChannel, (uint32_t)&screenBuffer, LL_SPI_DMA_GetRegAddr(spiHandle),
                           LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

    return (ERR_SUCCESS);
}

/**
 * brief Set the Data/Command pin
 *
 * @param function Value of the data/command pin
 */
static inline void setDataCommandGPIO(DCgpio_e function) {
    if(function == COMMAND)
        LL_GPIO_ResetOutputPin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin);
    else
        LL_GPIO_SetOutputPin(SSD1306_DC_GPIO_Port, SSD1306_DC_Pin);
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
    static const uint8_t MAX_PARAMETERS = 6U;  ///< Maximum number of parameters a command can have
    errorCode_u          result         = ERR_SUCCESS;

    //assertions
    assert(spiHandle);                    //handle is not null
    assert(parameters || !nbParameters);  //either 0 parameters, or parameters array not null

    //if too many parameters, error
    if(nbParameters > MAX_PARAMETERS)
        return (createErrorCode(SEND_CMD, 1, ERR_WARNING));

    //set command pin and enable SPI
    ssd1306SPITimer_ms = SPI_TIMEOUT_MS;
    setDataCommandGPIO(COMMAND);
    LL_SPI_Enable(spiHandle);

    //send the command byte
    LL_SPI_TransmitData8(spiHandle, regNumber);

    //send the parameters
    uint8_t* iterator = (uint8_t*)parameters;
    while(nbParameters && ssd1306SPITimer_ms) {
        //wait for the previous byte to be done, then send the next one
        while(!LL_SPI_IsActiveFlag_TXE(spiHandle) && ssd1306SPITimer_ms);
        if(ssd1306SPITimer_ms)
            LL_SPI_TransmitData8(spiHandle, *iterator);

        iterator++;
        nbParameters--;
    }

    //wait for transaction to be finished and clear Overrun flag
    while(LL_SPI_IsActiveFlag_BSY(spiHandle) && ssd1306SPITimer_ms);
    LL_SPI_ClearFlag_OVR(spiHandle);

    //disable SPI and return status
    LL_SPI_Disable(spiHandle);

    //if timeout, error
    if(!ssd1306SPITimer_ms)
        return (createErrorCode(SEND_CMD, 2, ERR_WARNING));

    return (result);
}

/**
 * @brief Wipe the screen blank and draw the separator and icons
 *
 * @return Success
 */
errorCode_u SSD1306drawBaseScreen() {
    uint8_t* iterator = screenBuffer;
    uint16_t counter  = 0;

    //define the whole screen as the drawing window
    limitColumns[0] = 0;
    limitColumns[1] = SSD_LAST_COLUMN;
    limitPages[0]   = 0;
    limitPages[1]   = SSD_LAST_PAGE;
    size            = MAX_DATA_SIZE;

    //fill the screen buffer with blank pixels value
    for(counter = 0; counter < (uint16_t)MAX_DATA_SIZE; counter++) *(iterator++) = 0x00U;

    //draw the middle screen separator in the buffer (avoid drawing in the arrows icon zone)
    static const uint8_t LIGN = 0x03U;
    iterator                  = &screenBuffer[((uint16_t)MAX_DATA_SIZE >> 1U) + ARROWSICON_WIDTH];
    for(counter = ARROWSICON_WIDTH; counter <= (uint16_t)SSD_LAST_COLUMN; counter++) *(iterator++) = LIGN;

    //draw the arrows icon
    for(counter = 0; counter < ARROWSICON_NB_BYTES; counter++) {
        //copy a byte, while making sure to copy it at the proper page number
        uint16_t bufferOffset                                     = (counter / ARROWSICON_WIDTH) * (SSD_LAST_COLUMN + 1);
        screenBuffer[bufferOffset + (counter % ARROWSICON_WIDTH)] = arrowsIcon_32px[counter];
    }

    //draw the absolute referential icon
    iterator = &screenBuffer[MAX_DATA_SIZE - REFERENCETYPE_NB_BYTES];
    for(counter = 0; counter < REFERENCETYPE_NB_BYTES; counter++) *(iterator++) = absoluteReferentialIcon[counter];

    state = stateSendingData;
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
 *
 * @param angleTenths	Angle to print
 * @param rotationAxis  Axis around which the rotation angle is to print
 *
 * @return Success
 * @retval 1	Angle above maximum amplitude
 */
errorCode_u SSD1306_printAngleTenths(int16_t angleTenths, rotationAxis_e rotationAxis) {
    static const uint8_t ANGLE_COLUMN                = 40U;   ///< Column number of the first screen line
    static const uint8_t ANGLE_ROLL_PAGE             = 1U;    ///< Number of the page at which display the roll axis angle
    static const uint8_t ANGLE_PITCH_PAGE            = 5U;    ///< Number of the page at which display the pitch axis angle
    static const int16_t MIN_ANGLE_DEG_TENTHS        = -900;  ///< Minimum angle allowed (in tenths of degrees)
    static const int16_t MAX_ANGLE_DEG_TENTHS        = 900;   ///< Maximum angle allowed (in tenths of degrees)
    static const uint8_t INDEX_SIGN                  = 0;     ///< Index of the sign in the angle indexes array
    static const uint8_t INDEX_TENS                  = 1U;    ///< Index of the tens in the angle indexes array
    static const uint8_t INDEX_UNITS                 = 2U;    ///< Index of the units in the angle indexes array
    static const uint8_t INDEX_TENTHS                = 4U;    ///< Index of the tenths in the angle indexes array
    static const uint8_t DIVIDE_10                   = 10U;   ///< 10 Divider (used for warnings)
    static const uint8_t DIVIDE_100                  = 100U;  ///< 100 Divider (used for warnings)
    uint8_t              charIndexes[ANGLE_NB_CHARS] = {INDEX_PLUS, 0, 0, INDEX_DOT, 0, INDEX_DEG};
    uint8_t*             iterator                    = screenBuffer;

    //clamp the angle to print to the min value
    if(angleTenths < MIN_ANGLE_DEG_TENTHS)
        angleTenths = MIN_ANGLE_DEG_TENTHS;

    //clamp the angle to print to the max value
    if(angleTenths > MAX_ANGLE_DEG_TENTHS)
        angleTenths = MAX_ANGLE_DEG_TENTHS;

    //if angle negative, replace plus sign with minus sign
    if(angleTenths < 0) {
        charIndexes[INDEX_SIGN] = INDEX_MINUS;
        angleTenths             = (int16_t)-angleTenths;
    }

    //store the values
    limitColumns[0] = ANGLE_COLUMN;
    limitColumns[1] = ANGLE_COLUMN + (VERDANA_CHAR_WIDTH * ANGLE_NB_CHARS) - 1;
    limitPages[0]   = (rotationAxis == ROLL ? ANGLE_ROLL_PAGE : ANGLE_PITCH_PAGE);
    limitPages[1]   = (rotationAxis == ROLL ? ANGLE_ROLL_PAGE : ANGLE_PITCH_PAGE) + 1;
    size            = ANGLE_NB_CHARS * VERDANA_NB_BYTES_CHAR;

    //fill the angle characters indexes array with the float values (tens, units, tenths)
    charIndexes[INDEX_TENS]   = (uint8_t)(angleTenths / DIVIDE_100);
    charIndexes[INDEX_UNITS]  = (uint8_t)((angleTenths / DIVIDE_10) % DIVIDE_10);
    charIndexes[INDEX_TENTHS] = (uint8_t)(angleTenths % DIVIDE_10);

    //fill the buffer with all the required bitmaps bytes (column by column, then character by character, then page by page)
    for(uint8_t page = 0; page < VERDANA_NB_PAGES; page++) {
        for(uint8_t character = 0; character < ANGLE_NB_CHARS; character++) {
            for(uint8_t column = 0; column < VERDANA_CHAR_WIDTH; column++) {
                *iterator = verdana_16ptNumbers[charIndexes[character]][(VERDANA_CHAR_WIDTH * page) + column];
                iterator++;
            }
        }
    }

    //get to printing state
    state = stateSendingData;
    return (ERR_SUCCESS);
}

/**
 * @brief Draw the icon representing the type of referential currently used
 * 
 * @param type Referential type
 * @return Success
 */
errorCode_u SSD1306_printReferentialIcon(referentialType_e type) {
    uint8_t*       iterator     = screenBuffer;
    const uint8_t* iconIterator = (type == ABSOLUTE ? absoluteReferentialIcon : relativeReferentialIcon);

    limitColumns[0]             = REFTYPE_COLUMN;
    limitColumns[1]             = REFTYPE_COLUMN + REFERENCETYPE_NB_BYTES;
    limitPages[0]               = REFTYPE_PAGE;
    limitPages[1]               = REFTYPE_PAGE;
    size                        = REFERENCETYPE_NB_BYTES;

    for(uint8_t i = 0; i < REFERENCETYPE_NB_BYTES; i++) *(iterator++) = *(iconIterator++);

    //get to printing state
    state = stateSendingData;
    return (ERR_SUCCESS);
}

/**
 * @brief Draw/erase the icon representing the hold function
 * 
 * @param status 1 to print, 0 to erase
 * @return Success
 */
errorCode_u SSD1306_printHoldIcon(uint8_t status) {
    uint8_t*       iterator     = screenBuffer;
    const uint8_t* iconIterator = holdIcon;

    limitColumns[0]             = REFTYPE_COLUMN - REFERENCETYPE_NB_BYTES;
    limitColumns[1]             = REFTYPE_COLUMN;
    limitPages[0]               = REFTYPE_PAGE;
    limitPages[1]               = REFTYPE_PAGE;
    size                        = REFERENCETYPE_NB_BYTES;

    for(uint8_t i = 0; i < REFERENCETYPE_NB_BYTES; i++)
        if(status)
            *(iterator++) = *(iconIterator++);
        else
            *(iterator++) = 0x00;

    //get to printing state
    state = stateSendingData;
    return (ERR_SUCCESS);
}

/**
 * @brief Turn the screen OFF
 * 
 * @return Success
 */
errorCode_u SSD1306_turnDisplayOFF() {
    return (sendCommand(DISPLAY_OFF, (void*)0, 0));
}

/**
 * @brief Run the state machine
 *
 * @return Return code of the current state
 */
errorCode_u SSD1306update() {
    return ((*state)());
}

/********************************************************************************************************************************************/
/********************************************************************************************************************************************/

/**
 * @brief State in which the SSD1306 configuration registers are set
 * 
 * @return Success
 * @retval 1	Error while setting a configuration register
 * @retval 2	Error while requesting the screen wipe
 */
static errorCode_u stateConfiguring() {
#define NB_INIT_REGISERS 8U  ///< Number of registers set at initialisation
    static const uint8_t initCommands[NB_INIT_REGISERS][3] = {
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
        if(isError(result))
            return (pushErrorCode(result, INIT, 1));
    }

    result = SSD1306drawBaseScreen();
    if(isError(result))
        return (pushErrorCode(result, INIT, 2));

    state = stateSendingData;
    return (ERR_SUCCESS);
}

/**
 * @brief State in which the screen awaits for commands
 *
 * @return Success
 */
errorCode_u stateIdle() {
    return (ERR_SUCCESS);
}

/**
 * @brief State in which data is sent to the screen
 *
 * @return Success
 * @retval 1	Error occurred while sending the column address command
 * @retval 2	Error occurred while sending the page address command
 */
errorCode_u stateSendingData() {
    errorCode_u result;

    //send the set start and end column addresses
    result = sendCommand(COLUMN_ADDRESS, limitColumns, 2);
    if(isError(result)) {
        state = stateIdle;
        return (pushErrorCode(result, SENDING_DATA, 1));
    }

    //send the set start and end page addresses
    result = sendCommand(PAGE_ADDRESS, limitPages, 2);
    if(isError(result)) {
        state = stateIdle;
        return (pushErrorCode(result, SENDING_DATA, 2));
    }

    //set data GPIO and enable SPI
    setDataCommandGPIO(DATA);
    LL_SPI_Enable(spiHandle);

    //configure the DMA transaction
    LL_DMA_DisableChannel(dmaHandle, dmaChannel);
    LL_DMA_ClearFlag_GI5(dmaHandle);
    LL_DMA_SetDataLength(dmaHandle, dmaChannel, size);
    LL_DMA_EnableChannel(dmaHandle, dmaChannel);

    //send the data
    screenTimer_ms = SPI_TIMEOUT_MS;
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
    if(!screenTimer_ms) {
        result = createErrorCode(WAITING_DMA_RDY, 1, ERR_ERROR);
        goto finalise;
    }

    //if DMA error, error
    if(LL_DMA_IsActiveFlag_TE5(dmaHandle)) {
        result = createErrorCode(WAITING_DMA_RDY, 2, ERR_ERROR);
        goto finalise;
    }

    //if transmission not complete yet, exit
    if(!LL_DMA_IsActiveFlag_TC5(dmaHandle))
        return (ERR_SUCCESS);

finalise:
    LL_DMA_DisableChannel(dmaHandle, dmaChannel);
    LL_SPI_Disable(spiHandle);
    state = stateIdle;
    return result;
}
