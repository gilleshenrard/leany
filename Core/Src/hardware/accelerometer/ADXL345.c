/**
 * @brief Implement the ADXL345 accelerometer communication
 * @author Gilles Henrard
 * @date 25/10/2023
 *
 * @note Additional information can be found in :
 *   - ADXL345 datasheet : https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
 *   - AN-1025 (FIFO application note) document : https://www.analog.com/media/en/technical-documentation/application-notes/AN-1025.pdf
 *
 */
#include "ADXL345.h"
#include "ADXL345registers.h"
#include "main.h"
#include <math.h>

//definitions
#define ADXL_TIMEOUT_MS		10U		///< SPI direct transmission timeout span in milliseconds
#define ADXL_BYTE_OFFSET	8U		///< Number of bits to offset a byte
#define ADXL_X_INDEX_MSB	1U		///< Index of the X MSB in the measurements
#define ADXL_X_INDEX_LSB	0U		///< Index of the X LSB in the measurements
#define ADXL_Y_INDEX_MSB	3U		///< Index of the Y MSB in the measurements
#define ADXL_Y_INDEX_LSB	2U		///< Index of the Y LSB in the measurements
#define ADXL_Z_INDEX_MSB	5U		///< Index of the Z MSB in the measurements
#define ADXL_Z_INDEX_LSB	4U		///< Index of the Z LSB in the measurements

//macros
#define ENABLE_SPI		HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET);	///< Macro used to enable the SPI communication towards the accelerometer
#define DISABLE_SPI		HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);	///< Macro used to disable the SPI communication towards the accelerometer
#define DIVIDE_16(val)	val >>= 4;															///< Macro used to divide a number by 16 and store it
#define RAD_TO_DEG(val)	(val * 180.0f) / (float)M_PI										///< Macro used to transform angles from radians to degrees

//type definitions
/**
 * @brief Enumeration of the function IDs of the ADXL345
 */
typedef enum _ADXLfunctionCodes_e{
	INIT = 0,      		///< ADXL345initialise()
	UPDATE,        		///< ADXL345update()
	CHK_MEASURES,  		///< ADXL345hasNewMeasurements()
	READ_REGISTER,		///< ADXL345readRegister()
	WRITE_REGISTER,		///< ADXL345writeRegister()
	READ_REGISTERS,		///< ADXL345readRegisters()
	GET_X_ANGLE,		///< ADXL345getXangleDegrees()
	GET_Y_ANGLE			///< ADXL345getYangleDegrees()
}ADXLfunctionCodes_e;

typedef errorCode_u (*adxlState)();

//machine state
static errorCode_u stStartup();
static errorCode_u stConfiguration();
static errorCode_u stMeasuring();

//manipulation functions
//static errorCode_u ADXL345readRegister(adxl345Registers_e registerNumber, uint8_t* value);
static errorCode_u ADXL345writeRegister(adxl345Registers_e registerNumber, uint8_t value);
static errorCode_u ADXL345readRegisters(adxl345Registers_e firstRegister, uint8_t* value, uint8_t size);

//global variables
SPI_HandleTypeDef*	ADXL_spiHandle = NULL;			///< SPI handle used with the ADXL345
adxlState			state = stStartup;				///< State machine current state
volatile uint8_t	adxlINT1occurred = 0;			///< Flag used to indicate the ADXL triggered an interrupt
uint8_t				adxlMeasurementsUpdated = 0;	///< Flag used to indicate new integrated measurements are ready within the ADXL345
uint8_t				buffer[ADXL_NB_DATA_REGISTERS];	///< Buffer used to pop 1 measurement from each ADXL FIFO
int16_t				finalX;							///< X value obtained after integration
int16_t				finalY;							///< Y value obtained after integration
int16_t				finalZ;							///< Z value obtained after integration


/********************************************************************************************************************************************/
/********************************************************************************************************************************************/


/**
 * @brief Initialise the ADXL345
 *
 * @param handle SPI handle used
 * @retval 0 Success
 * @retval 1 Error while setting bandwidth and power
 * @retval 2 Error while setting data format
 * @retval 3 Error while clearing FIFO
 * @retval 4 Error while setting FIFO mode
 * @retval 5 Error while setting interrupts
 * @retval 6 Error while setting measurement mode
 */
errorCode_u ADXL345initialise(const SPI_HandleTypeDef* handle){
	ADXL_spiHandle = (SPI_HandleTypeDef*)handle;
	return (ERR_SUCCESS);
}

/**
 * @brief Update the ADXL345 measurements
 *
 * @retval 0 Success
 * @retval 1 Error occurred while reading the axis values registers
 */
errorCode_u ADXL345update(){
	return ( (*state)() );
}

/**
 * @brief Check if new measurements have been updated
 *
 * @retval 0 No new values available
 * @retval 1 New values are available
 */
uint8_t ADXL345hasNewMeasurements(){
	uint8_t tmp = adxlMeasurementsUpdated;
	adxlMeasurementsUpdated = 0;

	return (tmp);
}

/**
 * @brief Read a single register on the ADXL345
 *
 * @param registerNumber Register number
 * @param[out] value Register value
 * @retval 0 Success
 * @retval 1 No SPI handle set
 * @retval 2 Register number out of range
 * @retval 3 Attempted to access a reserved register
 * @retval 4 Error while writing the command
 * @retval 5 Error while reading the value
 */
/*
errorCode_u ADXL345readRegister(adxl345Registers_e registerNumber, uint8_t* value){
	HAL_StatusTypeDef HALresult;
	errorCode_u result = { .dword = 0};
	uint8_t instruction = ADXL_READ | ADXL_SINGLE | registerNumber;

	//if handle not set, error
	if(ADXL_spiHandle == NULL)
		return (errorCode(result, READ_REGISTER, 1));

	//if register number above known, error
	if(registerNumber > ADXL_NB_REGISTERS)
		return (errorCode(result, READ_REGISTER, 2));

	//if register number between 0x01 and 0x1C included, error
	if((uint8_t)(registerNumber - 1) < ADXL_HIGH_RESERVED_REG)
		return (errorCode(result, READ_REGISTER, 3)); 	// @suppress("Avoid magic numbers")

	ENABLE_SPI

	//transmit the read instruction
	HALresult = HAL_SPI_Transmit(ADXL_spiHandle, &instruction, 1, ADXL_TIMEOUT_MS);
	if(HALresult != HAL_OK){
		DISABLE_SPI
		return (errorCodeLayer0(READ_REGISTER, 4, HALresult)); 	// @suppress("Avoid magic numbers")
	}

	//receive the reply
	HALresult = HAL_SPI_Receive(ADXL_spiHandle, value, 1, ADXL_TIMEOUT_MS);
	if(HALresult != HAL_OK)
		return (errorCodeLayer0(READ_REGISTER, 5, HALresult)); 	// @suppress("Avoid magic numbers")

	DISABLE_SPI

	return (result);
}
*/

/**
 * @brief Write a single register on the ADXL345
 *
 * @param registerNumber Register number
 * @param value Register value
 * @retval 0 Success
 * @retval 1 No SPI handle set
 * @retval 2 Register number out of range
 * @retval 3 Attempted to access a reserved register
 * @retval 4 Error while writing the command
 * @retval 5 Error while writing the value
 */
errorCode_u ADXL345writeRegister(adxl345Registers_e registerNumber, uint8_t value){
	HAL_StatusTypeDef HALresult;
	errorCode_u result = { .dword = 0};
	uint8_t instruction = ADXL_WRITE | ADXL_SINGLE | registerNumber;

	//if handle not set, error
	if(ADXL_spiHandle == NULL)
		return (errorCode(result, WRITE_REGISTER, 1));

	//if register number above known, error
	if(registerNumber > ADXL_NB_REGISTERS)
		return (errorCode(result, WRITE_REGISTER, 2));

	//if register number between 0x01 and 0x1C included, error
	if((uint8_t)(registerNumber - 1) < ADXL_HIGH_RESERVED_REG)
		return (errorCode(result, WRITE_REGISTER, 3)); 	// @suppress("Avoid magic numbers")

	ENABLE_SPI

	//transmit the read instruction
	HALresult = HAL_SPI_Transmit(ADXL_spiHandle, &instruction, 1, ADXL_TIMEOUT_MS);
	if(HALresult != HAL_OK){
		DISABLE_SPI
		return (errorCodeLayer0(WRITE_REGISTER, 4, HALresult)); 	// @suppress("Avoid magic numbers")
	}

	//receive the reply
	HALresult = HAL_SPI_Transmit(ADXL_spiHandle, &value, 1, ADXL_TIMEOUT_MS);
	if(HALresult != HAL_OK)
		result = errorCodeLayer0(WRITE_REGISTER, 5, HALresult); 	// @suppress("Avoid magic numbers")

	DISABLE_SPI
	return (result);
}

/**
 * @brief Read several registers on the ADXL345
 *
 * @param firstRegister Number of the first register to read
 * @param[out] value Registers value array
 * @param size Number of registers to read
 * @retval 0 Success
 * @retval 1 No SPI handle set
 * @retval 2 Register number out of range
 * @retval 3 Error while writing the command
 * @retval 4 Error while reading the values
 */
errorCode_u ADXL345readRegisters(adxl345Registers_e firstRegister, uint8_t* value, uint8_t size){
	HAL_StatusTypeDef HALresult;
	errorCode_u result = { .dword = 0};
	uint8_t instruction = ADXL_READ | ADXL_MULTIPLE | firstRegister;

	//if handle not set, error
	if(ADXL_spiHandle == NULL)
		return (errorCode(result, READ_REGISTERS, 1));

	//if register numbers above known, error
	if(firstRegister > ADXL_NB_REGISTERS)
		return (errorCode(result, READ_REGISTERS, 2));

	ENABLE_SPI

	//transmit the read instruction
	HALresult = HAL_SPI_Transmit(ADXL_spiHandle, &instruction, 1, ADXL_TIMEOUT_MS);
	if(HALresult != HAL_OK){
		DISABLE_SPI
		return (errorCodeLayer0(READ_REGISTERS, 3, HALresult)); 	// @suppress("Avoid magic numbers")
	}

	//receive the reply
	HALresult = HAL_SPI_Receive(ADXL_spiHandle, value, size, ADXL_TIMEOUT_MS);
	if(HALresult != HAL_OK)
		result = errorCodeLayer0(READ_REGISTERS, 4, HALresult); 	// @suppress("Avoid magic numbers")

	DISABLE_SPI
	return (result);
}

/**
 * @brief Get the latest measured angle between the X axis and the Z axis
 *
 * @return Angle in degrees
 */
float ADXL345getXangleDegrees(){
	return (RAD_TO_DEG(atanf((float)finalX / (float)finalZ)));
}

/**
 * @brief Get the latest measured angle between the Y axis and the Z axis
 *
 * @return Angle in degrees
 */
float ADXL345getYangleDegrees(){
	return (RAD_TO_DEG(atanf((float)finalY / (float)finalZ)));
}


/********************************************************************************************************************************************/
/********************************************************************************************************************************************/


/**
 * @brief Begin state of the state machine
 *
 * @return Success
 */
errorCode_u stStartup(){
	state = stConfiguration;
	return (ERR_SUCCESS);
}

/**
 * @brief State in which the registers of the ADXL are configured
 *
 * @return Success
 */
errorCode_u stConfiguration(){
	errorCode_u result;

	//configure bandwidth and power mode
	result = ADXL345writeRegister(BANDWIDTH_POWERMODE, ADXL_POWER_NORMAL | ADXL_RATE_100HZ);
	if(IS_ERROR(result))
		return (errorCode(result, INIT, 1));

	//configure data format
	result = ADXL345writeRegister(DATA_FORMAT, ADXL_SPI_4WIRE | ADXL_INT_ACTIV_LOW | ADXL_RANGE_2G);
	if(IS_ERROR(result))
		return (errorCode(result, INIT, 2));

	//clear the FIFO
	result = ADXL345writeRegister(FIFO_CONTROL, ADXL_MODE_BYPASS);
	if(IS_ERROR(result))
		return (errorCode(result, INIT, 3)); 	// @suppress("Avoid magic numbers")

	//set the FIFO mode and set 16 samples
	result = ADXL345writeRegister(FIFO_CONTROL, ADXL_MODE_FIFO | ADXL_TRIGGER_INT1 | ADXL_SAMPLES_16);
	if(IS_ERROR(result))
		return (errorCode(result, INIT, 4)); 	// @suppress("Avoid magic numbers")

	//trigger an interrupt when 16 measurements reached
	result = ADXL345writeRegister(INTERRUPT_ENABLE, ADXL_INT_WATERMARK);
	if(IS_ERROR(result))
		return (errorCode(result, INIT, 5)); 	// @suppress("Avoid magic numbers")

	//set the ADXL as in measurement mode (to be done last)
	result = ADXL345writeRegister(POWER_CONTROL, ADXL_MEASURE_MODE);
	if(IS_ERROR(result))
		return (errorCode(result, INIT, 6)); 	// @suppress("Avoid magic numbers")

	state = stMeasuring;
	return (result);
}

/**
 * @brief State in which the ADXL measures accelerations
 *
 * @return Success
 */
static errorCode_u stMeasuring(){
	errorCode_u result = { .dword = 0};

	//if watermark interrupt not fired, exit
	if(!adxlINT1occurred)
		return (ERR_SUCCESS);

	adxlINT1occurred = 0;
	finalX = finalY = finalZ = 0;

	//for eatch of the 16 samples to read
	for(uint8_t i = 0 ; i < ADXL_SAMPLES_16 ; i++){
		//read all data registers for 1 sample
		result = ADXL345readRegisters(DATA_X0, buffer, ADXL_NB_DATA_REGISTERS);
		if(IS_ERROR(result))
			return (errorCode(result, UPDATE, 1));

		//add the measurements (formatted from a two's complement) to their final value buffer
		finalX += (int16_t)(((uint16_t)(buffer[ADXL_X_INDEX_MSB]) << ADXL_BYTE_OFFSET) | (uint16_t)(buffer[ADXL_X_INDEX_LSB]));
		finalY += (int16_t)(((uint16_t)(buffer[ADXL_Y_INDEX_MSB]) << ADXL_BYTE_OFFSET) | (uint16_t)(buffer[ADXL_Y_INDEX_LSB]));
		finalZ += (int16_t)(((uint16_t)(buffer[ADXL_Z_INDEX_MSB]) << ADXL_BYTE_OFFSET) | (uint16_t)(buffer[ADXL_Z_INDEX_LSB]));
	}

	//divide the buffers by 16
	DIVIDE_16(finalX);
	DIVIDE_16(finalY);
	DIVIDE_16(finalZ);

	adxlMeasurementsUpdated = 1;
	return (ERR_SUCCESS);
}
