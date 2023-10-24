/**
 * @brief Implement the ADXL345 accelerometer communication
 * @author Gilles Henrard
 * @date 22/10/2023
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

#define ADXL_TIMEOUT_MS		10U		///< SPI direct transmission timeout span in milliseconds
#define ADXL_BYTE_OFFSET	8U		///< Number of bits to offset a byte
#define ADXL_X_INDEX_MSB	1U		///< Index of the X MSB in the measurements
#define ADXL_X_INDEX_LSB	0U		///< Index of the X LSB in the measurements
#define ADXL_Y_INDEX_MSB	3U		///< Index of the Y MSB in the measurements
#define ADXL_Y_INDEX_LSB	2U		///< Index of the Y LSB in the measurements
#define ADXL_Z_INDEX_MSB	5U		///< Index of the Z MSB in the measurements
#define ADXL_Z_INDEX_LSB	4U		///< Index of the Z LSB in the measurements

#define ENABLE_SPI		HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET);	///< Macro used to enable the SPI communication towards the accelerometer
#define DISABLE_SPI		HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);	///< Macro used to disable the SPI communication towards the accelerometer
#define DIVIDE_16(val)	val >>= 4;															///< Macro used to divide a number by 16 and store it
#define RAD_TO_DEG(val)	(val * 180.0f) / (float)M_PI										///< Macro used to transform angles from radians to degrees

//static HAL_StatusTypeDef ADXL345readRegister(adxl345Registers_e registerNumber, uint8_t* value);
static HAL_StatusTypeDef ADXL345writeRegister(adxl345Registers_e registerNumber, uint8_t value);
static HAL_StatusTypeDef ADXL345readRegisters(adxl345Registers_e firstRegister, uint8_t* value, uint8_t size);

SPI_HandleTypeDef* ADXL_spiHandle = NULL;	///< SPI handle used with the ADXL345
volatile uint8_t adxlINT1occurred = 0;		///< Flag used to indicate the ADXL triggered an interrupt
uint8_t adxlMeasurementsUpdated = 0;		///< Flag used to indicate new integrated measurements are ready within the ADXL345
uint8_t buffer[ADXL_NB_DATA_REGISTERS];		///< Buffer used to pop 1 measurement from each ADXL FIFO
int16_t finalX;								///< X value obtained after integration
int16_t finalY;								///< Y value obtained after integration
int16_t finalZ;								///< Z value obtained after integration

/**
 * @brief Initialise the ADXL345
 *
 * @param[in] handle SPI handle used
 */
HAL_StatusTypeDef ADXL345initialise(const SPI_HandleTypeDef* handle){
	ADXL_spiHandle = (SPI_HandleTypeDef*)handle;
	HAL_StatusTypeDef result;

	//configure bandwidth and power mode
	result = ADXL345writeRegister(BANDWIDTH_POWERMODE, ADXL_POWER_NORMAL | ADXL_RATE_100HZ);
	if(result != HAL_OK)
		return (result);

	//configure data format
	result = ADXL345writeRegister(DATA_FORMAT, ADXL_SPI_4WIRE | ADXL_INT_ACTIV_LOW | ADXL_RANGE_2G);
	if(result != HAL_OK)
		return (result);

	//clear the FIFO
	result = ADXL345writeRegister(FIFO_CONTROL, ADXL_MODE_BYPASS);
	if(result != HAL_OK)
		return (result);

	//set the FIFO mode and set 16 samples
	result = ADXL345writeRegister(FIFO_CONTROL, ADXL_MODE_FIFO | ADXL_TRIGGER_INT1 | ADXL_SAMPLES_16);
	if(result != HAL_OK)
		return (result);

	//trigger an interrupt when 16 measurements reached
	result = ADXL345writeRegister(INTERRUPT_ENABLE, ADXL_INT_WATERMARK);
	if(result != HAL_OK)
		return (result);

	//set the ADXL as in measurement mode (to be done last)
	result = ADXL345writeRegister(POWER_CONTROL, ADXL_MEASURE_MODE);
	if(result != HAL_OK)
		return (result);

	return (result);
}

/**
 * @brief Update the ADXL345 measurements
 *
 * @return 0
 */
uint16_t ADXL345update(){
	//if watermark interrupt not fired, exit
	if(!adxlINT1occurred)
		return (0);

	adxlINT1occurred = 0;
	finalX = finalY = finalZ = 0;

	//for eatch of the 16 samples to read
	for(uint8_t i = 0 ; i < ADXL_SAMPLES_16 ; i++){
		//read all data registers for 1 sample
		ADXL345readRegisters(DATA_X0, buffer, ADXL_NB_DATA_REGISTERS);

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
	return (0);
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
 * @param[in] registerNumber Register number
 * @param[out] value Register value
 * @return Return value of SPI transmissions
 */
/*
HAL_StatusTypeDef ADXL345readRegister(adxl345Registers_e registerNumber, uint8_t* value){
	HAL_StatusTypeDef result;
	uint8_t instruction = ADXL_READ | ADXL_SINGLE | registerNumber;

	//if handle not set, error
	if(ADXL_spiHandle == NULL)
		return (HAL_ERROR);

	//if register number above known, error
	if(registerNumber > ADXL_NB_REGISTERS)
		return (HAL_ERROR);

	//if register number between 0x01 and 0x1C included, error
	if((uint8_t)(registerNumber - 1) < ADXL_HIGH_RESERVED_REG)
		return (HAL_ERROR);

	//transmit the read instruction and receive the reply
	ENABLE_SPI
	result = HAL_SPI_Transmit(ADXL_spiHandle, &instruction, 1, ADXL_TIMEOUT_MS);
	if(result == HAL_OK)
		result = HAL_SPI_Receive(ADXL_spiHandle, value, 1, ADXL_TIMEOUT_MS);
	DISABLE_SPI

	return (result);
}
*/

/**
 * @brief Write a single register on the ADXL345
 *
 * @param[in] registerNumber Register number
 * @param[in] value Register value
 * @return Return value of SPI transmissions
 */
HAL_StatusTypeDef ADXL345writeRegister(adxl345Registers_e registerNumber, uint8_t value){
	HAL_StatusTypeDef result;
	uint8_t instruction = ADXL_WRITE | ADXL_SINGLE | registerNumber;

	//if handle not set, error
	if(ADXL_spiHandle == NULL)
		return (HAL_ERROR);

	//if register number above known, error
	if(registerNumber > ADXL_NB_REGISTERS)
		return (HAL_ERROR);

	//if register number between 0x01 and 0x1C included, error
	if((uint8_t)(registerNumber - 1) < ADXL_HIGH_RESERVED_REG)
		return (HAL_ERROR);

	//transmit the read instruction and receive the reply
	ENABLE_SPI
	result = HAL_SPI_Transmit(ADXL_spiHandle, &instruction, 1, ADXL_TIMEOUT_MS);
	if(result == HAL_OK)
		result = HAL_SPI_Transmit(ADXL_spiHandle, &value, 1, ADXL_TIMEOUT_MS);
	DISABLE_SPI

	return (result);
}

/**
 * @brief Read several registers on the ADXL345
 *
 * @param[in] firstRegister Number of the first register to read
 * @param[out] value Registers value array
 * @param[in] size Number of registers to read
 * @return Return value of SPI transmissions
 */
HAL_StatusTypeDef ADXL345readRegisters(adxl345Registers_e firstRegister, uint8_t* value, uint8_t size){
	HAL_StatusTypeDef result;
	uint8_t instruction = ADXL_READ | ADXL_MULTIPLE | firstRegister;

	//if handle not set, error
	if(ADXL_spiHandle == NULL)
		return (HAL_ERROR);

	//if register numbers above known, error
	if(firstRegister > ADXL_NB_REGISTERS)
		return (HAL_ERROR);

	//transmit the read instruction and receive the reply
	ENABLE_SPI
	result = HAL_SPI_Transmit(ADXL_spiHandle, &instruction, 1, ADXL_TIMEOUT_MS);
	if(result == HAL_OK)
		result = HAL_SPI_Receive(ADXL_spiHandle, value, size, ADXL_TIMEOUT_MS);
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
