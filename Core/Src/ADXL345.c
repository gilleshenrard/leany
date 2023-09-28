/**
 * @brief Implement the ADXL345 accelerometer communication
 * @author Gilles Henrard
 * @date 28/09/2023
 *
 */
#include "ADXL345.h"
#include "main.h"

#define ADXL_TIMEOUT_MS	10		///< SPI direct transmission timeout span in milliseconds

#define ADXL_WRITE		0x00	///< MSB configuration for write operations
#define ADXL_READ		0x80	///< MSB configuration for read operations
#define ADXL_SINGLE		0x00	///< Bit 6 configuration for single register operations
#define ADXL_MULTIPLE	0x40	///< Bit 6 configuration for multiple register operations

#define ENABLE_SPI	HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET);	///< Macro used to enable the SPI communication towards the accelerometer
#define DISABLE_SPI	HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);	///< Macro used to disable the SPI communication towards the accelerometer

SPI_HandleTypeDef* ADXL_spiHandle = NULL;	///< SPI handle used with the ADXL345

/**
 * @brief Initialise the ADXL345
 *
 * @param[in] handle SPI handle used
 */
void ADXL345initialise(const SPI_HandleTypeDef* handle){
	ADXL_spiHandle = (SPI_HandleTypeDef*)handle;
	DISABLE_SPI
}

/**
 * @brief Read a single register on the ADXL345
 *
 * @param[in] registerNumber Register number
 * @param[out] value Register value
 * @return Return value of SPI transmissions
 */
HAL_StatusTypeDef ADXL345readRegister(adxl345Registers_e registerNumber, uint8_t* value){
	HAL_StatusTypeDef result;
	uint8_t tmp = ADXL_READ | ADXL_SINGLE | registerNumber;

	ENABLE_SPI
	result = HAL_SPI_Transmit(ADXL_spiHandle, &tmp, 1, ADXL_TIMEOUT_MS);
	if(result == HAL_OK)
		result = HAL_SPI_Receive(ADXL_spiHandle, value, 1, ADXL_TIMEOUT_MS);
	DISABLE_SPI

	return (result);
}
