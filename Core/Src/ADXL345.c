/**
 * @brief Implement the ADXL345 accelerometer communication
 * @author Gilles Henrard
 * @date 28/09/2023
 *
 * @note Additional information can be found in :
 *   - ADXL345 datasheet : https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf
 *   - AN-1025 (FIFO application note) document : https://www.analog.com/media/en/technical-documentation/application-notes/AN-1025.pdf
 *
 */
#include "ADXL345.h"
#include "main.h"

#define ADXL_HIGH_RESERVED_REG	0x1C	///< Number of the last reserved register

#define ADXL_TIMEOUT_MS	10			///< SPI direct transmission timeout span in milliseconds

#define ADXL_WRITE		0x00		///< MSB configuration for write operations
#define ADXL_READ		0x80		///< MSB configuration for read operations
#define ADXL_SINGLE		0x00		///< Bit 6 configuration for single register operations
#define ADXL_MULTIPLE	0x40		///< Bit 6 configuration for multiple register operations

#define ADXL_MODE_BYPASS	0x00
#define ADXL_MODE_FIFO		0x40
#define ADXL_MODE_STREAM	0x80
#define ADXL_MODE_TRIGGER	0xC0
#define ADXL_TRIGGER_INT1	0x00
#define ADXL_TRIGGER_INT2	0x20
#define ADXL_SAMPLES_16		0x10

#define ADXL_INT_DATARDY	0x80
#define ADXL_INT_SINGLETAP	0x40
#define ADXL_INT_DOUBLETAP	0x20
#define ADXL_INT_ACTIVITY	0x10
#define ADXL_INT_INACTIVITY	0x08
#define ADXL_INT_FREEFALL	0x04
#define ADXL_INT_WATERMARK	0x02
#define ADXL_INT_OVERRUN	0x01

#define ADXL_SELF_TEST		0x80
#define ADXL_NO_SELF_TEST	0x00
#define ADXL_SPI_3WIRE		0x40
#define ADXL_SPI_4WIRE		0x00
#define ADXL_INT_ACTIV_HIGH 0x00
#define ADXL_INT_ACTIV_LOW	0x20
#define ADXL_FULL_RESOL		0x08
#define ADXL_LEFT_JUSTIFY	0x04
#define ADXL_RIGHT_JUSTIFY	0x00
#define ADXL_RANGE_2G		0x00
#define ADXL_RANGE_4G		0x01
#define ADXL_RANGE_8G		0x02
#define ADXL_RANGE_16G		0x03

#define ADXL_STANDBY_MODE	0x00	///< Power control bit 3 configuration for standby mode
#define ADXL_MEASURE_MODE	0x08	///< Power control bit 3 configuration for measurement mode

#define ENABLE_SPI	HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_RESET);	///< Macro used to enable the SPI communication towards the accelerometer
#define DISABLE_SPI	HAL_GPIO_WritePin(ADXL_CS_GPIO_Port, ADXL_CS_Pin, GPIO_PIN_SET);	///< Macro used to disable the SPI communication towards the accelerometer

HAL_StatusTypeDef ADXL345readRegister(adxl345Registers_e registerNumber, uint8_t* value);
HAL_StatusTypeDef ADXL345writeRegister(adxl345Registers_e registerNumber, uint8_t value);
HAL_StatusTypeDef ADXL345readRegisters(adxl345Registers_e firstRegister, uint8_t* value, uint8_t size);

SPI_HandleTypeDef* ADXL_spiHandle = NULL;	///< SPI handle used with the ADXL345
volatile uint8_t adxlINT1occurred = 0;
uint8_t buffer[6];
int16_t finalX;
int16_t finalY;
int16_t finalZ;

/**
 * @brief Initialise the ADXL345
 *
 * @param[in] handle SPI handle used
 */
HAL_StatusTypeDef ADXL345initialise(const SPI_HandleTypeDef* handle){
	ADXL_spiHandle = (SPI_HandleTypeDef*)handle;

	ADXL345writeRegister(DATA_FORMAT, ADXL_SPI_4WIRE | ADXL_INT_ACTIV_LOW | ADXL_RANGE_2G);

	//configure the FIFO as to wait for 16 samples before triggering INT1
	ADXL345writeRegister(FIFO_CONTROL, ADXL_MODE_FIFO | ADXL_TRIGGER_INT1 | ADXL_SAMPLES_16);
	ADXL345writeRegister(INTERRUPT_ENABLE, ADXL_INT_WATERMARK);

	//set the ADXL in the measurement mode (to be done last)
	return (ADXL345writeRegister(POWER_CONTROL, ADXL_MEASURE_MODE));
}

uint16_t ADXL345update(){
//	if(adxlINT1occurred){
		adxlINT1occurred = 0;
		ADXL345readRegisters(DATA_X0, buffer, 6);
		finalX = ((uint16_t)(buffer[1]) << 8) | (uint16_t)(buffer[0]);
		finalY = ((uint16_t)(buffer[3]) << 8) | (uint16_t)(buffer[2]);
		finalZ = ((uint16_t)(buffer[5]) << 8) | (uint16_t)(buffer[4]);
//	}
	return (0);
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
