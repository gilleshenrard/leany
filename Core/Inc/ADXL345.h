#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_
#include <stm32f1xx.h>

typedef enum{
	DEVICE_ID 		= 0x00,
	THRESHOLD_TAP	= 0x1D,
	ADXL_NB_REGISTERS
}adxl345Registers_e;

void ADXL345initialise(const SPI_HandleTypeDef* handle);
HAL_StatusTypeDef ADXL345readRegister(adxl345Registers_e registerNumber, uint8_t* value);

#endif /* INC_ADXL345_H_ */
