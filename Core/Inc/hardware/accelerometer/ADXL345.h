#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_
#include <stm32f1xx.h>

extern volatile uint8_t adxlINT1occurred;

HAL_StatusTypeDef ADXL345initialise(const SPI_HandleTypeDef* handle);
uint16_t ADXL345update();
float ADXL345getXangleDegrees();
float ADXL345getYangleDegrees();

#endif /* INC_ADXL345_H_ */
