#ifndef INC_ADXL345_H_
#define INC_ADXL345_H_
#include <stm32f1xx.h>
#include "errors.h"

extern volatile uint8_t		adxlINT1occurred;
extern volatile uint16_t	adxlTimer_ms;

extern uint8_t anglesDifferent(float angleA, float angleB);

errorCode_u ADXL345initialise(const SPI_HandleTypeDef* handle);
errorCode_u ADXL345update();
uint8_t ADXL345hasNewMeasurements();
float ADXL345getXangleDegrees();
float ADXL345getYangleDegrees();

#endif /* INC_ADXL345_H_ */
