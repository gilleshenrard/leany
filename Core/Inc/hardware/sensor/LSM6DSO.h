#ifndef LSM6DSO_H_INCLUDED
#define LSM6DSO_H_INCLUDED
#include <main.h>
#include "errorstack.h"

extern volatile uint16_t lsm6dsoTimer_ms;
extern volatile uint16_t lsm6dsoSPITimer_ms;

errorCode_u LSM6DSOinitialise(const SPI_TypeDef* handle);
errorCode_u LSM6DSOupdate();

#endif