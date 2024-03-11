#ifndef LSM6DSO_H_INCLUDED
#define LSM6DSO_H_INCLUDED
#include <main.h>
#include "errorstack.h"

errorCode_u LSM6DSOinitialise(const SPI_TypeDef* handle);
errorCode_u LSM6DSOupdate();

#endif