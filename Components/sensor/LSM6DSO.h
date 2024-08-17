#ifndef LSM6DSO_H_INCLUDED
#define LSM6DSO_H_INCLUDED
#include <main.h>
#include "errorstack.h"

/**
 * @brief Enumeration of the axis of which to get measurements
 */
typedef enum {
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS,
    NB_AXIS
} axis_e;

errorCode_u LSM6DSOinitialise(const SPI_TypeDef* handle);
errorCode_u LSM6DSOupdate();
uint8_t     LSM6DSOhasChanged(axis_e axis);
int16_t     getAngleDegreesTenths(axis_e axis);
void        LSM6DSOzeroDown(void);
void        LSM6DSOcancelZeroing(void);
errorCode_u LSM6DSOhold(uint8_t toHold);

#endif
