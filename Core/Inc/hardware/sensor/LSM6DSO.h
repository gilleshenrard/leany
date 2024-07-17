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

extern volatile uint16_t lsm6dsoTimer_ms;
extern volatile uint16_t lsm6dsoSPITimer_ms;

errorCode_u LSM6DSOinitialise(const SPI_TypeDef* handle);
errorCode_u LSM6DSOupdate();
uint8_t     LSM6DSOhasChanged(axis_e axis);
int16_t     getAngleDegreesTenths(axis_e axis);

#endif
