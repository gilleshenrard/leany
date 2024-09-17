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

void        createLSM6DSOTask(const SPI_TypeDef* handle);
void        lsm6dsoInterruptTriggered(uint8_t interruptPin);
uint8_t     lsm6dsoHasChanged(axis_e axis);
int16_t     getAngleDegreesTenths(axis_e axis);
void        lsm6dsoZeroDown(void);
void        lsm6dsoCancelZeroing(void);
errorCode_u lsm6dsoHold(uint8_t toHold);

#endif
