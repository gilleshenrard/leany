#ifndef HARDWARE_SENSOR_BMI270
#define HARDWARE_SENSOR_BMI270
#include <main.h>

/**
 * @brief Enumeration of the axis of which to get measurements
 */
typedef enum {
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS,
    NB_AXIS
} axis_e;

void    bmi270InterruptTriggered(uint8_t interruptPin);
void    createBMI270Task(void);
int16_t getAngleDegreesTenths(axis_e axis);
uint8_t anglesChanged(void);
void    bmi270ZeroDown(void);
void    bmi270CancelZeroing(void);
#endif
