#ifndef COMPONENTS_SENSOR_BMI270
#define COMPONENTS_SENSOR_BMI270
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

void bmi270InterruptTriggered(uint8_t interruptPin);
void createBMI270Task(const SPI_TypeDef* handle);
#endif
