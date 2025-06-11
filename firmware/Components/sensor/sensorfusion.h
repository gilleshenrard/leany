#ifndef COMPONENTS_SENSOR_SENSORFUSION_H
#define COMPONENTS_SENSOR_SENSORFUSION_H
#include "memsBMI270.h"

void  resetMahonyFilter(void);
void  updateMahonyFilter(float accelerometer_G[], const float gyroscope_radps[], float dtPeriod_sec);
float angleAlongAxis(axis_e axis);
float getAttitudeAngle(void);

#endif
