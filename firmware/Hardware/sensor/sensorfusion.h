#ifndef HARDWARE_SENSOR_SENSORFUSION_H
#define HARDWARE_SENSOR_SENSORFUSION_H
#include "memsBMI270.h"

enum {
    QUATER_ALIGNMENT = 16U,  ///< Memory alignment of the quaternion structure
};

/**
 * Structure defining a quaternion
 */
typedef struct {
    float q0;  ///< Scalar value
    float q1;  ///< Value which multiplies the unit vector along the X axis
    float q2;  ///< Value which multiplies the unit vector along the Y axis
    float q3;  ///< Value which multiplies the unit vector along the Z axis
} __attribute__((aligned(QUATER_ALIGNMENT))) quaternion_t;

void  resetMahonyFilter(quaternion_t* currentAttitude, float integratedErrors[NB_AXIS]);
void  updateMahonyFilter(quaternion_t* currentAttitude, float integratedErrors[NB_AXIS], float accelerometer_G[NB_AXIS],
                         const float gyroscope_radps[NB_AXIS], float dtPeriod_sec);
float angleAlongAxis(const quaternion_t* currentAttitude, axis_e axis);
float getAttitudeAngle(const quaternion_t* currentAttitude);

#endif
