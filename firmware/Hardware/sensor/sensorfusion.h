#ifndef HARDWARE_SENSOR_SENSORFUSION_H
#define HARDWARE_SENSOR_SENSORFUSION_H
#include <stdint.h>

enum {
    QUATER_ALIGNMENT    = 16U,  ///< Memory alignment of the quaternion structure
    CONTEXT_ALIGNMENT   = 64U,  ///< Memory alignment of the mahony context structure
    TIMEDELTA_ALIGNMENT = 16U,  ///< Memory alignment of the time delta structure
};

/**
 * Enumeration of the axis of which to get measurements
 */
typedef enum {
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS,
    NB_AXIS
} axis_e;

/**
 * Structure defining time delta
 */
typedef struct {
    uint32_t currentTick;        ///< Current sensor tick value
    uint32_t previousTick;       ///< Tick value at the last update
    uint32_t maxTick;            ///< Maximum sensor tick value
    float    resolutionSeconds;  ///< Tick resolution in [s]
} __attribute__((aligned(TIMEDELTA_ALIGNMENT))) timedelta_t;

/**
 * Structure defining a quaternion
 */
typedef struct {
    float q0;  ///< Scalar value
    float q1;  ///< Value which multiplies the unit vector along the X axis
    float q2;  ///< Value which multiplies the unit vector along the Y axis
    float q3;  ///< Value which multiplies the unit vector along the Z axis
} __attribute__((aligned(QUATER_ALIGNMENT))) quaternion_t;

/**
 * Structure defining a mahony filter context
 */
typedef struct {
    quaternion_t attitude;                 ///< Current attitude quaternion
    timedelta_t  dt;                       ///< Time delta between updates
    float        errorIntegrals[NB_AXIS];  ///< Array containing the integrated errors
    float        KP;                       ///< PI filter proportional gain
    float        KI;                       ///< PI filter integral gain
    uint8_t      badAccelerationCount;     ///< Number of bad accelerometer norms since reset
    uint8_t      badQuaternionCount;       ///< Number of bad quaternion norms since reset
} __attribute__((aligned(CONTEXT_ALIGNMENT))) mahonycontext_t;

static const float PROPORTIONAL_GAIN = 2.5F;  ///< Propotional gain (KP) of the Mahony filter
static const float INTEGRAL_GAIN     = 0.5F;  ///< Integral gain (KI) of the Mahony filter

void  resetMahonyFilter(mahonycontext_t* context);
void  updateMahonyFilter(mahonycontext_t* context, const float accelerometer_G[NB_AXIS],
                         const float gyroscope_radps[NB_AXIS]);
float angleAlongAxis(const mahonycontext_t* context, axis_e axis);
float getAttitudeAngle(const mahonycontext_t* context);

#endif
