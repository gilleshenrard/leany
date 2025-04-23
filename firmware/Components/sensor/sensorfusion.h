#ifndef COMPONENTS_SENSOR_SENSORFUSION_H
#define COMPONENTS_SENSOR_SENSORFUSION_H

void complementaryFilter(const float accelerometer_G[], const float gyroscope_radps[], float filteredAngles_rad[]);

#endif
