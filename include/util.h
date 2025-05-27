#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

typedef struct {
    float roll;
    float pitch;
    float yaw;
} drone_orientation;

typedef struct {
    int8_t x;
    int8_t y;
    int8_t z;
} drone_vector;

typedef struct {
    drone_orientation orientation;
    drone_vector velocity;
    uint16_t altitude;
} drone_state;

/**
 *  Converts the supplied pressure (hPa/mb) into a calculated altitude
 *  (meters) following the formula published by NOAA. Use sparingly,
 *  as the computational load is quite heavy.
 *  https://en.wikipedia.org/wiki/Pressure_altitude
 */
double noaa_altitude(float pressure_hPa);

/**
 *  TODO: Implement a fixed point algorithm to approximate
 *  altitude.
 *  Our barometer is only accurate to about +/- 0.50 hPa.
 *  By limiting the input to integer multiples for hPa, we
 *  are doubling the uncertainty. This input may be adjusted
 *  once the readings for the barometer are also in fixed
 *  point.
 */
int16_t approx_altitude(uint8_t pressure_hPa);

#endif
