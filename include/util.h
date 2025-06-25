#pragma once
#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

typedef struct {
    float w;
    float x;
    float y;
    float z;
} drone_vec4;

typedef struct {
    int8_t x;
    int8_t y;
    int8_t z;
} drone_vec3;

typedef struct {
    drone_vec4 orientation;
    drone_vec3 velocity;
    uint16_t altitude;
} drone_state;

/**
 *  Convert the supplied quaternion into an euler equivalent.
 */
drone_vec3 euler_value(drone_vec4 quat);

/**
 *  Calculates the requisite quaternion to rotate the current
 *  orientation by to reach the target orientation
 */
drone_vec4 quat_error(drone_vec4 current, drone_vec4 target);

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
