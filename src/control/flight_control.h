/**
 ************************************************************************************************
 * @file    flight_control.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   .
 ************************************************************************************************
 */

#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "../util/quaternion.h"

typedef struct {
    float motor_f; /* front (CCW) */
    float motor_r; /* right (CW) */
    float motor_b; /* back (CCW) */
    float motor_l; /* left (CW) */
} duty_cycle_t;

void set_throttle(const float value);

void adjust_control_axis(const float pitch, const float roll, const float yaw);

/*
 * In the future, we will need to be monitoring the angular rate.
 * If we're trying to make a change and nothing is happening,
 * we will need to increase the gain. Similarly, if we're
 * oscillating/overshooting, we need to decrease gain.
 */
const duty_cycle_t* calculate_duty(const quaternion_t *orientation);

#endif
