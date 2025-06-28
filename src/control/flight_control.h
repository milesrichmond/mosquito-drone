/**
 *******************************************************************************
 * @file    flight_control.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   Control interface for the mosquito-drone project.
 *******************************************************************************
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

/**
 * @brief Alters the throttle value.
 *
 * The throttle value does not really correspond to the duty cycle of the
 * motors, since some wiggle room must be left for the balancing offsets.
 * E.g. throttle=1.0 does not mean maximum motor speed.
 *
 * @param value Percent throttle (0.0-1.0).
 */
void set_throttle(const float value);

/**
 * @brief Controls the drone's desired position.
 *
 * All controls to the drone's position should go through this function,
 * where the drone's auto-balancing will attempt to match the drone's
 * orientation to that new controled orientation.
 *
 * Set both pitch and roll to 0.0 for a hover-like position.
 *
 * @param pitch Euler pitch value.
 *
 * @param roll Euler roll value.
 *
 * @param yaw Euler yaw value.
 */
void adjust_control_axis(const float pitch, const float roll, const float yaw);

/*
 * In the future, we will need to be monitoring the angular rate.
 * If we're trying to make a change and nothing is happening,
 * we will need to increase the gain. Similarly, if we're
 * oscillating/overshooting, we need to decrease gain.
 */
/**
 * @brief calculates the required duty cycle to correct drone orientation.
 *
 * This function will largely remain unfinished, as it is the main
 * controlling function for the drone. It will likely go through
 * several iterations and behaviors (so no promises, yeah?).
 *
 * @param orientation The drone's current orientation.
 */
const duty_cycle_t* calculate_duty(const quaternion_t *orientation);

#endif
