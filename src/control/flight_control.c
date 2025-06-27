/**
 ************************************************************************************************
 * @file    flight_control.c
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   .
 ************************************************************************************************
 */

#include "flight_control.h"
#include "parameters.h"

#include <stdint.h>

float throttle = 0.0f;
duty_cycle_t duty;

/**
 * The drone's desired orientation angle.
 * All contols will go through this axis, where the drone then
 * corrects its own axis to match.
 */
quaternion_t control_axis = { .w = 1.0f, .x = 0.0f, .y = 0.0f, .z = 0.0f };

void set_throttle(const float value)
{
    throttle = value;
}

void adjust_control_axis(const float pitch, const float roll, const float yaw)
{
    euler_t euler = { .pitch = pitch, .roll = roll, .yaw = yaw};
    quaternion_t rotation;
    quaternion_t old_axis = control_axis;

    quat(&euler, &rotation);
    quat_mult(&old_axis, &rotation, &control_axis);
}

const duty_cycle_t* calculate_duty(const quaternion_t *orientation)
{
    quaternion_t error;
    quat_error(orientation, &control_axis, &error);

    float x_correction = 2 * error.x * ATTITUDE_PITCH_GAIN; /* pitch */
    float y_correction = 2 * error.y * ATTITUDE_ROLL_GAIN; /* roll */
    float z_correction = 2 * error.z * ATTITUDE_YAW_GAIN; /* yaw */

    /* my brain hyurt */
    duty.motor_f = throttle + x_correction - z_correction;
    duty.motor_b = throttle - x_correction - z_correction;
    duty.motor_r = throttle - y_correction + z_correction;
    duty.motor_l = throttle + y_correction + z_correction;

    return &duty;
}
