/**
 ************************************************************************************************
 * @file    flight_control.c
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   .
 ************************************************************************************************
 */

#include "flight_control.h"

#include <stdint.h>

float duty_ramp(int16_t axis)
{
    return ((double)axis) / (0xFFFF); /* linear for now */
                    /* unlikely to work, since I want full duty cycle somewhere
                     * around 35-45 degrees off axis */
}

void calculate_duty(const quaternion_t *orientation)
{
    float pitch_ramp = duty_ramp(orientation->x);
    float roll_ramp = duty_ramp(orientation->y);
}
