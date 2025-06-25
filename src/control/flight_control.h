/**
 ************************************************************************************************
 * @file    flight_control.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   .
 ************************************************************************************************
 */

/*
 *  When balancing the motors using quaternions, the following thought process can be used.
 *  (if x is forward, y is left, z is up)
 *  To adjust pitch, reduce the value of x to zero.
 *  To adjust roll, reduce the value of y to zero.
 */

/*
 * An exponential growth in motor duty cycle as the orientation strays from zero makes the most sense,
 * since it will balance things out much more agressively when it needs to, and softer when it doesnt.
 */

#ifndef FLIGHT_CONTROL_H
#define FLIGHT_CONTROL_H

#include "../util/orientation.h"

typedef struct {
    float motor_1;
    float motor_2;
    float motor_3;
    float motor_4;
} duty_cycle_t;

void calculate_duty(const quaternion_t *orientation);

#endif
