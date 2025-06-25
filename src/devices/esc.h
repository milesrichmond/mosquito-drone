/**
 ************************************************************************************************
 * @file    esc.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   Abstraction layer to control a quadcopter's ESC. Written for an stm32 microcontroller.
 ************************************************************************************************
 */

#ifndef ESC_H
#define ESC_H

#include <stdint.h>

typedef enum {
    ESC_MOTOR_1,
    ESC_MOTOR_2,
    ESC_MOTOR_3,
    ESC_MOTOR_4,
} esc_motor_id_t;

char esc_init(uint32_t pwm_port_address);

char esc_drive_motor(esc_motor_id_t motor, uint16_t duty_cycle);

char esc_get_current(uint32_t *data);

#endif
