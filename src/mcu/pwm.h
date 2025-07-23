/**
 *******************************************************************************
 * @file    pwm.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   PWM functionality on stm32.
 *******************************************************************************
 */

#include <CMSIS/Device/ST/STM32F1xx/Include/stm32f103x6.h>
#include "../util/error_types.h"
#include "timer.h"
#include "types.h"

#include <stdint.h>

typedef struct {
    timer_t *tim;

    uint8_t enabled_channels;
    gpio_pin_t *ch_1;
    gpio_pin_t *ch_2;
    gpio_pin_t *ch_3;
    gpio_pin_t *ch_4;
} pwm_t;

error_t pwm_init(pwm_t *pwm);

error_t pwm_enable(pwm_t *pwm);

error_t pwm_disable(pwm_t *pwm);

error_t pwm_set_duty(pwm_t *pwm, uint8_t channel, uint16_t duty_cycle);
