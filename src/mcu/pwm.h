/**
 *******************************************************************************
 * @file    pwm.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   PWM functionality on stm32.
 *******************************************************************************
 */

#include <stm32f103x6.h>
#include "../util/error_types.h"
#include "timer.h"
#include "types.h"

#include <stdint.h>

typedef struct {
    timer_t *tim;

    uint8_t enabled_channels; /* Each bit represents a channel's state */
    gpio_pin_t *ch_1;
    gpio_pin_t *ch_2;
    gpio_pin_t *ch_3;
    gpio_pin_t *ch_4;
} pwm_t;

/**
 * @brief Initializes a clock for pwm functionality.
 *
 * @param pwm Pointers to the timer and gpio pins to be used.
 *
 * @return Error type
 */
error_t pwm_init(pwm_t *pwm);

/**
 * @brief Enables pwm.
 *
 * !!! PWM must be initilized first !!!
 *
 * @param pwm Pointers to the timer and gpio pins
 *
 * @return Error type
 */
error_t pwm_enable(pwm_t *pwm);

/**
 * @brief Disables pwm.
 *
 * !!! PWM must be initilized first !!!
 *
 * @param pwm Pointers to the timer and gpio pins
 *
 * @return Error type
 */
error_t pwm_disable(pwm_t *pwm);

/**
 * @brief Sets the duty cycle of an initilized and enabled pwm channel
 *
 * !!! PWM must be initilized first !!!
 *
 * Duty cycles range from 0x0000 to 0xFFFF
 *
 * @param pwm Pointers to the timer and gpio pins
 *
 * @param channel Integer channel number (Starts at 1)
 *
 * @param duty_cycle A 16 bit number representing the range of duty cycles
 *
 */
error_t pwm_set_duty(pwm_t *pwm, uint8_t channel, uint16_t duty_cycle);
