/**
 *******************************************************************************
 * @file    pwm.c
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   PWM functionality on stm32.
 *******************************************************************************
 */

#include "pwm.h"

/* TODO: ERROR HANDLING */
error_t pwm_init(pwm_t *pwm)
{
    RCC->APB1ENR |= pwm->tim->rcc_clk_mask;

    /* ch 1 enabled */
    if ((pwm->enabled_channels & (1 << 0)) != 0)
    {
        RCC->APB2ENR |= pwm->ch_1->port.rcc_clk_mask;
        pwm->ch_1->port.reg->CRL |= (
            7U << (pwm->ch_1->pin * 2) /* AF out, push-pull, 50 MHz */
        );
        pwm->tim->reg->CCR1 = 0; /* 0% duty default */
        pwm->tim->reg->CCMR1 |= (6 << 4) | TIM_CCMR1_OC1PE; /* pwm mode 1 */
        pwm->tim->reg->CCER |= TIM_CCER_CC1E; /* enable output */
    }

    /* ch 2 enabled */
    if ((pwm->enabled_channels & (1 << 1)) != 0)
    {
        RCC->APB2ENR |= pwm->ch_2->port.rcc_clk_mask;
        pwm->ch_2->port.reg->CRL |= (
            7U << (pwm->ch_2->pin * 2) /* AF out, push-pull, 50 MHz */
        );
        pwm->tim->reg->CCR2 = 0; /* 0% duty default */
        pwm->tim->reg->CCMR1 |= (6 << 12) | TIM_CCMR1_OC2PE; /* pwm mode 1 */
        pwm->tim->reg->CCER |= TIM_CCER_CC2E; /* enable output */
    }

    /* ch 3 enabled */
    if ((pwm->enabled_channels & (1 << 2)) != 0)
    {
        RCC->APB2ENR |= pwm->ch_3->port.rcc_clk_mask;
        pwm->ch_3->port.reg->CRL |= (
            7U << (pwm->ch_3->pin * 2) /* AF out, push-pull, 50 MHz */
        );
        pwm->tim->reg->CCR3 = 0; /* 0% duty default */
        pwm->tim->reg->CCMR2 |= (6 << 4) | TIM_CCMR2_OC3PE; /* pwm mode 1 */
        pwm->tim->reg->CCER |= TIM_CCER_CC3E; /* enable output */
    }

    /* ch 4 enabled */
    if ((pwm->enabled_channels & (1 << 3)) != 0)
    {
        RCC->APB2ENR |= pwm->ch_4->port.rcc_clk_mask;
        pwm->ch_4->port.reg->CRL |= (
            7U << (pwm->ch_4->pin * 2) /* AF out, push-pull, 50 MHz */
        );
        pwm->tim->reg->CCR4 = 0; /* 0% duty default */
        pwm->tim->reg->CCMR2 |= (6 << 12) | TIM_CCMR2_OC4PE; /* pwm mode 1 */
        pwm->tim->reg->CCER |= TIM_CCER_CC4E; /* enable output */
    }

    pwm->tim->reg->CR1 |= TIM_CR1_ARPE;
    pwm->tim->reg->EGR |= TIM_EGR_UG;

    return NO_ERROR;
}

error_t pwm_enable(pwm_t *pwm)
{
    tim_enable(pwm->tim);

    return NO_ERROR;
}

error_t pwm_disable(pwm_t *pwm)
{
    tim_disable(pwm->tim);

    return NO_ERROR;
}

error_t pwm_set_duty(pwm_t *pwm, uint8_t channel, uint16_t duty_cycle)
{
    if (channel - 1 > 3)
        return INVALID_ARG;

    if ((pwm->enabled_channels & (1 << channel)) == 0)
        return INVALID_STATE;

    switch (channel)
    {
        case 1:
            pwm->tim->reg->CCR1 = duty_cycle;
            break;
        case 2:
            pwm->tim->reg->CCR2 = duty_cycle;
            break;
        case 3:
            pwm->tim->reg->CCR3 = duty_cycle;
            break;
        case 4:
            pwm->tim->reg->CCR4 = duty_cycle;
            break;
        default:
            break;
    }

    return NO_ERROR;
}
