/**
 ************************************************************************************************
 * @file    timer.c
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   Abstraction layer for timers on stm32.
 ************************************************************************************************
 */

#include "timer.h"

#include <CMSIS/Device/ST/STM32F1xx/Include/stm32f103x6.h>

void tim_init(timer_t tim)
{
    if (tim.reg == TIM1)
    {
        RCC->APB2ENR |= tim.rcc_clk_mask; /* timer 1 is in apb2 for whatever reason */
    }
    else
    {
        RCC->APB1ENR |= tim.rcc_clk_mask;
    }

    tim.reg->PSC = tim.prescaler;
    tim.reg->ARR = tim.auto_reload;
    tim.reg->CNT = 0;
}

void tim_enable(timer_t tim)
{
    tim.reg->CR1 |= (TIM_CR1_CEN);
}

void tim_disable(timer_t tim)
{
    tim.reg->CR1 &= ~(TIM_CR1_CEN);
}

void tim_clear_uif(timer_t tim)
{
    tim.reg->SR &= ~(TIM_SR_UIF);
}
