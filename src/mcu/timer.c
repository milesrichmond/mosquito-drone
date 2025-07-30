/**
 *******************************************************************************
 * @file    timer.c
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   Abstraction layer for timers on stm32.
 *******************************************************************************
 */

#include "timer.h"

#include <stm32f103x6.h>

void tim_init(timer_t *tim)
{
    if (tim->reg == TIM1)
    {
        RCC->APB2ENR |= tim->rcc_clk_mask; /* timer 1 is in apb2 for whatever reason */
    }
    else
    {
        RCC->APB1ENR |= tim->rcc_clk_mask;
    }

    tim->reg->PSC = tim->prescaler;
    tim->reg->ARR = tim->auto_reload;
    tim->reg->CNT = 0;
}

void tim_enable(timer_t *tim)
{
    tim->reg->CR1 |= (TIM_CR1_CEN);
}

void tim_disable(timer_t *tim)
{
    tim->reg->CR1 &= ~(TIM_CR1_CEN);
}

void tim_clear_uif(timer_t tim)
{
    tim.reg->SR &= ~(TIM_SR_UIF);
}

/*
 *******************************************************************************
 *
 * Timer definitions
 *
 *******************************************************************************
 */

/*
 *******************************************************************************
 *
 * Interrupt Handlers
 *
 *******************************************************************************
 */

void TIM_default_handler(void) {}

/* TIM2-5 */
void (*GP_IRQHandler[4])(void) = { TIM_default_handler, TIM_default_handler,
                                    TIM_default_handler, TIM_default_handler };

/*
 * Something about these screams security issue. Just need to be careful when
 * changing them by user input. Otherwise, if treating this as a separate
 * library/module, this shouldn't concern us.
 */
void set_GP_IRQHandler(const uint8_t tim_number, void (*callback)(void))
{
    if (tim_number - 2 > 3)
        return;

    GP_IRQHandler[tim_number - 2] = callback;
}

void TIM2_IRQHandler(void)
{
    GP_IRQHandler[0]();
}

void TIM3_IRQHandler(void)
{
    GP_IRQHandler[1]();
}

void TIM4_IRQHandler(void)
{
    GP_IRQHandler[2]();
}

void TIM5_IRQHandler(void)
{
    GP_IRQHandler[3]();
}
