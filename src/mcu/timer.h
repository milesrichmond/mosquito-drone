/**
 ************************************************************************************************
 * @file    timer.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   .
 ************************************************************************************************
 */

#include "types.h"

#include <CMSIS/Device/ST/STM32F1xx/Include/stm32f103x6.h>

/* The stm32f103c8t6 has a SYSCLK of 72 Mhz max, which is scaled down through various
 * prescalers to each of the timers and clocks throughout the system.
 * For more, check figure 11 on the user man. */

/* finding a timer's frequency:
 * frequency (hz) = (Timer clock) / ((prescaler + 1) * (auto-reset + 1))
 */

typedef struct timer_t
{
    TIM_TypeDef *reg;
    uint32_t rcc_clk_mask;

    uint16_t prescaler;
    uint16_t auto_reload;
} timer_t;

extern const timer_t TIM_1;
extern const timer_t TIM_2;
extern const timer_t TIM_3;
extern const timer_t TIM_4;
extern const timer_t TIM_5;
extern const timer_t TIM_6;
extern const timer_t TIM_7;
extern const timer_t TIM_8;

void tim_init(timer_t tim);

void tim_enable(timer_t tim);

void tim_disable(timer_t tim);

void tim_clear_uif(timer_t tim);
