/**
 ************************************************************************************************
 * @file    timer.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   .
 ************************************************************************************************
 */

#include "types.h"

/* The stm32f103c8t6 has a SYSCLK of 72 Mhz max, which is scaled down through various
 * prescalers to each of the timers and clocks throughout the system.
 * For more, check figure 11 on the user man. */

/* finding a timer's frequency:
 * frequency (hz) = (Timer clock) / ((prescaler + 1) * (auto-reset + 1))
 */

void tim_init(timer_t tim);

void tim_enable(timer_t tim);

void tim_disable(timer_t tim);

void tim_clear_uif(timer_t tim);
