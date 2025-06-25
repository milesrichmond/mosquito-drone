/**
 ************************************************************************************************
 * @file    types.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   Reference types to port, pins, etc. for an stm32 microcontroller.
 ************************************************************************************************
 */

#include "types.h"

gpio_port_t GPIO_PA = { GPIOA, (1U << 2) };
gpio_port_t GPIO_PB = { GPIOB, (1U << 3) };
gpio_port_t GPIO_PC = { GPIOC, (1U << 4) };
