#include "types.h"

#include "stm32f103x6.h"

gpio_port_t GPIO_PA = { GPIOA, (1U<<2) };
gpio_port_t GPIO_PB = { GPIOB, (1U<<3) };
gpio_port_t GPIO_PC = { GPIOC, (1U<<4) };
