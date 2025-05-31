#ifndef TYPES_H
#define TYPES_H

#include "stm32f103x6.h"

/* GPIO */

typedef struct {
    GPIO_TypeDef *reg;
    const uint32_t clk_msk;
} gpio_port_t;

typedef struct {
    gpio_port_t port;
    const uint8_t pin;
} gpio_pin_t;

/* I2C */

typedef struct {
    I2C_TypeDef *reg;
    const uint32_t clk_msk;
    gpio_pin_t pin_1;
    gpio_pin_t pin_2;
} i2c_bus_t;

/* ADC */

typedef struct {
    ADC_TypeDef *reg;
    const uint32_t clk_msk;
    gpio_pin_t gpio_pin;
} adc_port_t;

/******************************************************************************/
/*                                                                            */
/*                    STM32F103 VARIABLES                                     */
/*                                                                            */
/******************************************************************************/

/* GPIO */

extern gpio_port_t GPIO_PA;
extern gpio_port_t GPIO_PB;
extern gpio_port_t GPIO_PC;


#endif
