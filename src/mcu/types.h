/**
 ************************************************************************************************
 * @file    types.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   Reference types to port, pins, etc. for an stm32 microcontroller.
 ************************************************************************************************
 */

#ifndef TYPES_H
#define TYPES_H

#include <CMSIS/Device/ST/STM32F1xx/Include/stm32f103x6.h>
#include <stdint.h>

/* tbh, these should go into their respective modules */

typedef struct {
    GPIO_TypeDef *reg;
    uint32_t rcc_clk_mask;
} gpio_port_t;

typedef struct {
    gpio_port_t port;
    uint8_t pin;
} gpio_pin_t;

typedef struct {
    TIM_TypeDef *reg;
    uint32_t rcc_clk_mask;
    gpio_pin_t pin;

    uint16_t prescaler;
    uint16_t auto_reload;
} timer_t;

typedef struct {
    I2C_TypeDef *reg;
    uint32_t rcc_clk_mask;
    gpio_pin_t pin_1;
    gpio_pin_t pin_2;
} i2c_bus_t;

typedef struct {
    ADC_TypeDef *reg;
    uint32_t rcc_clk_mask;
    gpio_pin_t gpio_pin;
} adc_pin_t;

/* Preconfigured variables for the mcu we're using */
extern const gpio_port_t GPIO_PA;
extern const gpio_port_t GPIO_PB;
extern const gpio_port_t GPIO_PC;

extern const timer_t TIMER_1; /*< Advanced control timer */
extern const timer_t TIMER_2;
extern const timer_t TIMER_3;
extern const timer_t TIMER_4;
extern const timer_t TIMER_5;
extern const timer_t TIMER_6;
extern const timer_t TIMER_7;
extern const timer_t TIMER_8; /*< Advanced control timer */


#endif
