#ifndef ADC_H
#define ADC_H

#include<stdint.h>
#include "stm32f103x6.h"

#include "util.h"

typedef enum {
    ADC_CONFIG_SIMULTANEOUS,
    ADC_CONFIG_INTERLEAVED,
    ADC_CONFIG_SINGLE_SHUNT,
} adc_config_t;

void adc_init(gpio_pin_t *pin, ADC_TypeDef *adc, uint8_t adc_channel, uint8_t adc_clk_offset);
void adc_port_configure(ADC_TypeDef port, adc_config_t conf);
void adc_calibrate(void);
void adc_read(uint16_t *data);

#endif
