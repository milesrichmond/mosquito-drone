#include "adc.h"

void adc_init(gpio_pin_t *pin, ADC_TypeDef *adc, uint8_t adc_channel, uint8_t adc_clk_mask)
{
    /* Clock access */
    RCC->APB2ENR |= pin->port.clk_msk;
    RCC->APB2ENR |= adc_clk_mask;

    /* Pin config */
    pin->port.reg->CRL &= ~(3U<<((pin->pin - 1) * 2));

    /* ADC default config */
    if (adc_channel > 9)
    {
	adc->SMPR1 |= ((1U<<2) << (adc_channel - 9)); /* Sample time = 41.5 cycles */
    } else
    {
	adc->SMPR2 |= ((1U<<2) << adc_channel); /* Sampe time = 41.5 cycles */
    }

    if (adc_channel < 7)
    {

    } else if (adc_channel < 13)
    {

    } else
    {

    }

    /* Power on module */
    adc->CR2 |= (1U<<0);
}
