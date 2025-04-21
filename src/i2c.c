#include "stm32f103x6.h"

// --- ADDR DEFINES ---

#define GPIOBEN			(1U<<3)
#define I2C1EN			(1U<<21)
#define I2C_CCR			(0x28)
#define SD_MODE_MAX_RISE_TIME	()

void i2c_init(void)
{
    RCC->APB2ENR |= GPIOBEN;

    GPIOB->CRL |= (1U<<30);
    GPIOB->CRL |= (1U<<31);

    GPIOB->CRL &= ~(1U<<29);
    GPIOB->CRL |= (1U<<28);
    GPIOB->CRL |= (1U<<31);
    GPIOB->CRL &= ~(1U<<30);

    GPIOB->ODR |= (1U<<6);
    GPIOB->ODR |= (1U<<7);

    RCC->APB2ENR |= I2C_CCR;

    // Software Reset
    I2C1->CR1 |= (1U<<15);
    I2C1->CR1 &= ~(1U<<15);

    I2C1->CCR |= I2C_CCR;

    I2C1->TRISE = SD_MODE_MAX_RISE_TIME;

    I2C1-> CR1 |= CR1_PE;
}
