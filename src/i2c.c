#include "i2c.h"
#include "stm32f103x6.h"

// --- ADDR DEFINES ---

#define GPIOBEN			(1U<<3)
#define I2C1EN			(1U<<21)
#define I2C1_CR2_FREQ		(0b01000) // 8
#define I2C_CCR			(0x28)
#define SD_MODE_MAX_RISE_TIME	(0b01001) // 9

// SR1
#define SR1_SB	    (1U<<0)
#define SR1_ADDR    (1U<<1)
#define SR1_BTF	    (1U<<2)
#define SR1_RXNE    (1U<<6)
#define SR1_TXE	    (1U<<7)

// SR2
#define SR2_BUSY    (1U<<1)

// CR1
#define CR1_PE	    (1U<<0)
#define CR1_START   (1U<<8)
#define CR1_STOP    (1U<<9)
#define CR1_ACK	    (1U<<10)

void i2c1_init(void)
{
    RCC->APB2ENR |= GPIOBEN;

    // Since we want a bidirectional alternate function, it will be output
    // Alternate function output Open-drain, max speed 10 MHz
    GPIOB->CRL &= ~(1U<<29);
    GPIOB->CRL |= (1U<<28);
    GPIOB->CRL |= (1U<<31);
    GPIOB->CRL |= (1U<<30);
    
    // We do not have access to pull-up resistors, so idk
    // if we need to enable them or not

    RCC->APB2ENR |= I2C1EN;

    // Software Reset
    I2C1->CR1 |= (1U<<15);
    I2C1->CR1 &= ~(1U<<15);

    I2C1->CR2 |= I2C1_CR2_FREQ; // 8 MHz FREQ
    I2C1->CCR |= I2C_CCR;	// Should now be targeting 100 kHz

    I2C1->TRISE = SD_MODE_MAX_RISE_TIME;

    I2C1->CR1 |= CR1_PE;
}

void i2c1_byte_read(char saddr, char maddr, uint8_t *data)
{
    volatile int tmp = 0;
    (void)tmp; // Silence unused warning, since tmp is only used for reading/clearing registers

    /* Wait until bus is not busy */
    while (I2C1->SR1 & (SR2_BUSY)){}

    I2C1->CR1 |= CR1_START;

    /* Wait for start flag to be set */
    while (!(I2C1->SR1 & (SR1_ADDR))){}

    /* Transmit slave address + write */
    I2C1->DR = saddr << 1;

    while (!(I2C1->SR1 & (SR1_ADDR))){}

    tmp = I2C1->SR2;

    I2C1->DR = maddr;

    while (!(I2C1->SR1 & (SR1_TXE))){}

    I2C1->CR1 |= CR1_START;

    while (!(I2C1->SR1 & (SR1_SB))){}

    I2C1->DR = saddr << 1 | 1;

    while (!(I2C1->SR1 & (SR1_ADDR))){}

    I2C1->CR1 &= ~CR1_ACK;

    tmp = I2C1->SR2;

    I2C1->CR1 |= CR1_STOP;

    while (!(I2C1->SR1 & SR1_RXNE)){}

    *data++ = I2C1->DR;
}

void i2c1_burst_read(char saddr, char maddr, int n, uint8_t *data)
{
    volatile int tmp = 0;
    (void)tmp; // Silence unused warning, only used for reading/clearing registers
    
    while (I2C1->SR2 & (SR2_BUSY)){}

    I2C1->CR1 |= CR1_START;

    while (!(I2C1->SR1 & SR1_SB)){}

    I2C1->DR = saddr << 1;

    while (!(I2C1->SR1 & SR1_ADDR)){}

    tmp = I2C1->SR2;

    while (!(I2C1->SR1 & SR1_TXE)){}

    I2C1->CR1 |= CR1_START;

    while (!(I2C1->SR1 & SR1_SB)){}

    I2C1->DR = saddr << 1 | 1;

    while (!(I2C1->SR1 & (SR1_ADDR))){}

    tmp = I2C1->SR2;

    I2C1->CR1 |= CR1_ACK;

    while (n > 0U)
    {
	// Stop condition
	if (n == 1U)
	{
	    I2C1->CR1 &= ~CR1_ACK;

	    I2C1->CR1 |= CR1_STOP;

	    while (!(I2C1->SR1 & SR1_RXNE)){}

	    *data++ = I2C1->DR;
	    break;
	} else {
	    while (!(I2C1->SR1 & SR1_RXNE)){}

	    (*data++) = I2C1->DR;

	    n--;
	}
    }
}

void i2c1_burst_write(char saddr, char maddr, int n, uint8_t *data)
{
    volatile int tmp = 0;
    (void)tmp; // Silence unused warning, only used for reading/clearing registers
    
    while (I2C1->SR2 & (SR2_BUSY)){}

    I2C1->CR1 |= CR1_START;

    while (!(I2C1->SR1 & (SR1_SB))){}

    I2C1->DR = saddr << 1;

    while (!(I2C1->SR1 & (SR1_ADDR))){}

    tmp = I2C1->SR2;

    while (!(I2C1->SR1 & (SR1_TXE))){}

    I2C1->DR = maddr;

    for (int i = 0; i < n; i++) {
	while (!(I2C1->SR1 & (SR1_TXE))){}

	I2C1->DR = *data++;
    }

    while (!(I2C1->SR1 & (SR1_BTF))){}

    I2C1->CR1 |= CR1_STOP;
}
