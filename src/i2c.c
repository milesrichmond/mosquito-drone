#include "i2c.h"
#include "stm32f103x6.h"

#include <stdio.h>

/* INITIALIZATION */

/* Alter by microcontroller */
void i2c1_init(void)
{
    /*------Setting up GPIO PIN B6 and B7------*/

    /*Enable clock access to GPIOB*/
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    /*Set PB6 to output 50MHz*/
    GPIOB->CRL |= GPIO_CRL_MODE6;
    /*Set PB6 to ALternate Open drain*/
    GPIOB->CRL |= GPIO_CRL_CNF6;

    /*Set PB7 to output 50MHz*/
    GPIOB->CRL |= GPIO_CRL_MODE7;
    /*Set PB7 to ALternate Open drain*/
    GPIOB->CRL |= GPIO_CRL_CNF7;

    /*Enable clock access to alternate function of the pins*/
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    /*-----------------------------------------*/

    /*------------Setting up I2C---------------*/

    /*Enable clock access to I2C1*/
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    /*Set the frequency register to 00000*/
    I2C1->CR2 &= ~(I2C_CR2_FREQ);

    /*Tell the peripheral that the clock is 8MHz(125ns)(T_CPLK1)*/ /*note: Allowed frequency Range is 2Mhz to 50Mhz*/
    I2C1->CR2 |= (8<<I2C_CR2_FREQ_Pos);

    /*Set the rise time to 1000ns(standard mode)*/
    /*(maximum rise time) = (T_CPLK1) * (TRISE - 1)*/
    I2C1->TRISE=0x9;

    /*Set the CCR*/
    /*T_high = CCR T_CPLK1*/
    /*T_low = CCR T_CPLK1*/
    I2C1->CCR|=0x28;

    /*Enable the peripheral*/
    I2C1->CR1 |= I2C_CR1_PE;

    /*-----------------------------------------*/
}

void i2c_init_port(uint8_t port_number)
{
    switch (port_number)
    {
	case 1:
	    i2c1_init();
	    break;
    }
}


/* BUS ACTIONS */

void i2c_scan_bus(I2C_TypeDef *i2c_channel)
{
    for (uint8_t i = 0; i < 128; i++)
    {
	i2c_channel->CR1 |= I2C_CR1_START;
	while(!(i2c_channel->SR1 & I2C_SR1_SB));
	
	i2c_channel->DR = (i << 1|0);
	while(!(i2c_channel->SR1)|!(i2c_channel->SR2));

	i2c_channel->CR1 |= I2C_CR1_STOP;
	for(int k = 0; k < 300; k++);

	if ((i2c_channel->SR1 & I2C_SR1_ADDR) == 2)
	{
	    printf("Found I2C device at address 0x%X (hexadecimal), or %d (decimal)\n\r", i, i);
	    fflush(stdout);
	}
    }
}

void i2c_byte_read(uint32_t i2c_port_addr, int8_t saddr, int8_t maddr, uint8_t *data)
{
    I2C_TypeDef *i2c_channel = (I2C_TypeDef *)i2c_port_addr;
    volatile int tmp = 0;
    (void)tmp;

    /* wait for empty bus */
    while (i2c_channel->SR2 & I2C_SR2_BUSY);

    /* start flag */
    i2c_channel->CR1 |= I2C_CR1_START;
    while (!(i2c_channel->SR1 & I2C_SR1_SB));

    /* slave address */
    i2c_channel->DR = saddr << 1;
    while (!(i2c_channel->SR1 & I2C_SR1_ADDR));

    /* clear ADDR flag */
    tmp = i2c_channel->SR2;

    /* wait for empty transmitter */
    while (!(i2c_channel->SR1 & I2C_SR1_TXE));

    /* transmit device memory address */
    i2c_channel->DR = maddr;
    while (!(i2c_channel->SR1 & I2C_SR1_ADDR));

    /* start flag */
    i2c_channel->CR1 |= I2C_CR1_START;
    while (!(i2c_channel->SR1 & I2C_SR1_SB));

    /* slave address + read */
    i2c_channel->DR = saddr << 1|1;
    while (!(i2c_channel->SR1 & I2C_SR1_ADDR));

    i2c_channel->CR1 &= ~(I2C_CR1_ACK);
    tmp = i2c_channel->SR2;

    /* stop flag */
    i2c_channel->CR1 |= I2C_CR1_STOP;

    /* wait for RXNE flag */
    while (!(i2c_channel->SR1 & I2C_SR1_RXNE));

    *data++ = I2C1->DR;

}

void i2c_burst_read(uint32_t i2c_port_addr, int8_t saddr, int8_t maddr, int n, uint8_t *data)
{
    I2C_TypeDef *i2c_channel = (I2C_TypeDef *)i2c_port_addr;
    volatile int tmp = 0;
    (void)tmp; /* silence warning, since temp is only used to read from registers to clear them */
    
    /* wait for empty bus */
    while (i2c_channel->SR2 & I2C_SR2_BUSY);

    /* start flag */
    i2c_channel->CR1 |= I2C_CR1_START;
    while (!(i2c_channel->SR1 & I2C_SR1_SB));

    /* slave address */
    i2c_channel->DR = saddr << 1;
    while (!(i2c_channel->SR1 & I2C_SR1_ADDR));
    
    /* clear ADDR flag */
    tmp = i2c_channel->SR2;

    /* wait for empty transmitter */
    while (!(i2c_channel->SR1 & I2C_SR1_TXE));

    /* Send memory address */
    i2c_channel->DR = maddr;
    while (!(i2c_channel->SR1 & I2C_SR1_TXE));

    /* start flag */
    i2c_channel->CR1 |= I2C_CR1_START;
    while (!(i2c_channel->SR1 & I2C_SR1_SB));

    /* slave address + read */
    i2c_channel->DR = saddr << 1 | 1;
    while (!(i2c_channel->SR1 & I2C_SR1_ADDR));

    /* clear ADDR flag */
    tmp = i2c_channel->SR2;

    i2c_channel->CR1 |= I2C_CR1_ACK;

    /* continuously read bytes */
    while (n > 0U)
    {
	/* stop condition */
	if (n == 1U)
	{
	    i2c_channel->CR1 &= ~(I2C_CR1_ACK);
	    i2c_channel->CR1 |= I2C_CR1_STOP;

	    while (!(i2c_channel->SR1 & I2C_SR1_RXNE));

	    *data++ = i2c_channel->DR;
	    break;
	}
	else
	{
	    while (!(i2c_channel->SR1 & I2C_SR1_RXNE));
	    
	    *data++ = i2c_channel->DR;
	    n--;
	}
    }
}

void i2c_burst_write(uint32_t i2c_port_addr, int8_t saddr, int8_t maddr, int n, uint8_t *data)
{
    I2C_TypeDef *i2c_channel = (I2C_TypeDef *)i2c_port_addr;
    volatile int tmp = 0;
    (void)tmp; /* silence warning, used for reading a register to clear it. */
    
    while(i2c_channel->SR2 & I2C_SR2_BUSY);

    /* start flag */
    i2c_channel->CR1 |= I2C_CR1_START;
    while (!(i2c_channel->SR1 & I2C_SR1_SB));

    /* slave address */
    i2c_channel->DR = saddr << 1;
    while (!(i2c_channel->SR1 & I2C_SR1_ADDR));

    /* clear addr flag */
    tmp = i2c_channel->SR2;

    /* wait for empty data register */
    while (!(i2c_channel->SR1 & (I2C_SR1_TXE)));

    /* send memory address */
    i2c_channel->DR = maddr;

    /* continuously transmit bytes */ 
    for (int i = 0; i < n; i++)
    {
	while (!(i2c_channel->SR1 & (I2C_SR1_TXE)));

	i2c_channel->DR = *data++;
    }

    /* Stop transmission */
    while (!(i2c_channel->SR1 & (I2C_SR1_BTF)));
    i2c_channel->CR1 |= I2C_CR1_STOP;
}
