#include "i2c.h"
#include "stm32f103x6.h"

#include <stdio.h>
#include "debug.h"

/**
 *  Enables clock access to the requisite components, then configures
 *  pins and i2c frequency.
 *
 *  Defaults:
 *    - 10 Mhz Open-drain GPIO
 *    - 8 Mhz i2c clock with 1000 ns max rise time
 */
void i2c_init(i2c_bus_t bus)
{
    /* Clock Access */
    RCC->APB2ENR |= bus.pin_1.port.clk_msk;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= bus.clk_msk;

    /* Pin Config */
    bus.pin_1.port.reg->CRL |= (1U << (bus.pin_1.pin * 4));	    /* 10 MHz */
    bus.pin_1.port.reg->CRL |= (3U << (bus.pin_1.pin * 4 + 2));	    /* Open-drain */
    
    bus.pin_2.port.reg->CRL |= (1U << (bus.pin_2.pin * 4));	    /* 10 MHz */
    bus.pin_2.port.reg->CRL |= (3U << (bus.pin_2.pin * 4 + 2));	    /* Open-drain */
    
    /* I2C Config */
    bus.reg->CR2 &= ~(I2C_CR2_FREQ);				    /* Clear frequency reg */
    bus.reg->CR2 |= (8U << I2C_CR2_FREQ_Pos);			    /* 8 MHz clock */

    bus.reg->TRISE = 0x9;					    /* 1000 ns max rise time */

    bus.reg->CCR |= 0x28;					    /* Clock control */

    /* Enable peripherals */
    bus.reg->CR1 |= I2C_CR1_PE;
}

/**
 *  Scans the bus for all responsive addresses and prints them to stdout.
 */
void i2c_scan(i2c_bus_t bus)
{
    for (uint8_t address = 0U; address < 128; address++)
    {
	while(bus.reg->SR2 & I2C_SR2_BUSY);

	bus.reg->CR1 |= I2C_CR1_START;
	while (!(bus.reg->SR1 & I2C_SR1_SB));

	bus.reg->DR = address << 1;
	while (!(bus.reg->SR1)|!(bus.reg->SR2));    /* No errors or alerts (Not Missing NACK) */

	bus.reg->CR1 |= I2C_CR1_STOP;
	for (int k = 0; k < 100; k++);		    /* There is no easy way to check for a stop
						     * condition, so a crude delay is used. */

	if ((bus.reg->SR1 & I2C_SR1_ADDR) == 2)	    /* ACK recieved with no errors */
	{
	    dbg_log("Found I2C device at address 0x%X\n\r", address);
	}
    }
}

/**
 *  Begins the first phase of all i2c communication.
 *  
 *  Transmits:
 *    1. Start bit
 *    2. Device address + write bit
 *    3. Device memory address
 */
void i2c_phase_1(i2c_bus_t bus, uint8_t device_addr, uint8_t mem_addr)
{
    while (bus.reg->SR2 & I2C_SR2_BUSY);

    bus.reg->CR1 |= I2C_CR1_START;
    while (!(bus.reg->SR1 & I2C_SR1_SB));

    bus.reg->DR = device_addr << 1;
    while (!(bus.reg->SR1 & I2C_SR1_ADDR));
    volatile int tmp = bus.reg->SR2;
    (void)tmp;

    while (!(bus.reg->SR1 & I2C_SR1_TXE));

    bus.reg->DR = mem_addr;
    while (!(bus.reg->SR1 & I2C_SR1_ADDR));
}

/**
 *  Reads from an i2c device.
 *
 *  TODO: Error handling
 */
void i2c_read(i2c_bus_t bus, uint8_t device_addr, uint8_t mem_addr, int bytes, uint8_t *data)
{
    /*
     *	1. Wait for empty bus
     *	2. Start bit
     *	3. Device address
     *	4. Write bit
     *	5. Memory address pointer
     */
    i2c_phase_1(bus, device_addr, mem_addr);

    /*
     *	6. Start bit
     *	7. Device address
     *	8. Read bit
     *	9. Read byte
     *	10. Stop condition / repeat at step 9
     */

    bus.reg->CR1 |= I2C_CR1_START;
    while (!(bus.reg->SR1 & I2C_SR1_SB));

    bus.reg->DR = device_addr << 1|1;
    while (!(bus.reg->SR1 & I2C_SR1_ADDR));
    volatile int tmp = bus.reg->SR2;
    (void)tmp;

    while (bytes > 0U)
    {
	if (bytes == 1U)
	{
	    bus.reg->CR1 |= I2C_CR1_STOP;
	} 
	
	while (!(bus.reg->SR1 & I2C_SR1_RXNE));
	*data++ = bus.reg->DR;
	bytes--;
    }
}

/**
 *  Writes to an i2c device.
 *
 *  TODO: Error handling
 */
void i2c_write(i2c_bus_t bus, uint8_t device_addr, uint8_t mem_addr, int bytes, uint8_t *data)
{
    /*
     *	1. Wait for empty bus
     *	2. Start bit
     *	3. Device address
     *	4. Write bit
     *	5. Memory address pointer
     */
    i2c_phase_1(bus, device_addr, mem_addr);

    /*
     *	6. Send byte
     *	7. Stop condition / repeat step 6
     */

    while (bytes > 0U)
    {
	while (!(bus.reg->SR1 & I2C_SR1_TXE));
	bus.reg->DR = *data++;
	bytes--;
    }

    while (!(bus.reg->SR1 & I2C_SR1_BTF));
    bus.reg->CR1 |= I2C_CR1_STOP;
}
























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
