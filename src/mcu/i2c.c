/**
 ************************************************************************************************
 * @file    i2c.c
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   .
 ************************************************************************************************
 */

#include "i2c.h"

#include "../util/debug.h"
#include <CMSIS/Device/ST/STM32F1xx/Include/stm32f103x6.h>

#define TIMEOUT 10000

error_t i2c_init(i2c_bus_t *bus)
{
    /* Enable clock access */
    RCC->APB2ENR |= bus->pin_1.port.rcc_clk_mask; /*< Both pins should be in the same port */
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    RCC->APB1ENR |= bus->rcc_clk_mask;

    /* Configure pins */
    bus->pin_1.port.reg->CRL &= ~(3U << (bus->pin_1.pin * 4)); /* clear */
    bus->pin_1.port.reg->CRL |= (1U << (bus->pin_1.pin * 4)); /* 10 Mhz */
    bus->pin_1.port.reg->CRL |= (3U << (bus->pin_1.pin * 4 + 2)); /* Open-drain */

    bus->pin_2.port.reg->CRL &= ~(3U << (bus->pin_2.pin * 4)); /* clear */
    bus->pin_2.port.reg->CRL |= (1U << (bus->pin_2.pin * 4)); /* 10 Mhz */
    bus->pin_2.port.reg->CRL |= (3U << (bus->pin_2.pin * 4 + 2)); /* Open-drain */

    /* Configure I2C */
    bus->reg->CR2 &= ~(I2C_CR2_FREQ);
    bus->reg->CR2 |= (8U << I2C_CR2_FREQ_Pos);
    bus->reg->TRISE = 0x9;
    bus->reg->CCR |= 0x28;

    /* Enable */
    bus->reg->CR1 |= I2C_CR1_PE;

    /* Error checking performed with a scan */
    return i2c_scan_addresses(bus);
}

error_t generate_start(i2c_bus_t *bus, uint16_t timeout)
{
    bus->reg->CR1 |= I2C_CR1_START;

    while (!(bus->reg->SR1 & I2C_SR1_SB) && --timeout);

    if (!timeout)
    {
        return I2C_TIMEOUT_ERROR;
    }

    return NO_ERROR;
}

void generate_stop(i2c_bus_t *bus)
{
    bus->reg->CR1 |= I2C_CR1_STOP;
    for (uint8_t k = 0; k < 100; k++);
}


void clear_addr_flag(i2c_bus_t *bus)
{
    volatile uint16_t tmp = bus->reg->SR1;
    tmp = bus->reg->SR2;
    (void)tmp;
}

error_t send_device_address(i2c_bus_t *bus, uint8_t addr, uint16_t timeout)
{
    bus->reg->DR = addr;

    /* wait for address to be sent and acknowledged */
    while (!(bus->reg->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) && --timeout);
    
    if (bus->reg->SR1 & I2C_SR1_AF)
    {
        bus->reg->SR1 &= ~(I2C_SR1_AF);
        return I2C_NACK_ERROR;
    }

    if (!timeout)
    {
        return I2C_TIMEOUT_ERROR;
    }

    clear_addr_flag(bus);
    return NO_ERROR;
}

error_t send_byte(i2c_bus_t *bus, uint8_t byte, uint16_t timeout)
{
    bus->reg->DR = byte;

    /* wait for transmission to finish */
    while (!(bus->reg->SR1 & I2C_SR1_TXE) && --timeout);

    if (!timeout)
    {
        return I2C_TIMEOUT_ERROR;
    }

    return NO_ERROR;
}

error_t read_byte(i2c_bus_t *bus, uint8_t *data, uint16_t timeout)
{
    while (!(bus->reg->SR1 & I2C_SR1_RXNE) && --timeout);

    if (!timeout)
    {
        return I2C_TIMEOUT_ERROR;
    }

    *data = bus->reg->DR;
    return NO_ERROR;
}

error_t i2c_scan_addresses(i2c_bus_t *bus)
{
    error_t err;

    for (uint8_t address = 0U; address < 128; address++)
    {
        while (bus->reg->SR2 & I2C_SR2_BUSY);
 
        err = generate_start(bus, TIMEOUT);
        if (err)
        {
            return err;
        }

        err = send_device_address(bus, address << 1, TIMEOUT);

        generate_stop(bus);
        if (!err)
        {
            dbg_log("Found I2C device at address 0x%X\n\r", address);
        }
    }

    return NO_ERROR;
}

error_t i2c_read(i2c_bus_t *bus, uint8_t device_address, uint8_t device_register, uint8_t *data)
{
    error_t err;
    while (bus->reg->SR2 & I2C_SR2_BUSY);
    
    /* Start condition */
    if ((err = generate_start(bus, TIMEOUT)))
        return err;

    /* Send device address + write bit */
    if ((err = send_device_address(bus, device_address << 1, TIMEOUT)))
        return err;

    /* Send device register address */
    if ((err = send_byte(bus, device_register, TIMEOUT)))
        return err;

    /* Repeated start */
    if ((err = generate_start(bus, TIMEOUT)))
        return err;

    /* Send device address + read bit */
    if ((err = send_device_address(bus, (device_address << 1) | 1, TIMEOUT)))
        return err;

    /* NACK after next byte */
    bus->reg->CR1 &= ~(I2C_CR1_ACK);

    clear_addr_flag(bus);
    generate_stop(bus);

    if ((err = read_byte(bus, data, TIMEOUT)))
        return err;

    return NO_ERROR;
}

error_t i2c_read_burst(i2c_bus_t *bus, uint8_t device_address, uint8_t device_register, uint8_t bytes, uint8_t *data)
{
    error_t err;
    while (bus->reg->SR2 & I2C_SR2_BUSY);

    /* Start Condition */
    if ((err = generate_start(bus, TIMEOUT)))
        return err;

    /* Send device address + write bit */
    if ((err = send_device_address(bus, device_address << 1, TIMEOUT)))
        return err;

    /* Send device register address */
    if ((err = send_byte(bus, device_register, TIMEOUT)))
        return err;

    /* Repeated start */
    if ((err = generate_start(bus, TIMEOUT)))
        return err;

    /* Send device address + read bit */
    if ((err = send_device_address(bus, (device_register << 1) | 1, TIMEOUT)))
        return err;

    bus->reg->CR1 |= I2C_CR1_ACK;
    clear_addr_flag(bus);

    while (bytes > 0U)
    {
        if (bytes == 1U)
        {
            /* Don't ACK last byte + stop condition */
            bus->reg->CR1 &= ~(I2C_CR1_ACK);
            bus->reg->CR1 |= I2C_CR1_STOP;
        }

        if ((err = read_byte(bus, data++, TIMEOUT)))
            return err;
        bytes--;
    }

    return NO_ERROR;
}

error_t i2c_write(i2c_bus_t *bus, uint8_t device_address, uint8_t device_register, uint8_t data)
{
    error_t err;
    while (bus->reg->SR2 & I2C_SR2_BUSY);

    /* Start condition */
    if ((err = generate_start(bus, TIMEOUT)))
        return err;

    /* Send device address + write bit */
    if ((err = send_device_address(bus, device_address << 1, TIMEOUT)))
        return err;

    clear_addr_flag(bus);

    /* Send device register address */
    if ((err = send_byte(bus, device_register, TIMEOUT)))
        return err;

    /* Send data to be written */
    if ((err = send_byte(bus, data, TIMEOUT)))
        return err;

    generate_stop(bus);

    return NO_ERROR;
}

error_t i2c_write_burst(i2c_bus_t *bus, uint8_t device_address, uint8_t device_register, uint8_t bytes, uint8_t *data)
{
    error_t err;
    while (bus->reg->SR2 & I2C_SR2_BUSY);

    /* Start condition */
    if ((err = generate_start(bus, TIMEOUT)))
        return err;

    /* Send device address + write bit */
    if ((err = send_device_address(bus, device_address, TIMEOUT)))
        return err;

    clear_addr_flag(bus);

    /* Send device register address */
    if ((err = send_byte(bus, device_register, TIMEOUT)))
        return err;

    while (bytes-- > 0)
    {
        if ((err = send_byte(bus, *data++, TIMEOUT)))
            return err;
    }

    generate_stop(bus);

    return NO_ERROR;
}
