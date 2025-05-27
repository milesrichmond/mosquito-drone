#ifndef I2C_H
#define I2C_H

#include "stm32f103x6.h"
#include <stdint.h>

void i2c_init_port(uint8_t port_number);
void i2c1_init(void);

void i2c_scan_bus(I2C_TypeDef *i2c_port_addr);

void i2c_byte_read(uint32_t i2c_port_addr, int8_t saddr, int8_t maddr, uint8_t *data);
void i2c_burst_read(uint32_t i2c_port_addr, int8_t saddr, int8_t maddr, int n, uint8_t *data);
void i2c_burst_write(uint32_t i2c_port_addr, int8_t saddr, int8_t maddr, int n, uint8_t *data);

#endif
