#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void i2c1_init(void);
void i2c1_byte_read(char saddr, char maddr, uint8_t* data);
void i2c1_burst_read(char saddr, char maddr, int n, uint8_t* data);
void i2c1_burst_write(char saddr, char maddr, int n, uint8_t* data);

#endif
