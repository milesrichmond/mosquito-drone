#ifndef I2C_H
#define I2C_H

void i2c1_init(void);
void i2c1_byte_read(char saddr, char maddr, char* data);
void i2c1_burst_read(char saddr, char maddr, char* data);
void i2c1_burst_write(char saddr, char maddr, char* data);

#endif
