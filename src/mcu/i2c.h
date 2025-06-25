/**
 ************************************************************************************************
 * @file    i2c.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   .
 ************************************************************************************************
 */


/*
 * In the future, I would be interested in trying out SMBus instead,
 * since some error checking is easier, but idk rn.
 */

#ifndef I2C_H
#define I2C_H

#include "../util/error_types.h"
#include "../mcu/types.h"

error_t i2c_init(i2c_bus_t *bus);

error_t i2c_scan_addresses(i2c_bus_t *bus);

error_t i2c_read(i2c_bus_t *bus, uint8_t device_address, uint8_t device_register, uint8_t *data);

error_t i2c_read_burst(i2c_bus_t *bus, uint8_t device_address, uint8_t device_register, uint8_t bytes, uint8_t *data);

error_t i2c_write(i2c_bus_t *bus, uint8_t device_address, uint8_t device_register, uint8_t data);

error_t i2c_write_burst(i2c_bus_t *bus, uint8_t device_address, uint8_t device_register, uint8_t bytes, uint8_t *data);

#endif
