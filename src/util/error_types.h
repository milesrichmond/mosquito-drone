/**
 *******************************************************************************
 * @file    error_types.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   Error codes and type shared by the mosquito-drone project.
 *******************************************************************************
 */

#ifndef ERROR_TYPES_H
#define ERROR_TYPES_H

#include <stdint.h>

#define error_t uint8_t

#define NO_ERROR        0x00
#define INVALID_ARG     0x01
#define INVALID_ADDR    0x02
#define INVALID_STATE   0x03

/*
 *******************************************************************************
 *
 * I2C
 *
 *******************************************************************************
 */

#define I2C_GENERAL_ERROR   0x10
#define I2C_TIMEOUT_ERROR    0x11
#define I2C_NACK_ERROR      0x12
#define I2C_CONFIG_ERROR    0x13

#endif
