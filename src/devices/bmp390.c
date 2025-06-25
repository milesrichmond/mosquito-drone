/**
 ************************************************************************************************
 * @file    bmp390.c
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   Abstraction layer to interface with the BMP390 sensor from an stm32 microcontroller.
 ************************************************************************************************
 */

#include "bmp390.h"

#include "../mcu/i2c.h"
#include "../mcu/types.h"
#include <math.h>

/*
 *******************************************************************************
 *
 * BMP390 Registers
 *
 *******************************************************************************
 */

typedef enum {
    /* SYS */

    CHIP_ID_ADDR     = 0x00,
    REV_ID_ADDR      = 0x01,
    ERR_REG_ADDR     = 0x02,
    STATUS_ADDR      = 0x03,
    CMD_ADDR         = 0x7E,

    /* CONFIG */

    IF_CONF_ADDR     = 0x1A,
    PWR_CTRL_ADDR    = 0x1B,
    OSR_ADDR         = 0x1C,
    ODR_ADDR         = 0x1D,
    CONFIG_ADDR      = 0x1F,
    TRIM_COEF_ADDR   = 0x31,

    /* DATA */

    PRES_DAT_LSB_ADDR    = 0x04,
    PRES_DAT_MSB_ADDR    = 0x05,
    PRES_DAT_XLSB_ADDR    = 0x06,

    TEMP_DAT_LSB_ADDR    = 0x07,
    TEMP_DAT_MSB_ADDR    = 0x08,
    TEMP_DAT_XLSB_ADDR    = 0x09,

    /* FIFO */

    FIFO_LENGTH_ADDR     = 0x12,
    FIFO_DAT_ADDR        = 0x14,
    FIFO_WATERMARK_ADDR  = 0x15,
    FIFO_CONFIG_1_ADDR   = 0x17,
    FIFO_CONFIG_2_ADDR   = 0x18,

    /* INTERRUPT */

    INT_STATUS_ADDR  = 0x11,
    INT_CTRL_ADDR    = 0x19,
} bmp390_reg_t;

/*
 *******************************************************************************
 *
 * Calibration Types
 *
 *******************************************************************************
 */

typedef struct {
    int8_t   P11;
    int8_t   P10;
    int16_t  P9;
    int8_t   P8;
    int8_t   P7;
    uint16_t P6;
    uint16_t P5;
    int8_t   P4;
    int8_t   P3;
    int16_t  P2;
    int16_t  P1;

    int8_t   T3;
    uint16_t T2;
    uint16_t T1;
} nvm_calibration_t;

typedef struct {
    double T[3];
    double P[11];
} calibration_t;

/*
 *******************************************************************************
 *
 * IMPLEMENTATION
 *
 *******************************************************************************
 */

i2c_bus_t i2c_bus;
calibration_t calib;
uint8_t ctrl_state = 0U;

error_t init_calibration(void)
{
    uint8_t coef[21];
    error_t err = i2c_read_burst(&i2c_bus, BMP390_DEVICE_ADDRESS, TRIM_COEF_ADDR, 21, coef);
    if (err)
    {
        return err;
    }

    nvm_calibration_t nvm;

    nvm.T1 = ((uint16_t)coef[0]) | ((uint16_t)coef[1] << 8);
    nvm.T2 = ((uint16_t)coef[2]) | ((uint16_t)coef[3] << 8);
    nvm.T3 = (int8_t)coef[4];
    nvm.P1 = ((int16_t)coef[5]) | ((int16_t)coef[6] << 8);
    nvm.P2 = ((int16_t)coef[7]) | ((int16_t)coef[8] << 8);
    nvm.P3 = (int8_t)coef[9];
    nvm.P4 = (int8_t)coef[10];
    nvm.P5 = ((uint16_t)coef[11]) | ((uint16_t)coef[12] << 8);
    nvm.P6 = ((uint16_t)coef[13]) | ((uint16_t)coef[14] << 8);
    nvm.P7 = (int8_t)coef[15];
    nvm.P8 = (int8_t)coef[16];
    nvm.P9 = ((int16_t)coef[16]) | ((int16_t)coef[17] << 8);
    nvm.P10 = (int8_t)coef[19];
    nvm.P11 = (int8_t)coef[20];

    /* Convert to floating point */

    calib.T[0] = nvm.T1 * 256;
    calib.T[1] = ((double)nvm.T2) / pow(2, 30); /* These values could be hard-coded, but at a severe loss of readablity */
    calib.T[2] = ((double)nvm.T3) / pow(2, 48);
    calib.P[0] = ((double)nvm.P1 - pow(2, 14)) / pow(2, 20);
    calib.P[1] = ((double)nvm.P2 - pow(2, 14)) / pow(2, 29);
    calib.P[2] = ((double)nvm.P3) / pow(2, 32);
    calib.P[3] = ((double)nvm.P4) / pow(2, 37);
    calib.P[4] = ((double)nvm.P5) * pow(2, 3);
    calib.P[5] = ((double)nvm.P6) / pow(2, 6);
    calib.P[6] = ((double)nvm.P7) / pow(2, 8);
    calib.P[7] = ((double)nvm.P8) / pow(2, 15);
    calib.P[8] = ((double)nvm.P9) / pow(2, 48);
    calib.P[9] = ((double)nvm.P10) / pow(2, 48);
    calib.P[10] = ((double)nvm.P11) / pow(2, 65); /* dividing by 36 quintillion ._. */

    return NO_ERROR;
}

error_t bmp390_init(const i2c_bus_t bus)
{
    i2c_bus = bus;
    return init_calibration();
}

error_t bmp390_set_pwr_mode(bmp390_pwr_mode_t mode)
{
    switch (mode)
    {
        case BMP390_PWR_NORMAL:
            ctrl_state |= (3U << 4);
            break;
        case BMP390_PWR_FORCED:
            ctrl_state &= ~(1U << 5);
            ctrl_state |= (1U << 4);
            break;
        case BMP390_PWR_SLEEP:
            ctrl_state &= ~(3U << 4);
            break;
        default:
            return INVALID_ARG;
    }

    return i2c_write(&i2c_bus, BMP390_DEVICE_ADDRESS, PWR_CTRL_ADDR, ctrl_state);
}

error_t bmp390_set_opr_mode(bmp390_opr_mode_t mode)
{
    switch (mode)
    {
        case BMP390_OPR_NONE:
            ctrl_state &= ~(3U << 0);
            break;
        case BMP390_OPR_PRES:
            ctrl_state |= (1U << 0);
            ctrl_state &= ~(1U << 1);
            break;
        case BMP390_OPR_TEMP:
            ctrl_state &= ~(1U << 0);
            ctrl_state |= (1U << 1);
            break;
        case BMP390_OPR_BOTH:
            ctrl_state |= (3U << 0);
            break;
        default:
            return INVALID_ARG;
    }

    return i2c_write(&i2c_bus, BMP390_DEVICE_ADDRESS, PWR_CTRL_ADDR, ctrl_state);
}

/**
 * The BMP390's user manual dictates the calibration operations that must be performed.
 * They are reflected in this implementation.
 */
error_t bmp390_read_temp_data(float *data)
{
    uint8_t buffer[3];
    error_t err = i2c_read_burst(&i2c_bus, BMP390_DEVICE_ADDRESS, TEMP_DAT_XLSB_ADDR, 3, buffer);
    if (err)
    {
        return err;
    }

    uint32_t raw_data = ((uint32_t)buffer[0]) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16);

    float part_1 = (float)(raw_data - calib.T[0]);
    float part_2 = (float)(part_1 * calib.T[1]);
    *data = part_2 + (part_1 * part_1) * calib.T[2];

    return NO_ERROR;
}

/**
 * The BMP390's user manual dictates the calibration operations that must be performed.
 * They are reflected in this implementation.
 */
error_t bmp390_read_pres_data(float temperature, float *data)
{
    uint8_t buffer[3];
    error_t err = i2c_read_burst(&i2c_bus, BMP390_DEVICE_ADDRESS, PRES_DAT_XLSB_ADDR, 3, buffer);
    if (err)
    {
        return err;
    }

    uint32_t raw_data = ((int32_t)buffer[0]) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16);

    float tmp_1, tmp_2, tmp_3;
    float part_1, part_2, part_3;
    float temperature_sq = temperature * temperature; /* reduce floating point multiplication */
    float temperature_cb = temperature_sq * temperature; /* reduce floating point multiplication */

    tmp_1 = calib.P[5] * temperature;
    tmp_2 = calib.P[6] * temperature_sq;
    tmp_3 = calib.P[7] * temperature_cb;
    part_1 = calib.P[4] + tmp_1 + tmp_2 + tmp_3;

    tmp_1 = calib.P[1] * temperature;
    tmp_2 = calib.P[2] * temperature_sq;
    tmp_3 = calib.P[3] * temperature_cb;
    part_2 = (float)raw_data * (calib.P[0] + tmp_1 + tmp_2 + tmp_3);

    tmp_1 = (float)raw_data * (float)raw_data;
    tmp_2 = calib.P[8] + calib.P[9] * temperature;
    tmp_3 = tmp_1 * tmp_2;
    part_3 = tmp_3 + ((float)raw_data * (float)raw_data * (float)raw_data) * calib.P[10];

    *data = part_1 + part_2 + part_3;

    return NO_ERROR;
}
