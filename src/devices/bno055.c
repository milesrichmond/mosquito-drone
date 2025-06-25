/**
 *******************************************************************************
 * @file    bno055.c
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   Abstraction layer for BNO055 sensor for an stm32 microcontroller.
 *******************************************************************************
 */

#include "bno055.h"
#include "../mcu/i2c.c"

/**
 *******************************************************************************
 *  BNO055 Registers
 *******************************************************************************
 */
typedef enum {
    /* IDs */

    CHIP_ID_ADDR         = 0x00,
    ACCEL_REV_ID_ADDR    = 0x01,
    MAG_REV_ID_ADDR      = 0x02,
    GYRO_REV_ID_ADDR     = 0x03,
    SW_REV_ID_LSB_ADDR   = 0x04,
    SW_REV_ID_MSB_ADDR   = 0x05,
    BL_REV_ID_ADDR       = 0x06,

    PAGE_ID_ADDR = 0x07,

    /* DATA REGISTERS */

    ACCEL_DAT_X_LSB_ADDR = 0x08,
    ACCEL_DAT_X_MSB_ADDR = 0x09,
    ACCEL_DAT_Y_LSB_ADDR = 0x0A,
    ACCEL_DAT_Y_MSB_ADDR = 0x0B,
    ACCEL_DAT_Z_LSB_ADDR = 0x0C,
    ACCEL_DAT_Z_MSB_ADDR = 0x0D,

    MAG_DAT_X_LSB_ADDR   = 0x0E,
    MAG_DAT_X_MSB_ADDR   = 0x0F,
    MAG_DAT_Y_LSB_ADDR   = 0x10,
    MAG_DAT_Y_MSB_ADDR   = 0x11,
    MAG_DAT_Z_LSB_ADDR   = 0x12,
    MAG_DAT_Z_MSB_ADDR   = 0x13,

    GYRO_DAT_X_LSB_ADDR  = 0x14,
    GYRO_DAT_X_MSB_ADDR  = 0x15,
    GYRO_DAT_Y_LSB_ADDR  = 0x16,
    GYRO_DAT_Y_MSB_ADDR  = 0x17,
    GYRO_DAT_Z_LSB_ADDR  = 0x18,
    GYRO_DAT_Z_MSB_ADDR  = 0x19,

    EULER_DAT_H_LSB_ADDR = 0x1A,
    EULER_DAT_H_MSB_ADDR = 0x1B,
    EULER_DAT_R_LSB_ADDR = 0x1C,
    EULER_DAT_R_MSB_ADDR = 0x1D,
    EULER_DAT_P_LSB_ADDR = 0x1E,
    EULER_DAT_P_MSB_ADDR = 0x1F,

    QUAT_DAT_W_LSB_ADDR  = 0x20,
    QUAT_DAT_W_MSB_ADDR  = 0x21,
    QUAT_DAT_X_LSB_ADDR  = 0x22,
    QUAT_DAT_X_MSB_ADDR  = 0x23,
    QUAT_DAT_Y_LSB_ADDR  = 0x24,
    QUAT_DAT_Y_MSB_ADDR  = 0x25,
    QUAT_DAT_Z_LSB_ADDR  = 0x26,
    QUAT_DAT_Z_MSB_ADDR  = 0x27,

    LIA_DAT_X_LSB_ADDR   = 0x28,
    LIA_DAT_X_MSB_ADDR   = 0x29,
    LIA_DAT_Y_LSB_ADDR   = 0x2A,
    LIA_DAT_Y_MSB_ADDR   = 0x2B,
    LIA_DAT_Z_LSB_ADDR   = 0x2C,
    LIA_DAT_Z_MSB_ADDR   = 0x2D,

    GRAV_DAT_X_LSB_ADDR  = 0x2E,
    GRAV_DAT_X_MSB_ADDR  = 0x2F,
    GRAV_DAT_Y_LSB_ADDR  = 0x30,
    GRAV_DAT_Y_MSB_ADDR  = 0x31,
    GRAV_DAT_Z_LSB_ADDR  = 0x32,
    GRAV_DAT_Z_MSB_ADDR  = 0x33,

    TEMP_DAT_ADDR        = 0x34,

    /* STATUS REGISTERS */

    CALIB_STAT_ADDR      = 0x35,
    SELFTEST_RESULT_ADDR = 0x36,
    INTR_STAT_ADDR       = 0x37,

    SYS_CLK_STAT_ADDR    = 0x38,
    SYS_STAT_ADDR        = 0x39,
    SYS_ERR_ADDR         = 0x3A,

    /* UNIT SELECTION REGISTER */

    UNIT_SEL_ADDR = 0x3B,

    /* MODE REGISTERS */

    OPR_MODE_ADDR    = 0x3D,
    PWR_MODE_ADDR    = 0x3E,
    SYS_TRIGGER_ADDR = 0x3F,
    TEMP_SOURCE_ADDR = 0x40,

    /* AXIS REMAP REGISTERS */

    AXIS_MAP_CONFIG_ADDR = 0x41,
    AXIS_MAP_SIGN_ADDR   = 0x42,

    /* OFFSET REGISTERS */
    
    ACCEL_OFFSET_X_LSB_ADDR  = 0x55,
    ACCEL_OFFSET_X_MSB_ADDR  = 0x56,
    ACCEL_OFFSET_Y_LSB_ADDR  = 0x57,
    ACCEL_OFFSET_Y_MSB_ADDR  = 0x58,
    ACCEL_OFFSET_Z_LSB_ADDR  = 0x59,
    ACCEL_OFFSET_Z_MSB_ADDR  = 0x5A,
    
    MAG_OFFSET_X_LSB_ADDR    = 0x5B,
    MAG_OFFSET_X_MSB_ADDR    = 0x5C,
    MAG_OFFSET_Y_LSB_ADDR    = 0x5D,
    MAG_OFFSET_Y_MSB_ADDR    = 0x5E,
    MAG_OFFSET_Z_LSB_ADDR    = 0x5F,
    MAG_OFFSET_Z_MSB_ADDR    = 0x60,

    GYRO_OFFSET_X_LSB_ADDR   = 0x61,
    GYRO_OFFSET_X_MSB_ADDR   = 0x62,
    GYRO_OFFSET_Y_LSB_ADDR   = 0x63,
    GYRO_OFFSET_Y_MSB_ADDR   = 0x64,
    GYRO_OFFSET_Z_LSB_ADDR   = 0x65,
    GYRO_OFFSET_Z_MSB_ADDR   = 0x66,

    /* RADIUS REGISTERS */

    ACCEL_RADIUS_LSB_ADDR    = 0x67,
    ACCEL_RADIUS_MSB_ADDR    = 0x68,
    MAG_RADIUS_LSB_ADDR      = 0x69,
    MAG_RADIUS_MSB_ADDR      = 0x6A,
    
} bno055_reg_t;

/**
 *******************************************************************************
 *  IMPLEMENTATION
 *******************************************************************************
 */

uint32_t I2C_PORT_ADDR = 0;
bno055_opr_mode_t current_opr_mode; /*< Kept track of for error checking */

char is_fusion_mode(void)
{
    return (current_opr_mode == BNO055_OPR_IMU ||
        current_opr_mode == BNO055_OPR_COMP ||
        current_opr_mode == BNO055_OPR_M4G ||
        current_opr_mode == BNO055_OPR_NDOF_FMC_OFF ||
        current_opr_mode == BNO055_OPR_NDOF);
}

char sensor_data_available(bno055_sensor_data_t sensor)
{
    if (is_fusion_mode() || current_opr_mode == BNO055_OPR_AMG)
    {
    return 1;
    }

    switch (sensor)
    {
    case BNO055_DAT_ACCEL:
        return (current_opr_mode == BNO055_OPR_ACC ||
            current_opr_mode == BNO055_OPR_ACCMAG ||
            current_opr_mode == BNO055_OPR_ACCGYR);
    case BNO055_DAT_MAG:
        return (current_opr_mode == BNO055_OPR_MAG ||
            current_opr_mode == BNO055_OPR_ACCMAG ||
            current_opr_mode == BNO055_OPR_MAGGYR);
    case BNO055_DAT_GYR:
        return (current_opr_mode == BNO055_OPR_GYR ||
            current_opr_mode == BNO055_OPR_ACCGYR ||
            current_opr_mode == BNO055_OPR_MAGGYR);
    /* Temperature is tricky, since we have two sources. For now,
     * it will be constrained to having both sensors enabled.
     * In the future, the value of the TEMP_SOURCE register should
     * be tracked locally. */
    case BNO055_DAT_TEMP:
        return (current_opr_mode == BNO055_OPR_ACCGYR);
    default: /*< The default case should never happen */
        return 1;
    }
}

error_t bno055_init(uint32_t i2c_port_address)
{
    I2C_PORT_ADDR = i2c_port_address;

    /* check if i2c is valid */

    return NO_ERROR;
}

error_t bno055_set_pwr_mode(bno055_pwr_mode_t mode)
{
    /*
     * 1. i2c write call
     * 2. return the i2c function's error code (ideally zero)
     */

    uint8_t reg_value = 0x00;
    switch (mode)
    {
    case BNO055_PWR_NORMAL:
        reg_value = 0x00;
        break;
    case BNO055_PWR_LOW:
        reg_value = 0x01;
        break;
    case BNO055_PWR_SUSPEND:
        reg_value = 0x02;
        break;
    default:
        return INVALID_ARG;
    }

    return i2c_write(I2C_PORT_ADDR, BNO055_DEVICE_ADDRESS, PWR_MODE_ADDR, reg_value);
}

error_t bno055_set_opr_mode(bno055_opr_mode_t mode)
{
    /*
     * 1. i2c write call
     * 2. return the i2c function's error code (ideally zero)
     */

    uint8_t reg_value = 0x00;
    switch (mode)
    {
    
    /* Non-fusion modes */
    case BNO055_OPR_CONFIG:
        reg_value = 0x00;
        break;
    case BNO055_OPR_ACC:
        reg_value = 0x01;
        break;
    case BNO055_OPR_MAG:
        reg_value = 0x02;
        break;
    case BNO055_OPR_GYR:
        reg_value = 0x03;
        break;
    case BNO055_OPR_ACCMAG:
        reg_value = 0x04;
        break;
    case BNO055_OPR_ACCGYR:
        reg_value = 0x05;
        break;
    case BNO055_OPR_MAGGYR:
        reg_value = 0x06;
        break;
    case BNO055_OPR_AMG:
        reg_value = 0x07;
        break;

    /* Fusion modes */
    case BNO055_OPR_IMU:
        reg_value = 0x08;
        break;
    case BNO055_OPR_COMP:
        reg_value = 0x09;
        break;
    case BNO055_OPR_M4G:
        reg_value = 0x0A;
        break;
    case BNO055_OPR_NDOF_FMC_OFF:
        reg_value = 0x0B;
        break;
    case BNO055_OPR_NDOF:
        reg_value = 0x0C;
        break;

    default:
        return INVALID_ARG;
    }

    current_opr_mode = mode;

    return i2c_write(I2C_PORT_ADDR, BNO055_DEVICE_ADDRESS, OPR_MODE_ADDR, reg_value);
}

error_t bno055_remap_axis(char remap_config, char remap_sign)
{
    /*
     * Concatenate the configuration so that it can be writen in a burst.
     * Both registers are next to one another (AXIS_MAP_CONFIG=0x41, AXIS_MAP_SIGN=0x42)
     *
     * return the i2c function's error code
     */

    uint16_t concatenated_config = ((uint16_t)remap_config) | (((uint16_t)remap_sign) << 8);
    return i2c_write_burst(I2C_PORT_ADDR, BNO055_DEVICE_ADDRESS, AXIS_MAP_CONFIG_ADDR, 2, &concatenated_config);
}

error_t bno055_acc_set_opr_mode(char opr_mode)
{

}

error_t bno055_gyr_set_opr_mode(char opr_mode)
{

}

error_t bno055_mag_set_opr_mode(char opr_mode)
{

}

error_t bno055_set_unit(char unit_config)
{
    return i2c_write(I2C_PORT_ADDR, BNO055_DEVICE_ADDRESS, UNIT_SEL_ADDR, unit_config);
}

/**
 * Method to handle reading of the 4 byte long orientation quaternion
 * Requires fusion modes.
 *
 * 2^14 LSB = 1 Quaternion unit
 */
error_t bno055_read_quat(float *data)
{
    uint8_t buffer[8];
    error_t err = i2c_read_burst(I2C_PORT_ADDR, BNO055_DEVICE_ADDRESS, QUAT_DAT_W_LSB_ADDR, 8, buffer);

    if (err)
    {
        return err;
    }

    int16_t w = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
    int16_t x = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
    int16_t y = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);
    int16_t z = ((int16_t)buffer[6]) | (((int16_t)buffer[7]) << 8);

    data[0] = ((float)w) / (1U << 14);
    data[1] = ((float)x) / (1U << 14);
    data[2] = ((float)y) / (1U << 14);
    data[3] = ((float)z) / (1U << 14);

    return NO_ERROR;
}

error_t bno055_read_vector(uint8_t reg, float unit_conversion, float *data)
{
    uint8_t buffer[6];
    error_t err = i2c_read_burst(I2C_PORT_ADDR, BNO055_DEVICE_ADDRESS, reg, 6, buffer);

    if (err)
    {
        return err;
    }

    /* Read each set of bytes, orienting them such that the upper half (MSB)
     * is the leftmost set of bits. (MSB-LSB in a 16 bit value) 
     * 
     * Technically euler is h, r, p, but that distinction does not matter.
     */
    int16_t x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
    int16_t y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
    int16_t z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

    data[0] = x * unit_conversion;
    data[1] = y * unit_conversion;
    data[2] = z * unit_conversion;

    return NO_ERROR;
}

/**
 * There is no unit conversion required, as 1 LSB = 1 degree C.
 * If fahrenheit is desired, divide the result by two (2 LSB = 1 C).
 */
error_t bno055_read_temp(float *data)
{
    uint8_t buffer;
    error_t err = i2c_read(I2C_PORT_ADDR, BNO055_DEVICE_ADDRESS, TEMP_DAT_ADDR, &buffer);

    if (err)
    {
        return err;
    }

    *data = (float)buffer;

    return NO_ERROR;
}

/**
 * Performs error checks in the event the requested sensor type is not enabled.
 * Fusion modes will automatically enable all 3 sensors.
 */
error_t bno055_read_data(bno055_sensor_data_t type, float *data)
{
    if (!sensor_data_available(type))
    {
        return INVALID_STATE;
    }

    switch (type)
    {
    /* Special cases (assorted lengths) */
    case BNO055_DAT_QUAT:
        return bno055_read_quat(data);

    case BNO055_DAT_TEMP:
        return bno055_read_temp(data);
    
    /* Boring cases (3 byte vectors) */
    case BNO055_DAT_ACCEL:
        return bno055_read_vector(ACCEL_DAT_X_LSB_ADDR, 1 / 100.0f, data);
    
    case BNO055_DAT_MAG:
        return bno055_read_vector(MAG_DAT_X_LSB_ADDR, 1 / 16.0f, data);

    case BNO055_DAT_GYR:
        return bno055_read_vector(GYRO_DAT_X_LSB_ADDR, 1 / 16.0f, data);

    case BNO055_DAT_EULER:
        return bno055_read_vector(EULER_DAT_H_LSB_ADDR, 1/ 16.0f, data);

    case BNO055_DAT_LINACCEL:
        return bno055_read_vector(LIA_DAT_X_LSB_ADDR, 1 / 100.0f, data);

    case BNO055_DAT_GRAVITY:
        return bno055_read_vector(GRAV_DAT_X_LSB_ADDR, 1 / 100.0f, data);

    default:
        return INVALID_ARG;
    }
}
