#ifndef ADAFRUIT_BNO055_H
#define ADAFRUIT_BNO055_H

#include <stdint.h>

#define BNO055_DEVICE_ADDR   (0x28)
#define BNO055_DEVICE_ADDR_2   (0x29)
#define BNO055_ID       (0xA0)

/**
 *  BNO055 Registers
 */
typedef enum {
    /* IDs */

    BNO055_CHIP_ID_ADDR         = 0x00,
    BNO055_ACCEL_REV_ID_ADDR    = 0x01,
    BNO055_MAG_REV_ID_ADDR      = 0x02,
    BNO055_GYRO_REV_ID_ADDR     = 0x03,
    BNO055_SW_REV_ID_LSB_ADDR   = 0x04,
    BNO055_SW_REV_ID_MSB_ADDR   = 0x05,
    BNO055_BL_REV_ID_ADDR       = 0x06,

    BNO055_PAGE_ID_ADDR = 0x07,

    /* DATA REGISTERS */

    BNO055_ACCEL_DAT_X_LSB_ADDR = 0x08,
    BNO055_ACCEL_DAT_X_MSB_ADDR = 0x09,
    BNO055_ACCEL_DAT_Y_LSB_ADDR = 0x0A,
    BNO055_ACCEL_DAT_Y_MSB_ADDR = 0x0B,
    BNO055_ACCEL_DAT_Z_LSB_ADDR = 0x0C,
    BNO055_ACCEL_DAT_Z_MSB_ADDR = 0x0D,

    BNO055_MAG_DAT_X_LSB_ADDR   = 0x0E,
    BNO055_MAG_DAT_X_MSB_ADDR   = 0x0F,
    BNO055_MAG_DAT_Y_LSB_ADDR   = 0x10,
    BNO055_MAG_DAT_Y_MSB_ADDR   = 0x11,
    BNO055_MAG_DAT_Z_LSB_ADDR   = 0x12,
    BNO055_MAG_DAT_Z_MSB_ADDR   = 0x13,

    BNO055_GYRO_DAT_X_LSB_ADDR  = 0x14,
    BNO055_GYRO_DAT_X_MSB_ADDR  = 0x15,
    BNO055_GYRO_DAT_Y_LSB_ADDR  = 0x16,
    BNO055_GYRO_DAT_Y_MSB_ADDR  = 0x17,
    BNO055_GYRO_DAT_Z_LSB_ADDR  = 0x18,
    BNO055_GYRO_DAT_Z_MSB_ADDR  = 0x19,

    BNO055_EULER_DAT_H_LSB_ADDR = 0x1A,
    BNO055_EULER_DAT_H_MSB_ADDR = 0x1B,
    BNO055_EULER_DAT_R_LSB_ADDR = 0x1C,
    BNO055_EULER_DAT_R_MSB_ADDR = 0x1D,
    BNO055_EULER_DAT_P_LSB_ADDR = 0x1E,
    BNO055_EULER_DAT_P_MSB_ADDR = 0x1F,

    BNO055_QUAT_DAT_W_LSB_ADDR  = 0x20,
    BNO055_QUAT_DAT_W_MSB_ADDR  = 0x21,
    BNO055_QUAT_DAT_X_LSB_ADDR  = 0x22,
    BNO055_QUAT_DAT_X_MSB_ADDR  = 0x23,
    BNO055_QUAT_DAT_Y_LSB_ADDR  = 0x24,
    BNO055_QUAT_DAT_Y_MSB_ADDR  = 0x25,
    BNO055_QUAT_DAT_Z_LSB_ADDR  = 0x26,
    BNO055_QUAT_DAT_Z_MSB_ADDR  = 0x27,

    BNO055_LIA_DAT_X_LSB_ADDR   = 0x28,
    BNO055_LIA_DAT_X_MSB_ADDR   = 0x29,
    BNO055_LIA_DAT_Y_LSB_ADDR   = 0x2A,
    BNO055_LIA_DAT_Y_MSB_ADDR   = 0x2B,
    BNO055_LIA_DAT_Z_LSB_ADDR   = 0x2C,
    BNO055_LIA_DAT_Z_MSB_ADDR   = 0x2D,

    BNO055_GRAV_DAT_X_LSB_ADDR  = 0x2E,
    BNO055_GRAV_DAT_X_MSB_ADDR  = 0x2F,
    BNO055_GRAV_DAT_Y_LSB_ADDR  = 0x30,
    BNO055_GRAV_DAT_Y_MSB_ADDR  = 0x31,
    BNO055_GRAV_DAT_Z_LSB_ADDR  = 0x32,
    BNO055_GRAV_DAT_Z_MSB_ADDR  = 0x33,

    BNO055_TEMP_DAT_ADDR        = 0x34,

    /* STATUS REGISTERS */

    BNO055_CALIB_STAT_ADDR      = 0x35,
    BNO055_SELFTEST_RESULT_ADDR = 0x36,
    BNO055_INTR_STAT_ADDR       = 0x37,

    BNO055_SYS_CLK_STAT_ADDR    = 0x38,
    BNO055_SYS_STAT_ADDR        = 0x39,
    BNO055_SYS_ERR_ADDR         = 0x3A,

    /* UNIT SELECTION REGISTER */

    BNO055_UNIT_SEL_ADDR = 0x3B,

    /* MODE REGISTERS */

    BNO055_OPR_MODE_ADDR    = 0x3D,
    BNO055_PWR_MODE_ADDR    = 0x3E,
    BNO055_SYS_TRIGGER_ADDR = 0x3F,
    BNO055_TEMP_SOURCE_ADDR = 0x40,

    /* AXIS REMAP REGISTERS */

    BNO055_AXIS_MAP_CONFIG_ADDR = 0x41,
    BNO055_AXIS_MAP_SIGN_ADDR   = 0x42,

    /* OFFSET REGISTERS */
    
    BNO055_ACCEL_OFFSET_X_LSB_ADDR  = 0x55,
    BNO055_ACCEL_OFFSET_X_MSB_ADDR  = 0x56,
    BNO055_ACCEL_OFFSET_Y_LSB_ADDR  = 0x57,
    BNO055_ACCEL_OFFSET_Y_MSB_ADDR  = 0x58,
    BNO055_ACCEL_OFFSET_Z_LSB_ADDR  = 0x59,
    BNO055_ACCEL_OFFSET_Z_MSB_ADDR  = 0x5A,
    
    BNO055_MAG_OFFSET_X_LSB_ADDR    = 0x5B,
    BNO055_MAG_OFFSET_X_MSB_ADDR    = 0x5C,
    BNO055_MAG_OFFSET_Y_LSB_ADDR    = 0x5D,
    BNO055_MAG_OFFSET_Y_MSB_ADDR    = 0x5E,
    BNO055_MAG_OFFSET_Z_LSB_ADDR    = 0x5F,
    BNO055_MAG_OFFSET_Z_MSB_ADDR    = 0x60,

    BNO055_GYRO_OFFSET_X_LSB_ADDR   = 0x61,
    BNO055_GYRO_OFFSET_X_MSB_ADDR   = 0x62,
    BNO055_GYRO_OFFSET_Y_LSB_ADDR   = 0x63,
    BNO055_GYRO_OFFSET_Y_MSB_ADDR   = 0x64,
    BNO055_GYRO_OFFSET_Z_LSB_ADDR   = 0x65,
    BNO055_GYRO_OFFSET_Z_MSB_ADDR   = 0x66,

    /* RADIUS REGISTERS */

    BNO055_ACCEL_RADIUS_LSB_ADDR    = 0x67,
    BNO055_ACCEL_RADIUS_MSB_ADDR    = 0x68,
    BNO055_MAG_RADIUS_LSB_ADDR      = 0x69,
    BNO055_MAG_RADIUS_MSB_ADDR      = 0x6A,
    
} bno055_reg_t;

/**
 *  Vector data types for the BNO055
 */
typedef enum {
    VEC_ACCEL = BNO055_ACCEL_DAT_X_LSB_ADDR,
    VEC_MAGNETO = BNO055_MAG_DAT_X_LSB_ADDR,
    VEC_GYRO = BNO055_GYRO_DAT_X_LSB_ADDR,
    VEC_EULER = BNO055_EULER_DAT_H_LSB_ADDR,
    VEC_ACCEL_LINEAR = BNO055_LIA_DAT_X_LSB_ADDR,
    VEC_GRAV = BNO055_GRAV_DAT_X_LSB_ADDR,
} bno055_vector_t;

/**
 * Operation modes for the BNO055
 */
typedef enum {
    BNO055_MODE_CONFIG      = 0x00,
    BNO055_MODE_ACCEL_ONLY  = 0x01,
    BNO055_MODE_MAG_ONLY    = 0x02,
    BNO055_MODE_GYRO_ONLY   = 0x03,
    BNO055_MODE_ACCEL_MAG   = 0x04,
    BNO055_MODE_ACCEL_GYRO  = 0x05,
    BNO055_MODE_MAG_GYRO    = 0x06,
    BNO055_MODE_AMG         = 0x07,

    /* FUSION MODES */
    BNO055_MODE_IMU     = 0x08,
    BNO055_MODE_COMPASS = 0x09,
    BNO055_MODE_M4G     = 0x0A,
    BNO055_MODE_NDOF_FMC_OFF    = 0x0B,
    BNO055_MODE_NDOF    = 0x0C,
} bno055_mode_t;

/**
 *  Power modes for the BNO055
 */
typedef enum {
    BNO055_PWR_NORMAL   = 0x00,
    BNO055_PWR_LOW      = 0x01,
    BNO055_PWR_SUSPEND  = 0x02,
} bno055_pwr_mode_t;


/**
 *  Initializes the corresponding i2c_port and records the port
 *  for later use.
 */
void bno055_init(uint32_t I2C_PORT_ADDR);

/* --- CONFIG --- */

/**
 *  Enables the specified sensors on the device
 */
void bno055_set_mode(bno055_mode_t mode);

/**
 *  Configures the device's power mode
 */
void bno055_set_pwr_mode(bno055_pwr_mode_t mode);

/* --- SENSOR DATA --- */

/**
 *  Reads the specified vector from the device's sensors
 *
 *  !Ensure the device is in the correct operating mode
 *  for the desired sensor!
 */
void bno055_get_vector(bno055_vector_t vector_type, float *data);

/**
 * UNIMPLEMENTED
 */
void bno055_get_quaternion(void);

/**
 *  Reads the device's temperature sensor
 *
 *  !Ensure the device is in the proper operating mode!
 */
void bno055_get_temp(int8_t *data);

#endif
