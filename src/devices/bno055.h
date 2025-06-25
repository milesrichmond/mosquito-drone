/**
 ************************************************************************************************
 * @file    bno055.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   Abstraction layer to interface with the BNO055 sensor from an stm32 microcontroller.
 ************************************************************************************************
 */

#ifndef BNO055_H
#define BNO055_H

#include <stdint.h>
#include "../util/error_types.h"

#define BNO055_DEVICE_ADDRESS (0x28)

typedef enum {
    BNO055_PWR_NORMAL,
    BNO055_PWR_LOW,
    BNO055_PWR_SUSPEND,
} bno055_pwr_mode_t;

typedef enum {
    /* Non-fusion modes */
    BNO055_OPR_CONFIG,
    BNO055_OPR_ACC,
    BNO055_OPR_MAG,
    BNO055_OPR_GYR,
    BNO055_OPR_ACCMAG,
    BNO055_OPR_ACCGYR,
    BNO055_OPR_MAGGYR,
    BNO055_OPR_AMG,

    /* Fusion modes */
    BNO055_OPR_IMU,
    BNO055_OPR_COMP,
    BNO055_OPR_M4G,
    BNO055_OPR_NDOF_FMC_OFF,
    BNO055_OPR_NDOF,
} bno055_opr_mode_t;

typedef enum {
    BNO055_DAT_ACCEL,       /* 3 bytes */
    BNO055_DAT_MAG,         /* 3 bytes */
    BNO055_DAT_GYR,         /* 3 bytes */
    BNO055_DAT_TEMP,        /* 1 byte  */

    /* Only available in fusion modes */
    BNO055_DAT_EULER,       /* 3 bytes */
    BNO055_DAT_QUAT,        /* 4 bytes */
    BNO055_DAT_LINACCEL,    /* 3 bytes */
    BNO055_DAT_GRAVITY,     /* 3 bytes */
} bno055_sensor_data_t;

/**
 * @brief Initializes the BNO055 for use.
 *
 * @param i2c_port_address The port address that will be used in any subsequent i2c communications.
 *
 * @return Non-zero value if the initialization is unsuccessful.
 */
error_t bno055_init(uint32_t i2c_port_address);

/**
 * @brief Configures the BNO055's power state.
 *
 * Writes the value corresponding to the specified mode to the PWR_MODE register of the BNO055.
 *
 * @param Which power state to configure to.
 *
 * @return Non-zero if an error occurs.
 */
error_t bno055_set_pwr_mode(bno055_pwr_mode_t mode);

/**
 * @brief Configures the BNO055's operation mode.
 *
 * Writes the value corresponding to the specified mode to the OPR_MODE register of the BNO055.
 *
 * @param Which operation mode to configure to.
 *
 * @return Non-zero if an error occurs.
 */
error_t bno055_set_opr_mode(bno055_opr_mode_t mode);

/**
 * @brief Remaps the sensors axies for different sensor orientations.
 *
 * Writes to the AXIS_MAP_CONFIG & AXIS_MAP_SIGN registers of the BNO055.
 * < em>Refer to the BNO055's data sheet for a detailed list of all possible configuration values< /em>
 *
 * @param remap_config Refer to the BNO055's documentation for supported orientations and their corresponding values.
 *
 * @param remap_sign Refer to the BNO055's documentation for supported orientations and their corresponding values.
 *
 * @return Non-zero if an error occurs.
 */
error_t bno055_remap_axis(char remap_config, char remap_sign);

/**
 * @brief Individually configues the accelerometer.
 *
 * Writes to the ACC_CONFIG register of the BNO055.
 * < b>The sensor cannot be individually configued in fusion modes< /b>
 * Additionally configurations must include:
 *  - G Range
 *  - Bandwidth
 *  - Power mode
 * < em>Refer to the BNO055's data sheet for a detailed list of all possible configuration values< /em>
 *
 * @param opr_mode The full configuration of the sensor.
 *
 * @return Non-zero if an error occurs.
 */
error_t bno055_acc_set_opr_mode(char opr_mode);

/**
 * @brief Individually configures the gyroscope.
 *
 * Writes to the GYR_CONFIG register of the BNO055.
 * < b>The sensor cannot be individually configued in fusion modes< /b>
 * Additionally configurations must include:
 *  - Range
 *  - Bandwidth
 *  - Power mode
 * < em>Refer to the BNO055's data sheet for a detailed list of all possible configuration values< /em>
 *
 * @param opr_mode The full configuration of the sensor.
 *
 * @return Non-zero if an error occurs.
 */
error_t bno055_gyr_set_opr_mode(char opr_mode);

/**
 * @brief Individually configures the magnetometer.
 *
 * Writes to the MAG_CONFIG of the BNO055.
 * < b>The sensor cannot be individually configued in fusion modes< /b>
 * Additionally configurations must include:
 *  - Data output rate
 *  - Operation mode
 *  - Power mode
 * < em>Refer to the BNO055's data sheet for a detailed list of all possible configuration values< /em>
 *
 * @param opr_mode The full configuration of the sensor.
 *
 * @return Non-zero if an error occurs.
 */
error_t bno055_mag_set_opr_mode(char opr_mode);

/**
 * @brief Configures the units the device will use for data output.
 *
 * Writes to the UNIT_SEL register of the BNO055 to change the units of the output data.
 * < em>Refer to the BNO055's data sheet for a detailed list of all possible configuration values< /em>
 *
 * @param unit_config The full configuration to be used by the device.
 *
 * @return Non-zero if an error occurs.
 */
error_t bno055_set_unit(char unit_config);

/**
 * @brief Reads the requested data type from the BNO055's sensors.
 *
 * Reads in the requested data from the BNO055's data output registers.
 * Some data types are not available if the device is not in fusion mode.
 *
 * @param type The type of data to be read.
 *
 * @param data A pointer to where the read data should be stored.
 */
error_t bno055_read_data(bno055_sensor_data_t type, float *data);

#endif
