/**
 ************************************************************************************************
 * @file    bmp390.h
 * @author  Miles Richmond (milesf.richmond@gmail.com)
 * @brief   Abstraction layer to interface with the BMP390 sensor from an stm32 microcontroller.
 ************************************************************************************************
 */

#ifndef BMP390_H
#define BMP390_H

#include <stdint.h>
#include "../util/error_types.h"
#include "../mcu/types.h"

#define BMP390_DEVICE_ADDRESS (0x77)

typedef enum {
    BMP390_PWR_NORMAL,
    BMP390_PWR_FORCED,
    BMP390_PWR_SLEEP,
} bmp390_pwr_mode_t;

typedef enum {
    BMP390_OPR_NONE,
    BMP390_OPR_PRES,
    BMP390_OPR_TEMP,
    BMP390_OPR_BOTH,
} bmp390_opr_mode_t;

/**
 * @brief Initializes the BMP390 for use.
 *
 * Stores the i2c port address that will be used in i2c communications. Additionally
 * requestes calibration information from the BMP390's NVM over i2c.
 *
 * @param i2c_port_address The port address that will be used in any subsequent i2c communications.
 *
 * @return Non-zero value if the initialization or i2c read operation were unsuccessful.
 */
error_t bmp390_init(const i2c_bus_t bus);

/**
 * @brief Configures the BMP390's power state.
 *
 * Writes the specified mode's corresponding value to the PWR_CTRL register of the BMP390.
 *
 * @param mode Which power state to configure to.
 *
 * @return Non-zero if an error occurs.
 */
error_t bmp390_set_pwr_mode(bmp390_pwr_mode_t mode);

/**
 * @brief Configures the BMP390's operation mode.
 *
 * Writes the specified mode's corresponding value to the PWR_CTRL register of the BMP390.
 *
 * @param mode Which operation mode to configure to.
 *
 * @return Non-zero if an error occurs.
 */
error_t bmp390_set_opr_mode(bmp390_opr_mode_t mode);

/**
 * @brief Reads from the BMP390's temperature sensor.
 *
 * Reads in the temperature data from the DATA_3, DATA_4, and DATA_5 registers of the BMP390.
 * Output precision will depend on the configured oversampling.
 *
 * @param data Where to write the sensor's output (in Celcius).
 *
 * @return Non-zero if an error occurs.
 */
error_t bmp390_read_temp_data(float *data);

/**
 * @brief Reads from the BMP390's pressure sensor.
 *
 * Reads in the pressure data from the DATA_0, DATA_1, and DATA_2 registers of the BMP390.
 * A temperature reading is required, from this sensor or otherwise, to provide a correct
 * reading.
 *
 * @param temperature The current temperature. Required for an accurate pressure reading.
 *
 * @param data Where to write the sensor's output (in mbar).
 *
 * @return Non-zero if an error occurs.
 */
error_t bmp390_read_pres_data(float temperature, float *data);

#endif
