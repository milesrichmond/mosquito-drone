#include "adafruit-BNO055.h"
#include "i2c.h"
#include "stm32f103x6.h"

/**
 *  Value set after init function is called.
 *  Allows for reconfiguation of the wiring without altering the driver code
 */
uint32_t BNO055_PORT_ADDR;

/**
 *  Initializes the corresponding i2c bus, and records it for later function calls
 *
 *  TODO: Setup an i2c_init that can just take an I2C_TypeDef so that this can
 *  adapt without any internal code changes.
 */
void bno055_init(uint32_t I2C_PORT_ADDR)
{
    BNO055_PORT_ADDR = I2C_PORT_ADDR;
}

/**
 *  Sets the operating sensors for the device
 */
void bno055_set_mode(bno055_mode_t mode)
{
    i2c_burst_write(BNO055_PORT_ADDR, BNO055_DEVICE_ADDR, BNO055_OPR_MODE_ADDR, 1, ((uint8_t *)&mode));
}

/**
 *  Sets the power mode for the device
 */
void bno055_set_pwr_mode(bno055_pwr_mode_t mode)
{
    i2c_burst_write(BNO055_PORT_ADDR, BNO055_DEVICE_ADDR, BNO055_PWR_MODE_ADDR, 1, ((uint8_t *)&mode));
}

/* SENSOR DATA */

/**
 *  Reads the specified vector from the device's sensors
 *
 *  !Ensure the device is in the correct operating mode for the desired sensor!
 */
void bno055_get_vector(bno055_vector_t vector_type, float *data)
{
    /* Read from the X_LSB to the Z_MSB of that particular set of sensor data */
    uint8_t buffer[6];
    i2c_burst_read(BNO055_PORT_ADDR, BNO055_DEVICE_ADDR, vector_type, 6, buffer);

    /* Read each set of bytes, orienting them such that the upper half (MSB)
     * is the leftmost set of bits. (MSB-LSB in a 16 bit value) */
    int16_t x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
    int16_t y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
    int16_t z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

    /* Convert to actual units */
    switch (vector_type) {
	case VEC_ACCEL: /* m/s/s */
	    data[0] =  x / 100.0f;
	    data[1] = ((float) y) / 100.0f;
	    data[2] = ((float) z) / 100.0f;
	    break;
	case VEC_MAGNETO: /* µT */
	    data[0] = ((float) x) / 16.0f;
	    data[1] = ((float) y) / 16.0f;
	    data[2] = ((float) z) / 16.0f;
	    break;
	case VEC_GYRO: /* DPS */
	    data[0] = ((float) x) / 16.0f;
	    data[1] = ((float) y) / 16.0f;
	    data[2] = ((float) z) / 16.0f;
	    break;
	case VEC_EULER: 
	    data[0] = ((float) x) / 16.0f;
	    data[1] = ((float) y) / 16.0f;
	    data[2] = ((float) z) / 16.0f;
	    break;
	case VEC_ACCEL_LINEAR: /* m/s/s */
	    data[0] = ((float) x) / 100.0f;
	    data[1] = ((float) y) / 100.0f;
	    data[2] = ((float) z) / 100.0f;
	    break;
	case VEC_GRAV: /* m/s/s */
	    data[0] = ((float) x) / 100.0f;
	    data[1] = ((float) y) / 100.0f;
	    data[2] = ((float) z) / 100.0f;
	    break;
    }
}

void bno055_get_quaternion(void)
{
    /* tbh, I don't think this will be something we will use.
     * Ultimately it should be implemented for a complete driver */
}

/**
 *  Reads from the device's temperature sensor
 *
 *  !Ensure the device is in the proper operating mode!
 */
void bno055_get_temp(int8_t *data)
{
   i2c_byte_read(BNO055_PORT_ADDR, BNO055_DEVICE_ADDR, BNO055_TEMP_DAT_ADDR, (uint8_t *)data); 
}
