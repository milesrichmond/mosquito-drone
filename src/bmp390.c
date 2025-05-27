#include "bmp390.h"
#include "i2c.h"
#include <math.h>
#include<stdio.h>

/* This is the base address of the i2c port the BMP390 is connected to */
uint32_t BMP390_PORT_ADDR;

/* Calibration */

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
} calib_nvm_t;

typedef struct {
    calib_nvm_t nvm;
    double T[3];
    double P[11];
    float last_temp_f;
} calib_t;

calib_t calibration;
uint8_t pwr_ctrl_state = 0;

/* INIT */

void init_calibration(void)
{
    /* Set coefficients from nvm */
    uint8_t coefficients[21];
    i2c_burst_read(BMP390_PORT_ADDR, BMP390_DEVICE_ADDR, BMP390_TRIM_COEF_ADDR, 21, coefficients);
    
    /* NVM Constants */

    calibration.nvm.T1 = ((uint16_t)coefficients[0]) | ((uint16_t)coefficients[1] << 8);
    calibration.nvm.T2 = ((uint16_t)coefficients[2]) | ((uint16_t)coefficients[3] << 8);
    calibration.nvm.T3 = (int8_t)coefficients[4];

    calibration.nvm.P1 = ((int16_t)coefficients[5]) | ((int16_t)coefficients[6] << 8);
    calibration.nvm.P2 = ((int16_t)coefficients[7]) | ((int16_t)coefficients[8] << 8);
    calibration.nvm.P3 = (int8_t)coefficients[9];
    calibration.nvm.P4 = (int8_t)coefficients[10];
    calibration.nvm.P5 = ((uint16_t)coefficients[11]) | ((uint16_t)coefficients[12] << 8);
    calibration.nvm.P6 = ((uint16_t)coefficients[13]) | ((uint16_t)coefficients[14] << 8);
    calibration.nvm.P7 = (int8_t)coefficients[15];
    calibration.nvm.P8 = (int8_t)coefficients[16];
    calibration.nvm.P9 = ((int16_t)coefficients[17]) | ((int16_t)coefficients[18] << 8);
    calibration.nvm.P10 = (int8_t)coefficients[19];
    calibration.nvm.P11 = (int8_t)coefficients[20];
    
    /* FP Conversion */

    calibration.T[0] = calibration.nvm.T1 * 256;
    calibration.T[1] = ((double)calibration.nvm.T2) / pow(2, 30);
    calibration.T[2] = ((double)calibration.nvm.T3) / pow(2, 48);

    calibration.P[0] = (((double)calibration.nvm.P1) - pow(2, 14)) / pow(2, 20);
    calibration.P[1] = (((double)calibration.nvm.P2) - pow(2, 14)) / pow(2, 29);
    calibration.P[2] = ((double)calibration.nvm.P3) / pow(2, 32);
    calibration.P[3] = ((double)calibration.nvm.P4) / pow(2, 37);
    calibration.P[4] = ((double)calibration.nvm.P5) * pow(2, 3);
    calibration.P[5] = ((double)calibration.nvm.P6) / pow(2, 6);
    calibration.P[6] = ((double)calibration.nvm.P7) / pow(2, 8);
    calibration.P[7] = ((double)calibration.nvm.P8) / pow(2, 15);
    calibration.P[8] = ((double)calibration.nvm.P9) / pow(2, 48);
    calibration.P[9] = ((double)calibration.nvm.P10) / pow(2, 48);
    calibration.P[10] = ((double)calibration.nvm.P11) / pow(2, 65);

    /* Set default value for temperature */
    calibration.last_temp_f = 15.0f;
}

void bmp390_init(uint32_t I2C_PORT_ADDR)
{
    BMP390_PORT_ADDR = I2C_PORT_ADDR;

    init_calibration();
}


/* CONFIG */

void bmp390_set_pwr(bmp390_pwr_mode_t mode)
{
    switch (mode)
    {
	case BMP390_PWR_SLEEP:
	    pwr_ctrl_state &= ~(3U << 4);
	    break;
	case BMP390_PWR_FORCED:
	    pwr_ctrl_state &= ~(1U << 5);
	    pwr_ctrl_state |= (1U << 4);
	    break;
	case BMP390_PWR_NORMAL:
	    pwr_ctrl_state |= (3U << 4);
	    break;
    }

    i2c_burst_write(BMP390_PORT_ADDR, BMP390_DEVICE_ADDR, BMP390_PWR_CTRL_ADDR, 1, &pwr_ctrl_state);
}

void bmp390_set_mode(bmp390_mode_t mode)
{
    switch (mode)
    {
	case BMP390_MODE_NONE:
	    pwr_ctrl_state &= ~(3U << 0);
	    break;
	case BMP390_MODE_PRES:
	    pwr_ctrl_state |= (1U << 0);
	    pwr_ctrl_state &= ~(1U << 1);
	case BMP390_MODE_TEMP:
	    pwr_ctrl_state |= (1U << 1);
	    pwr_ctrl_state &= ~(1U << 0);
	    break;
	case BMP390_MODE_BOTH:
	    pwr_ctrl_state |= (3U << 0);
	    break;
    }

    i2c_burst_write(BMP390_PORT_ADDR, BMP390_DEVICE_ADDR, BMP390_PWR_CTRL_ADDR, 1, &pwr_ctrl_state);
}

/* BASIC READ */

void bmp390_get_temperature(float *data)
{
    uint8_t buffer[3];
    i2c_burst_read(BMP390_PORT_ADDR, BMP390_DEVICE_ADDR, BMP390_TEMP_DAT_XLSB_ADDR, 3, buffer);

    uint32_t raw_data = ((uint32_t)buffer[2]) | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[0] << 16);

    /* The following conversion is closely modeled after the example provided in the
     * BMP390 data sheet.
     */
    float partial_1 = (float)(raw_data - calibration.T[0]);
    float partial_2 = (float)(partial_1 * calibration.T[1]);
    calibration.last_temp_f = partial_2 + (partial_1 * partial_1) * calibration.T[2];
    *data = calibration.last_temp_f;
}

void bmp390_get_pressure(float *data)
{
    uint8_t buffer[3];
    i2c_burst_read(BMP390_PORT_ADDR, BMP390_DEVICE_ADDR, BMP390_PRES_DAT_XLSB_ADDR, 3, buffer);
    int32_t raw_data = ((int32_t)buffer[2]) | (((int32_t)buffer[1]) << 8) | (((int32_t)buffer[0]) << 16);
    
    /* The following conversion is closely modeled after the example provided in the
     * BMP390 data sheet.
     */
    float partial_1, partial_2, partial_3, partial_4;
    float partial_out1, partial_out2;

    float temp = calibration.last_temp_f;
    partial_1 = calibration.P[5] * temp;
    partial_2 = calibration.P[6] * (temp * temp);
    partial_3 = calibration.P[7] * (temp * temp * temp);
    partial_out1 = calibration.P[4] + partial_1 + partial_2 + partial_3;

    partial_1 = calibration.P[1] * temp;
    partial_2 = calibration.P[2] * (temp * temp);
    partial_3 = calibration.P[3] * (temp * temp * temp);
    partial_out2 = (float)raw_data * (calibration.P[0] + partial_1 + partial_2 + partial_3);

    partial_1 = (float)raw_data * (float)raw_data;
    partial_2 = calibration.P[8] + calibration.P[9] * calibration.last_temp_f;
    partial_3 = partial_1 * partial_2;
    partial_4 = partial_3 + ((float)raw_data * (float)raw_data * (float)raw_data) * calibration.P[10];
    *data = partial_out1 + partial_out2 + partial_4;
}

/* FIFO */
/* These are primarily unimplemented, since we have not fully established
 * our requirements for this board.
 */

void bmp390_fifo_enable(bmp390_mode_t mode)
{

}

void bmp390_fifo_read(uint8_t *data)
{

}

void bmp390_fifo_stop_on_full(uint8_t enabled)
{

}
