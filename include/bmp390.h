#ifndef BMP390_H
#define BMP390_H

#include <stdint.h>

#define BMP390_DEVICE_ADDR  (0x77)

typedef enum {

    /* SYS */

    BMP390_CHIP_ID_ADDR     = 0x00,
    BMP390_REV_ID_ADDR      = 0x01,
    BMP390_ERR_REG_ADDR     = 0x02,
    BMP390_STATUS_ADDR      = 0x03,
    BMP390_CMD_ADDR         = 0x7E,

    /* CONFIG */

    BMP390_IF_CONF_ADDR     = 0x1A,
    BMP390_PWR_CTRL_ADDR    = 0x1B,
    BMP390_OSR_ADDR         = 0x1C,
    BMP390_ODR_ADDR         = 0x1D,
    BMP390_CONFIG_ADDR      = 0x1F,
    BMP390_TRIM_COEF_ADDR   = 0x31,

    /* DATA */

    BMP390_PRES_DAT_LSB_ADDR    = 0x04,
    BMP390_PRES_DAT_MSB_ADDR    = 0x05,
    BMP390_PRES_DAT_XLSB_ADDR    = 0x06,

    BMP390_TEMP_DAT_LSB_ADDR    = 0x07,
    BMP390_TEMP_DAT_MSB_ADDR    = 0x08,
    BMP390_TEMP_DAT_XLSB_ADDR    = 0x09,

    /* FIFO */

    BMP390_FIFO_LENGTH_ADDR     = 0x12,
    BMP390_FIFO_DAT_ADDR        = 0x14,
    BMP390_FIFO_WATERMARK_ADDR  = 0x15,
    BMP390_FIFO_CONFIG_1_ADDR   = 0x17,
    BMP390_FIFO_CONFIG_2_ADDR   = 0x18,

    /* INTERRUPT */

    BMP390_INT_STATUS_ADDR  = 0x11,
    BMP390_INT_CTRL_ADDR    = 0x19,
} bmp390_reg_t;

/* MODES */

typedef enum {
    BMP390_MODE_NONE,
    BMP390_MODE_PRES,
    BMP390_MODE_TEMP,
    BMP390_MODE_BOTH
} bmp390_mode_t;

typedef enum {
    BMP390_PWR_SLEEP = 0U,
    BMP390_PWR_FORCED = 1U,
    BMP390_PWR_NORMAL = 3U,
} bmp390_pwr_mode_t;

/* INIT */

void bmp390_init(uint32_t I2C_PORT_ADDR);

/* CONFIG */

void bmp390_set_pwr(bmp390_pwr_mode_t mode);

void bmp390_set_mode(bmp390_mode_t mode);

/* BASIC READ */

void bmp390_get_temperature(float *data);

void bmp390_get_pressure(float *data);

/* FIFO */

void bmp390_fifo_enable(bmp390_mode_t mode);

void bmp390_fifo_read(uint8_t *data);

void bmp390_fifo_stop_on_full(uint8_t enabled);

#endif
