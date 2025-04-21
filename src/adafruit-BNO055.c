#include "adafruit-BNO055.h"
#include "i2c.h"

void bno055_init()
{
    uint8_t tmp;

    i2c1_init();

    // Read IDs
    i2c1_byte_read(BNO055_DEVICE_ADDR, BNO055_ID_CHIP, &tmp);
    i2c1_byte_read(BNO055_DEVICE_ADDR, BNO055_ID_ACC, &tmp);
    i2c1_byte_read(BNO055_DEVICE_ADDR, BNO055_ID_MAG, &tmp);
    i2c1_byte_read(BNO055_DEVICE_ADDR, BNO055_ID_GYR, &tmp);

    // Most default options from POR will be fine for our use
    // We will also not be using the fusion mode that the chip
    // provides, since it will be more fun to work with. If
    // things prove difficult, it can be introduced
}

void bno055_read(uint8_t reg, int n, uint8_t* data)
{
    i2c1_burst_read(BNO055_DEVICE_ADDR, reg, n, data);
}

void bno055_write(uint8_t reg, uint8_t value)
{
    uint8_t data = value;
    i2c1_burst_write(BNO055_DEVICE_ADDR, reg, 1, &data);
}

// --- DATA ---

void bno055_read_acc(uint16_t *data)
{
    bno055_read(BNO055_REG_ACC, 6, (uint8_t *)data);
}

void bno055_read_mag(uint16_t *data)
{
    bno055_read(BNO055_REG_MAG, 6, (uint8_t *)data);
}

void bno055_read_gyr(uint16_t *data)
{
    bno055_read(BNO055_REG_GYR, 6, (uint8_t *)data);
}

void bno055_read_eul(uint16_t *data)
{
    bno055_read(BNO055_REG_EUL, 6, (uint8_t *)data);
}

void bno055_read_qua(uint16_t *data)
{
    bno055_read(BNO055_REG_QUA, 6, (uint8_t *)data);
}

void bno055_read_lia(uint16_t *data)
{
    bno055_read(BNO055_REG_LIA, 6, (uint8_t *)data);
}

void bno055_read_grv(uint16_t *data)
{
    bno055_read(BNO055_REG_GRV, 6, (uint8_t *)data);
}

void bno055_read_temp(uint8_t *data)
{
    bno055_read(BNO055_REG_TEMP, 6, data);
}
