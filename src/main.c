#include <stdint.h>

#include "stm32f103x6.h"
#include "bmp390.h"
#include "adafruit-BNO055.h"

#include "debug.h"
#include "util.h"
#include "i2c.h"

void bno055_setup(void)
{
    bno055_init(I2C1_BASE);
    dbg_log("[BNO055] initialised\n");

    bno055_set_pwr_mode(BNO055_PWR_NORMAL);
    dbg_log("[BNO055] normal power mode\n");

    bno055_set_mode(BNO055_MODE_NDOF);
    dbg_log("[BNO055] accelerometer only\n");
}

void bmp390_setup(void)
{
    bmp390_init(I2C1_BASE);
    dbg_log("[BMP390] initialised\n");

    bmp390_set_pwr(BMP390_PWR_NORMAL);
    dbg_log("[BMP390] normal power mode\n");

    bmp390_set_mode(BMP390_MODE_BOTH);
    dbg_log("[BMP390] temperature & pressure\n");
}

int main(void)
{
    initialise_monitor_handles();

    i2c1_init();
    i2c_scan_bus(I2C1);
    bno055_setup();
    //bmp390_setup();

    dbg_log("[Main] begin data polling...");
    uint8_t quat[8];

    while (1)
    {
	i2c_burst_read(I2C1_BASE, BNO055_DEVICE_ADDR, 0x20, 8, quat);

	int16_t w = ((uint16_t)quat[1]) << 8 | ((uint16_t)quat[0]);
	int16_t x = ((uint16_t)quat[3]) << 8 | ((uint16_t)quat[2]);
	int16_t y = ((uint16_t)quat[5]) << 8 | ((uint16_t)quat[4]);
	int16_t z = ((uint16_t)quat[7]) << 8 | ((uint16_t)quat[6]);

	dbg_log("w:%d     x:%d    ty:%d     z:%d\n", w, x, y, z);
    }

    return 0;
}
