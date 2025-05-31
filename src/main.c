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

    bno055_set_mode(BNO055_MODE_ACCEL_ONLY);
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
    bmp390_setup();

    dbg_log("[Main] begin data polling...");
    float acc_buffer[3];
    long poll_count = 0;
    float temperature = 0.0f;
    float pressure = 0.0f;
    float altitude = 0.0f;

    while (1)
    {
	bno055_get_vector(VEC_ACCEL, acc_buffer);

	bmp390_get_temperature(&temperature);
	bmp390_get_pressure(&pressure);

	//printf("%ld:\t x: %.2f, y: %.2f, z: %.2f\n", poll_count, acc_buffer[0], acc_buffer[1], acc_buffer[2]);
	//printf("%ld:\t %f degrees\n", poll_count, temperature);
	//printf("\t%f Pa (%f mbar)\n", pressure, pressure / 100.0f);

	altitude = noaa_altitude(pressure / 100);

	dbg_log("Altitude (NOAA): %lf ft\n", altitude);
	poll_count++;
    }

    return 0;
}
