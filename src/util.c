#include "util.h"

#include <math.h>

/* Q8.8 fixed point for the time being */
int16_t approx_altitude(uint8_t pressure_hPa)
{
    return 0;
}

double noaa_altitude(float pressure_hPa)
{
    return 145366.45 * (1 - powf(pressure_hPa / 1013.25, 0.190284));
}
