#include <stdint.h>

#include "stm32f103x6.h"
#include "adafruit-BNO055.h"

uint16_t acc_buffer[3];

int main(void)
{
   
    bno055_init();

    double accel_x, accel_y, accel_z;
    
    while(1)
    {
	bno055_read_acc(acc_buffer);

	accel_x = acc_buffer[0];
	accel_y = acc_buffer[1];
	accel_z = acc_buffer[2];

	(void)accel_x;
	(void)accel_y;
	(void)accel_z;

	// UART needs to be implemented
    }

    return 0;
}
