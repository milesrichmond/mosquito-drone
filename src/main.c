#include <stdint.h>

#include "stm32f103x6.h"
#include "dbg.h"

int main(void)
{
    dbg_init();
    dbg_log("Hello");
    

    long thing = 0;

    while(1)
    {
	printf("Test: %ld\n", thing++);
	// UART needs to be implemented
    }

    return 0;
}
