#include "util.h"

void msq_write(uint32_t target, uint8_t offset, uint8_t state)
{
    if (state) {
	target |= (1U<<offset);
    } else {
	target &= ~(1U<<offset);
    }
}

void msq_write16()


void mosquito_write16(uint16_t target, char offset, char *data)
{

}

void mosquito_write32(uint32_t target, char offset, char *data)
{

}
