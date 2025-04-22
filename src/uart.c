#include "uart.h"
#include <stdint.h>


#define GPIOAEN	    (1U<<2)
#define UART1EN	    (1U<<14)

#define DBG_UART_BAUDRATE   (115200)
#define SYS_FREQ	    (72000000)
#define APB1_CLK	    (SYS_FREQ)
#define CR1_TE		    (1U<<3)
#define CR1_UE		    (1U<<13)
#define SR_TXE		    (1U<<7)

static void uart_set_baudrate(uint32_t periph_clk, uint32_t baudrate);
static void uart_write(int ch);

void dummy(void)
{
    uart_write(0);
}

void uart_init(void)
{
    RCC->APB1ENR |= GPIOAEN;

    /* Set to alternate function */
    // USARTx_TX -> Alt func push-pull (pg 166)
    // target pin PA9
    GPIOA->CRH |= (1U<<7);
    GPIOA->CRH &= ~(1U<<6);
    GPIOA->CRH |= (1U<<5);
    GPIOA->CRH |= (1U<<4);

    RCC->APB2ENR |= UART1EN;

    uart_set_baudrate(APB1_CLK, DBG_UART_BAUDRATE);

    /* configure tranfer direction */
    USART1->CR1 = CR1_TE;

    USART1->CR1 |= CR1_UE;
}

static void uart_write(int ch)
{
    while (!(USART1->SR & SR_TXE)){}

    USART1->DR = (ch & 0xFF);
}

static uint16_t compute_uart_bd(uint32_t periph_clk, uint32_t baudrate)
{
    return ((periph_clk + (baudrate / 2U)) / baudrate);
}

static void uart_set_baudrate(uint32_t periph_clk, uint32_t baudrate)
{
    USART1->BRR = compute_uart_bd(periph_clk, baudrate);
}
