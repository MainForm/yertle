#include "debug_uart.h"

#include "stm32f1xx.h"

#define DEBUG_UART_BAUD 115200U

void debug_uart_init(void)
{
    SystemCoreClockUpdate();

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2: USART2_TX, alternate-function push-pull, 50 MHz. */
    GPIOA->CRL = (GPIOA->CRL & ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2)) |
                 GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1;
    /* PA3: USART2_RX, floating input. */
    GPIOA->CRL = (GPIOA->CRL & ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3)) |
                 GPIO_CRL_CNF3_0;

    USART2->BRR = (SystemCoreClock + (DEBUG_UART_BAUD / 2U)) / DEBUG_UART_BAUD;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void debug_uart_write(const char *text)
{
    while (*text != '\0') {
        while ((USART2->SR & USART_SR_TXE) == 0U) {
        }
        USART2->DR = (uint8_t) *text++;
    }
}

void debug_uart_write_line(const char *text)
{
    debug_uart_write(text);
    debug_uart_write("\r\n");
}

bool debug_uart_try_read(uint8_t *byte)
{
    if ((USART2->SR & USART_SR_RXNE) == 0U) {
        return false;
    }

    *byte = (uint8_t) USART2->DR;
    return true;
}
