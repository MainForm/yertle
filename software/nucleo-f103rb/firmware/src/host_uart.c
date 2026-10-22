#include "host_uart.h"

#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "board_clock.h"
#include "uart_diagnostics.h"
#include "host_rx_dma.h"

#define HOST_TX_TIMEOUT_MS 20U

void host_uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2: USART2_TX, alternate-function push-pull, 50 MHz. */
    GPIOA->CRL = (GPIOA->CRL & ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2)) |
                 GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1;
    /* PA3: USART2_RX, floating input. */
    GPIOA->CRL = (GPIOA->CRL & ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3)) |
                 GPIO_CRL_CNF3_0;

    USART2->BRR = (board_apb1_clock_hz() + (HOST_UART_BAUD / 2U)) / HOST_UART_BAUD;
    USART2->CR3 = 0U;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
    host_rx_dma_init();
}

static bool wait_tx_flag(uint32_t flag, TickType_t started)
{
    while ((USART2->SR & flag) == 0U) {
        if ((TickType_t)(xTaskGetTickCount() - started) >= pdMS_TO_TICKS(HOST_TX_TIMEOUT_MS)) {
            uart_diagnostics_timeout(1U);
            return false;
        }
    }
    return true;
}

bool host_uart_write_bytes(const uint8_t *data, size_t length)
{
    if (length == 0U) return true;
    if (data == NULL) return false;
    const TickType_t started = xTaskGetTickCount();
    for (size_t index = 0U; index < length; ++index) {
        if (!wait_tx_flag(USART_SR_TXE, started)) return false;
        USART2->DR = data[index];
        uart_diagnostics_tx_byte(1U);
    }
    /* TC includes the final stop bit; interrupts remain enabled while waiting. */
    return wait_tx_flag(USART_SR_TC, started);
}

bool host_uart_try_read(uint8_t *byte, bool *discontinuity)
{
    return host_rx_dma_try_read(byte, discontinuity);
}
