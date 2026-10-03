#include "host_uart.h"

#include "stm32f1xx.h"

#include "board_clock.h"
#include "uart_diagnostics.h"

#define HOST_UART_BAUD 115200U
#define HOST_RX_CAPACITY 256U

static uint8_t rx_buffer[HOST_RX_CAPACITY];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool rx_discontinuity;
static volatile bool rx_resync;
static volatile uart_isr_stats_t *diagnostics;
#define stats (*diagnostics)

/* ISR owns production; the task disables this IRQ briefly while consuming. */
void USART2_IRQHandler(void)
{
    const uint32_t timing_start = DWT->CYCCNT;
    const uint32_t status = USART2->SR;
    if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE |
                   USART_SR_PE)) == 0U) {
        uart_isr_timing_finish(&stats, timing_start);
        return;
    }

    /* Reading SR then DR clears STM32F1 RXNE and receive error flags. */
    const uint8_t byte = (uint8_t) USART2->DR;
    ++stats.rx_interrupts;
    if ((status & USART_SR_RXNE) != 0U) { ++stats.rx_bytes; }
    if ((status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0U) {
        rx_tail = rx_head;
        ++stats.rx_errors;
        uart_diagnostics_rx_error(&stats, status, byte);
        rx_discontinuity = true;
        rx_resync = true;
        uart_isr_timing_finish(&stats, timing_start);
        return;
    }
    if (rx_resync) {
        if (byte == 0U) {
            rx_resync = false;
        }
        uart_isr_timing_finish(&stats, timing_start);
        return;
    }

    const uint16_t next = (uint16_t) ((rx_head + 1U) % HOST_RX_CAPACITY);
    if (next == rx_tail) {
        ++stats.rx_overflows;
        rx_tail = rx_head;
        rx_discontinuity = true;
        rx_resync = byte != 0U;
        uart_isr_timing_finish(&stats, timing_start);
        return;
    }
    rx_buffer[rx_head] = byte;
    __DMB();
    rx_head = next;
    const uint32_t used = (rx_head + HOST_RX_CAPACITY - rx_tail) % HOST_RX_CAPACITY;
    if (used > stats.rx_peak) { stats.rx_peak = used; }
    uart_isr_timing_finish(&stats, timing_start);
}

void host_uart_init(void)
{
    diagnostics = uart_diagnostics_port(1U);
    uart_isr_timing_init(&stats);
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    /* PA2: USART2_TX, alternate-function push-pull, 50 MHz. */
    GPIOA->CRL = (GPIOA->CRL & ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2)) |
                 GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1 | GPIO_CRL_CNF2_1;
    /* PA3: USART2_RX, floating input. */
    GPIOA->CRL = (GPIOA->CRL & ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3)) |
                 GPIO_CRL_CNF3_0;

    USART2->BRR = (board_apb1_clock_hz() + (HOST_UART_BAUD / 2U)) / HOST_UART_BAUD;
    rx_head = 0U;
    rx_tail = 0U;
    rx_discontinuity = false;
    rx_resync = false;
    NVIC_DisableIRQ(USART2_IRQn);
    NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_SetPriority(USART2_IRQn, 5U);
    USART2->CR3 = USART_CR3_EIE;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE |
                  USART_CR1_RXNEIE | USART_CR1_PEIE;
    NVIC_EnableIRQ(USART2_IRQn);
}

void host_uart_write_bytes(const uint8_t *data, size_t length)
{
    for (size_t index = 0U; index < length; ++index) {
        while ((USART2->SR & USART_SR_TXE) == 0U) {
        }
        USART2->DR = data[index];
        uart_diagnostics_tx_byte(1U);
    }
}

bool host_uart_try_read(uint8_t *byte, bool *discontinuity)
{
    if ((byte == NULL) || (discontinuity == NULL)) {
        return false;
    }
    NVIC_DisableIRQ(USART2_IRQn);
    __DMB();
    *discontinuity = rx_discontinuity;
    rx_discontinuity = false;
    const bool available = rx_tail != rx_head;
    if (available) {
        *byte = rx_buffer[rx_tail];
        rx_tail = (uint16_t) ((rx_tail + 1U) % HOST_RX_CAPACITY);
    }
    __DMB();
    NVIC_EnableIRQ(USART2_IRQn);
    return available;
}
