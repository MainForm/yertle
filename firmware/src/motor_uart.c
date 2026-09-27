#include "motor_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx.h"

#include "board_clock.h"
#include "uart_diagnostics.h"

#define MOTOR_UART_BAUD 1000000U
#define MOTOR_RX_CAPACITY 256U

static uint8_t rx_buffer[MOTOR_RX_CAPACITY];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool rx_failed;
static volatile uart_stats_t *diagnostics;
#define stats (*diagnostics)

void motor_uart_get_stats(uart_stats_t *output)
{
    taskENTER_CRITICAL();
    *output = stats;
    taskEXIT_CRITICAL();
}

void USART1_IRQHandler(void)
{
    const uint32_t timing_start = DWT->CYCCNT;
    const uint32_t status = USART1->SR;
    if ((status & (USART_SR_RXNE | USART_SR_ORE | USART_SR_FE | USART_SR_NE |
                   USART_SR_PE)) == 0U) {
        uart_isr_timing_finish(&stats, timing_start);
        return;
    }

    /* SR then DR clears receive/error flags on STM32F1. */
    const uint8_t byte = (uint8_t) USART1->DR;
    ++stats.rx_interrupts;
    if ((status & USART_SR_RXNE) != 0U) { ++stats.rx_bytes; }
    if ((status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0U) {
        rx_failed = true;
        ++stats.rx_errors;
    }
    if (rx_failed) {
        uart_isr_timing_finish(&stats, timing_start);
        return;
    }

    const uint16_t next = (uint16_t) ((rx_head + 1U) % MOTOR_RX_CAPACITY);
    if (next == rx_tail) {
        ++stats.rx_overflows;
        rx_failed = true;
        uart_isr_timing_finish(&stats, timing_start);
        return;
    }
    rx_buffer[rx_head] = byte;
    __DMB();
    rx_head = next;
    const uint32_t used = (rx_head + MOTOR_RX_CAPACITY - rx_tail) % MOTOR_RX_CAPACITY;
    if (used > stats.rx_peak) { stats.rx_peak = used; }
    uart_isr_timing_finish(&stats, timing_start);
}

static bool wait_for_flag(uint32_t flag, uint32_t timeout_ms)
{
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

    while ((USART1->SR & flag) == 0U) {
        if ((xTaskGetTickCount() - start) >= timeout) {
            ++stats.timeouts;
            return false;
        }
    }
    return true;
}

static bool read_byte(uint8_t *byte, TickType_t start, TickType_t timeout,
                      enum motor_diag_reason timeout_reason)
{
    for (;;) {
        NVIC_DisableIRQ(USART1_IRQn);
        __DMB();
        const bool failed = rx_failed;
        const bool available = rx_tail != rx_head;
        if (!failed && available) {
            *byte = rx_buffer[rx_tail];
            rx_tail = (uint16_t) ((rx_tail + 1U) % MOTOR_RX_CAPACITY);
        }
        __DMB();
        NVIC_EnableIRQ(USART1_IRQn);
        if (failed) {
            motor_diagnostics_record(DIAG_RX_ERROR);
            return false;
        }
        if (available) {
            motor_diagnostics_rx_byte(*byte);
            return true;
        }
        if ((xTaskGetTickCount() - start) >= timeout) {
            ++stats.timeouts;
            motor_diagnostics_record(timeout_reason);
            return false;
        }
        /* RX continues in the ISR while this task yields its CPU. */
        vTaskDelay(1U);
    }
}

void motor_uart_init(void)
{
    diagnostics = uart_diagnostics_port(0U);
    uart_isr_timing_init(&stats);
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    /* PA9: USART1_TX, alternate-function push-pull, 50 MHz. */
    GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9)) |
                 GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1;
    /* PA10: USART1_RX, floating input. */
    GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10)) |
                 GPIO_CRH_CNF10_0;

    USART1->BRR = (board_apb2_clock_hz() + (MOTOR_UART_BAUD / 2U)) /
                  MOTOR_UART_BAUD;
    NVIC_DisableIRQ(USART1_IRQn);
    rx_head = 0U;
    rx_tail = 0U;
    rx_failed = false;
    NVIC_ClearPendingIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 5U);
    USART1->CR3 = USART_CR3_EIE;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE |
                  USART_CR1_RXNEIE | USART_CR1_PEIE;
    motor_uart_discard_rx();
}

void motor_uart_discard_rx(void)
{
    NVIC_DisableIRQ(USART1_IRQn);
    /* Called under the bus mutex before a new request, never during replies. */
    volatile uint32_t discard = USART1->SR;
    discard = USART1->DR;
    (void) discard;
    rx_tail = rx_head;
    rx_failed = false;
    NVIC_ClearPendingIRQ(USART1_IRQn);
    __DMB();
    NVIC_EnableIRQ(USART1_IRQn);
}

bool motor_uart_write(const uint8_t *data, size_t length, uint32_t timeout_ms)
{
    if ((data == NULL) || (length == 0U)) {
        return false;
    }

    motor_diagnostics_tx(data, length);
    for (size_t index = 0U; index < length; ++index) {
        if (!wait_for_flag(USART_SR_TXE, timeout_ms)) {
            motor_diagnostics_record(DIAG_TX_TIMEOUT);
            return false;
        }
        USART1->DR = data[index];
        ++stats.tx_bytes;
    }
    const bool completed = wait_for_flag(USART_SR_TC, timeout_ms);
    if (!completed) motor_diagnostics_record(DIAG_TX_TIMEOUT);
    return completed;
}

bool motor_uart_read_frame(uint8_t *frame, size_t capacity, size_t *length,
                           uint32_t timeout_ms)
{
    if ((frame == NULL) || (length == NULL) || (capacity < 6U)) {
        return false;
    }

    const TickType_t start = xTaskGetTickCount();
    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    if (timeout == 0U) {
        timeout = 1U;
    }

    uint8_t byte = 0U;
    bool first_header = false;
    bool received_any = false;
    for (;;) {
        if (!read_byte(&byte, start, timeout,
                       received_any ? DIAG_HEADER_TIMEOUT : DIAG_NO_RESPONSE)) {
            return false;
        }
        received_any = true;
        if (byte == 0xFFU) {
            if (first_header) {
                frame[0] = 0xFFU;
                frame[1] = 0xFFU;
                break;
            }
            first_header = true;
        } else {
            first_header = false;
        }
    }

    if (!read_byte(&frame[2], start, timeout, DIAG_BODY_TIMEOUT) ||
        !read_byte(&frame[3], start, timeout, DIAG_BODY_TIMEOUT)) {
        return false;
    }

    const size_t total = (size_t) frame[3] + 4U;
    if ((total < 6U) || (total > capacity)) {
        motor_diagnostics_record(DIAG_BAD_LENGTH);
        return false;
    }
    for (size_t index = 4U; index < total; ++index) {
        if (!read_byte(&frame[index], start, timeout, DIAG_BODY_TIMEOUT)) {
            return false;
        }
    }
    *length = total;
    return true;
}
