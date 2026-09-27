#include "motor_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx.h"

#include "board_clock.h"

#define MOTOR_UART_BAUD 1000000U

static bool wait_for_flag(uint32_t flag, uint32_t timeout_ms)
{
    const TickType_t start = xTaskGetTickCount();
    const TickType_t timeout = pdMS_TO_TICKS(timeout_ms);

    while ((USART1->SR & flag) == 0U) {
        if ((xTaskGetTickCount() - start) >= timeout) {
            return false;
        }
    }
    return true;
}

static bool read_byte(uint8_t *byte, TickType_t start, TickType_t timeout)
{
    while ((USART1->SR & USART_SR_RXNE) == 0U) {
        if ((USART1->SR & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0U) {
            volatile uint32_t clear = USART1->SR;
            clear = USART1->DR;
            (void) clear;
        }
        if ((xTaskGetTickCount() - start) >= timeout) {
            return false;
        }
    }
    *byte = (uint8_t) USART1->DR;
    return true;
}

void motor_uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    /* PA9: USART1_TX, alternate-function push-pull, 50 MHz. */
    GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9)) |
                 GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1;
    /* PA10: USART1_RX, floating input. */
    GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10)) |
                 GPIO_CRH_CNF10_0;

    USART1->BRR = (board_apb2_clock_hz() + (MOTOR_UART_BAUD / 2U)) /
                  MOTOR_UART_BAUD;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    motor_uart_discard_rx();
}

void motor_uart_discard_rx(void)
{
    while ((USART1->SR & USART_SR_RXNE) != 0U) {
        volatile uint32_t discard = USART1->DR;
        (void) discard;
    }
}

bool motor_uart_write(const uint8_t *data, size_t length, uint32_t timeout_ms)
{
    if ((data == NULL) || (length == 0U)) {
        return false;
    }

    for (size_t index = 0U; index < length; ++index) {
        if (!wait_for_flag(USART_SR_TXE, timeout_ms)) {
            return false;
        }
        USART1->DR = data[index];
    }
    return wait_for_flag(USART_SR_TC, timeout_ms);
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
    for (;;) {
        if (!read_byte(&byte, start, timeout)) {
            return false;
        }
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

    if (!read_byte(&frame[2], start, timeout) ||
        !read_byte(&frame[3], start, timeout)) {
        return false;
    }

    const size_t total = (size_t) frame[3] + 4U;
    if ((total < 6U) || (total > capacity)) {
        return false;
    }
    for (size_t index = 4U; index < total; ++index) {
        if (!read_byte(&frame[index], start, timeout)) {
            return false;
        }
    }
    *length = total;
    return true;
}
