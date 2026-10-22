#ifndef MOTOR_UART_H
#define MOTOR_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* USART1 on PA9(TX)/PA10(RX), connected to the UART side of the FE-URT module. */
void motor_uart_init(void);
void motor_uart_discard_rx(void);
bool motor_uart_write(const uint8_t *data, size_t length, uint32_t timeout_ms);
/* Task context, bus mutex held: assembles a reply from the RX DMA buffer.
 * Overflow/receive errors invalidate the transaction until discard_rx(). */
bool motor_uart_read_frame(uint8_t *frame, size_t capacity, size_t *length,
                           uint32_t timeout_ms);

#endif /* MOTOR_UART_H */
