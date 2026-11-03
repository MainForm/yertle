#ifndef MOTOR_UART_H
#define MOTOR_UART_H

#include "motor_bus_map.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* USART1 on PA9(TX)/PA10(RX), connected to the UART side of the FE-URT module. */
void motor_uart_init(motor_bus_id_t bus);
void motor_uart_discard_rx(motor_bus_id_t bus);
bool motor_uart_write(motor_bus_id_t bus, const uint8_t *data, size_t length, uint32_t timeout_ms);
/* Task context, bus mutex held: assembles a reply from the RX DMA buffer.
 * Overflow/receive errors invalidate the transaction until discard_rx(). */
bool motor_uart_read_frame(motor_bus_id_t bus, uint8_t *frame, size_t capacity, size_t *length,
                           uint32_t timeout_ms);

#endif /* MOTOR_UART_H */
