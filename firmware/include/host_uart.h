#ifndef HOST_UART_H
#define HOST_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* PC/RPi and firmware must use the same 8N1 host-link baud rate. */
#define HOST_UART_BAUD 1000000U

/* USART2 transport for the binary PC/RPi host link. */
void host_uart_init(void);
/* Task-only bounded write. Success means UART wire completion, not host ACK. */
bool host_uart_write_bytes(const uint8_t *data, size_t length);
/* Task-only ring-buffer read. Discontinuity also reports loss without a byte. */
bool host_uart_try_read(uint8_t *byte, bool *discontinuity);

#endif /* HOST_UART_H */
