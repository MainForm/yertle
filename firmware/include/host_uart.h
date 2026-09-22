#ifndef HOST_UART_H
#define HOST_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* USART2 transport for the binary PC/RPi host link. */
void host_uart_init(void);
void host_uart_write_bytes(const uint8_t *data, size_t length);
bool host_uart_try_read(uint8_t *byte);

#endif /* HOST_UART_H */
