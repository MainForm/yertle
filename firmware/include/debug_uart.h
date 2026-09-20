#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdbool.h>
#include <stdint.h>

void debug_uart_init(void);
void debug_uart_write(const char *text);
void debug_uart_write_line(const char *text);
bool debug_uart_try_read(uint8_t *byte);

#endif /* DEBUG_UART_H */
