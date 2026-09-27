#ifndef UART_DIAGNOSTICS_H
#define UART_DIAGNOSTICS_H
#include <stddef.h>
#include "uart_stats.h"
/* All measurements are owned here, not by the control tasks. */
enum motor_diag_reason {
    DIAG_BUILD_FAILED, DIAG_TX_TIMEOUT, DIAG_NO_RESPONSE, DIAG_HEADER_TIMEOUT,
    DIAG_BODY_TIMEOUT, DIAG_RX_ERROR, DIAG_BAD_LENGTH, DIAG_BAD_CHECKSUM,
    DIAG_PARSE_ERROR, DIAG_ID_OR_DATA_LENGTH, DIAG_SERVO_FAULT, DIAG_VALID_REPLY,
    DIAG_REASON_COUNT
};
typedef void (*uart_diag_sender_t)(void *, uint8_t, const uint8_t *, uint16_t);
volatile uart_stats_t *uart_diagnostics_port(unsigned port);
void uart_isr_timing_init(volatile uart_stats_t *s);
void uart_isr_timing_finish(volatile uart_stats_t *s, uint32_t start);
void motor_diagnostics_record(enum motor_diag_reason reason);
void motor_diagnostics_tx(const uint8_t *data, size_t length);
void motor_diagnostics_rx_byte(uint8_t byte);
void motor_diagnostics_masks(uint8_t motion, uint8_t current);
void uart_diagnostics_send(void *context, uart_diag_sender_t send);
#endif
