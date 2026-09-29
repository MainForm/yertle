#ifndef UART_DIAGNOSTICS_H
#define UART_DIAGNOSTICS_H
#include <stddef.h>
#include "uart_stats.h"
/* Equal-priority UART/DMA ISRs are the RX/event producer; bus-mutex-serialized
 * tasks own TX/timeout counters and DMA stop residue in separate storage;
 * host_protocol_task is the diagnostic consumer. Snapshots are best effort,
 * not one globally simultaneous transaction. No diagnostic IRQ masking. */
enum motor_diag_reason {
    DIAG_BUILD_FAILED, DIAG_TX_TIMEOUT, DIAG_NO_RESPONSE, DIAG_HEADER_TIMEOUT,
    DIAG_BODY_TIMEOUT, DIAG_RX_ERROR, DIAG_BAD_LENGTH, DIAG_BAD_CHECKSUM,
    DIAG_PARSE_ERROR, DIAG_ID_OR_DATA_LENGTH, DIAG_SERVO_FAULT, DIAG_VALID_REPLY,
    DIAG_REASON_COUNT
};
typedef void (*uart_diag_sender_t)(void *, uint8_t, const uint8_t *, uint16_t);
volatile uart_isr_stats_t *uart_diagnostics_port(unsigned port);
void uart_diagnostics_get_stats(unsigned port, uart_stats_t *output);
void uart_diagnostics_tx_byte(unsigned port);
void uart_diagnostics_tx_bytes(unsigned port, uint32_t count);
void uart_diagnostics_rx_tail_bytes(unsigned port, uint32_t count);
void uart_diagnostics_timeout(unsigned port);
enum motor_transport_event {
    TRANSPORT_TX_STARTED = 2, TRANSPORT_TX_COMPLETE, TRANSPORT_RX_IDLE,
    TRANSPORT_RX_FULL, TRANSPORT_TX_DMA_ERROR, TRANSPORT_RX_DMA_ERROR,
    TRANSPORT_SYNC_REQUEST, TRANSPORT_SYNC_COMPLETE,
    TRANSPORT_FEEDBACK_CYCLE, TRANSPORT_FEEDBACK_COMPLETE
};
void motor_transport_dma_mode(uint32_t capacity);
void motor_transport_record(enum motor_transport_event event);
void uart_isr_timing_init(volatile uart_isr_stats_t *s);
void uart_isr_timing_finish(volatile uart_isr_stats_t *s, uint32_t start);
void uart_diagnostics_rx_error(volatile uart_isr_stats_t *s, uint32_t status, uint8_t byte);
void motor_diagnostics_record(enum motor_diag_reason reason);
void motor_diagnostics_tx(const uint8_t *data, size_t length);
void motor_diagnostics_rx_byte(uint8_t byte);
void motor_diagnostics_masks(uint16_t motion, uint16_t current);
void uart_diagnostics_send(void *context, uart_diag_sender_t send);
#endif
