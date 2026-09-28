#ifndef UART_STATS_H
#define UART_STATS_H
#include <stdint.h>
#include "stm32f1xx.h"
/* Cumulative since boot; 32-bit counters wrap. Bytes are UART register traffic. */
typedef struct {
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t rx_interrupts;
    uint32_t rx_errors;
    uint32_t rx_overflows;
    uint32_t rx_peak;
    uint32_t timeouts;
    uint32_t isr_calls;
    uint64_t isr_cycles;
    uint32_t isr_max_cycles;
    uint32_t isr_timing_enabled;
    uint32_t ore_errors, fe_errors, ne_errors, pe_errors;
    uint32_t last_error_sr, last_error_byte, last_error_cycle;
} uart_stats_t;

/* Sole writer: owning UART ISR (initialization occurs before IRQ enable).
 * Aligned 32-bit volatile loads/stores are indivisible on Cortex-M3.
 * Task-owned TX/timeout counters deliberately do not live here. */
typedef struct {
    uint32_t rx_bytes, rx_interrupts, rx_errors, rx_overflows, rx_peak;
    uint32_t isr_calls, cycles_low, cycles_high;
    uint32_t isr_max_cycles, isr_timing_enabled;
    uint32_t ore_errors, fe_errors, ne_errors, pe_errors;
    uint32_t last_error_sr, last_error_byte, last_error_cycle;
} uart_isr_stats_t;

#endif
