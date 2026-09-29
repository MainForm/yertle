#include "motor_uart.h"
#include "motor_dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart_diagnostics.h"

void motor_uart_init(void) { motor_dma_init(); }
void motor_uart_get_stats(uart_stats_t *s) { uart_diagnostics_get_stats(0U,s); }
void motor_uart_discard_rx(void) { motor_dma_stop(); }

bool motor_uart_write(const uint8_t *data, size_t length, uint32_t timeout_ms) {
    if (!data || !length) return false;
    motor_diagnostics_tx(data,length);
    if (!motor_dma_start(data,length)) {
        motor_diagnostics_record(DIAG_BUILD_FAILED); return false;
    }
    const TickType_t start=xTaskGetTickCount();
    TickType_t timeout=pdMS_TO_TICKS(timeout_ms);
    if (!timeout) timeout=1U;
    while (!motor_dma_tx_complete()) {
        if (motor_dma_failed()) {
            /* DMA TX failures already have a distinct transport counter. */
            if (!motor_dma_tx_failed()) motor_diagnostics_record(DIAG_RX_ERROR);
            motor_dma_stop(); return false;
        }
        if (xTaskGetTickCount()-start>=timeout) {
            uart_diagnostics_timeout(0U); motor_diagnostics_record(DIAG_TX_TIMEOUT);
            motor_dma_stop(); return false;
        }
        vTaskDelay(1U); /* DMA transfers while the owning task sleeps. */
    }
    return true;
}
static bool read_byte(uint8_t *byte, TickType_t start, TickType_t timeout, enum motor_diag_reason reason) {
    for (;;) {
        if (motor_dma_failed()) { motor_diagnostics_record(DIAG_RX_ERROR); return false; }
        if (motor_dma_read_byte(byte)) { motor_diagnostics_rx_byte(*byte); return true; }
        if (xTaskGetTickCount()-start>=timeout) {
            uart_diagnostics_timeout(0U); motor_diagnostics_record(reason); return false;
        }
        vTaskDelay(1U);
    }
}
bool motor_uart_read_frame(uint8_t *frame, size_t capacity, size_t *length, uint32_t timeout_ms) {
    if (!frame || !length || capacity<6U) return false;
    const TickType_t start=xTaskGetTickCount();
    TickType_t timeout=pdMS_TO_TICKS(timeout_ms);
    if (!timeout) timeout=1U;
    uint8_t byte=0;
    bool first_header=false, received_any=false;
    for (;;) {
        if (!read_byte(&byte,start,timeout,received_any ? DIAG_HEADER_TIMEOUT : DIAG_NO_RESPONSE)) return false;
        received_any=true;
        if (byte==0xffU) {
            if (first_header) { frame[0]=0xffU; frame[1]=0xffU; break; }
            first_header=true;
        } else first_header=false;
    }
    if (!read_byte(&frame[2],start,timeout,DIAG_BODY_TIMEOUT) ||
        !read_byte(&frame[3],start,timeout,DIAG_BODY_TIMEOUT)) return false;
    const size_t total=(size_t)frame[3]+4U;
    if (total<6U || total>capacity) { motor_diagnostics_record(DIAG_BAD_LENGTH); return false; }
    for (size_t i=4; i<total; ++i) if (!read_byte(&frame[i],start,timeout,DIAG_BODY_TIMEOUT)) return false;
    *length=total;
    return true;
}
