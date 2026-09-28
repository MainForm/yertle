#include "uart_diagnostics.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "host_uart.h"
#include "motor_uart.h"

static volatile uart_stats_t ports[2];
static uint32_t reasons[DIAG_REASON_COUNT];
static uint32_t last_reason = 0xffffffffU;
static uint32_t masks;
static uint8_t tx_preview[16], rx_preview[16];
static uint32_t tx_length, rx_length;
static uint32_t snapshot_max_cycles;

/* Called only by the owning UART ISR; no RTOS calls or output here. */
void uart_diagnostics_rx_error(volatile uart_stats_t *s, uint32_t status, uint8_t byte)
{
    if (status & USART_SR_ORE) ++s->ore_errors;
    if (status & USART_SR_FE) ++s->fe_errors;
    if (status & USART_SR_NE) ++s->ne_errors;
    if (status & USART_SR_PE) ++s->pe_errors;
    s->last_error_sr = status;
    s->last_error_byte = byte;
    s->last_error_cycle = DWT->CYCCNT;
}

volatile uart_stats_t *uart_diagnostics_port(unsigned port) { return &ports[port]; }

void uart_isr_timing_init(volatile uart_stats_t *s)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    const uint32_t before = DWT->CYCCNT;
    __NOP(); __NOP(); __NOP(); __NOP();
    s->isr_timing_enabled = DWT->CYCCNT != before;
}

void uart_isr_timing_finish(volatile uart_stats_t *s, uint32_t start)
{
    const uint32_t elapsed = DWT->CYCCNT - start;
    ++s->isr_calls;
    if (s->isr_timing_enabled) {
        s->isr_cycles += elapsed;
        if (elapsed > s->isr_max_cycles) s->isr_max_cycles = elapsed;
    }
}

void motor_diagnostics_record(enum motor_diag_reason reason)
{
    taskENTER_CRITICAL();
    if (reason < DIAG_REASON_COUNT) { ++reasons[reason]; last_reason = reason; }
    taskEXIT_CRITICAL();
}

void motor_diagnostics_tx(const uint8_t *data, size_t length)
{
    taskENTER_CRITICAL();
    tx_length = (uint32_t) length;
    rx_length = 0U;
    memset(tx_preview, 0, sizeof(tx_preview));
    memset(rx_preview, 0, sizeof(rx_preview));
    memcpy(tx_preview, data, length < sizeof(tx_preview) ? length : sizeof(tx_preview));
    taskEXIT_CRITICAL();
}

void motor_diagnostics_rx_byte(uint8_t byte)
{
    taskENTER_CRITICAL();
    if (rx_length < sizeof(rx_preview)) rx_preview[rx_length] = byte;
    ++rx_length;
    taskEXIT_CRITICAL();
}

void motor_diagnostics_masks(uint8_t motion, uint8_t current)
{
    taskENTER_CRITICAL();
    masks = motion | ((uint32_t) current << 8U);
    taskEXIT_CRITICAL();
}

static void pack(uint8_t *output, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) output[i] = (uint8_t) (value >> (8U * i));
}

void uart_diagnostics_send(void *context, uart_diag_sender_t send)
{
    /* Copy all diagnostics before sending; never hold IRQ masking over TX. */
    uart_stats_t snapshot[2];
    uint32_t counters[DIAG_REASON_COUNT], last, valid, sr, txlen, rxlen, uptime;
    uint8_t tx[16], rx[16];
    uint32_t registers[5];
    taskENTER_CRITICAL();
    const uint32_t snapshot_start = DWT->CYCCNT;
    host_uart_get_stats(&snapshot[1]);
    motor_uart_get_stats(&snapshot[0]);
    memcpy(counters, reasons, sizeof(counters));
    last = last_reason; valid = masks; sr = USART1->SR;
    txlen = tx_length; rxlen = rx_length;
    memcpy(tx, tx_preview, sizeof(tx)); memcpy(rx, rx_preview, sizeof(rx));
    uptime = (uint32_t) (xTaskGetTickCount() * portTICK_PERIOD_MS);
    registers[0] = USART1->BRR; registers[1] = USART1->CR1;
    registers[2] = USART1->CR3; registers[3] = GPIOA->CRH;
    registers[4] = (NVIC->ISER[(unsigned) USART1_IRQn / 32U] >>
                    ((unsigned) USART1_IRQn % 32U)) & 1U;
    const uint32_t snapshot_cycles = DWT->CYCCNT - snapshot_start;
    if (snapshot_cycles > snapshot_max_cycles) snapshot_max_cycles = snapshot_cycles;
    taskEXIT_CRITICAL();
    uint8_t payload[64];
    pack(payload, uptime);
    for (unsigned p = 0; p < 2; ++p) {
        const uart_stats_t *s = &snapshot[p];
        const uint32_t fields[] = {s->tx_bytes, s->rx_bytes, s->rx_interrupts,
            s->rx_errors, s->rx_overflows, s->rx_peak, s->timeouts};
        for (unsigned i = 0; i < 7; ++i) pack(&payload[4 + p * 28 + i * 4], fields[i]);
    }
    send(context, 0x85U, payload, 60U);
    pack(payload, uptime); pack(&payload[4], SystemCoreClock);
    for (unsigned p = 0; p < 2; ++p) {
        const uart_stats_t *s = &snapshot[p];
        unsigned o = 8 + p * 16;
        pack(&payload[o], s->isr_calls); pack(&payload[o + 4], (uint32_t) s->isr_cycles);
        pack(&payload[o + 8], (uint32_t) (s->isr_cycles >> 32U));
        pack(&payload[o + 12], s->isr_max_cycles);
    }
    pack(&payload[40], snapshot[0].isr_timing_enabled | (snapshot[1].isr_timing_enabled << 1U));
    send(context, 0x86U, payload, 44U);
    pack(payload, uptime); pack(&payload[4], last); pack(&payload[8], valid);
    pack(&payload[12], sr);
    for (unsigned i = 0; i < DIAG_REASON_COUNT; ++i) pack(&payload[16 + i * 4], counters[i]);
    send(context, 0x87U, payload, 64U);
    pack(payload, uptime); pack(&payload[4], txlen); pack(&payload[8], rxlen);
    memcpy(&payload[12], tx, 16); memcpy(&payload[28], rx, 16);
    for (unsigned i = 0; i < 5; ++i) pack(&payload[44 + i * 4], registers[i]);
    send(context, 0x88U, payload, 64U);
    pack(payload, uptime);
    for (unsigned p = 0; p < 2; ++p) {
        const uart_stats_t *s = &snapshot[p];
        const uint32_t fields[] = {s->ore_errors, s->fe_errors, s->ne_errors,
            s->pe_errors, s->last_error_sr, s->last_error_byte, s->last_error_cycle};
        for (unsigned i = 0; i < 7; ++i) pack(&payload[4 + p * 28 + i * 4], fields[i]);
    }
    pack(&payload[60], snapshot_max_cycles);
    send(context, 0x89U, payload, 64U);
}
