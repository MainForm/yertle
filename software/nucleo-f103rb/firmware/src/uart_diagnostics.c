#include "uart_diagnostics.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

static volatile uart_isr_stats_t ports[2];
static volatile uint32_t tx_bytes[2], timeouts[2];
static volatile uint32_t rx_tail_bytes[2]; /* Task-owned DMA progress/residue. */
static volatile uint32_t rx_task_peak[2], rx_task_overflows[2];
static volatile uint32_t transport[16]; /* Per-field single writer; word 0 packed uptime. */
static uint64_t last_cycles[2]; /* Sole reader's cache; bounded retry fallback. */
static volatile uint32_t reasons[DIAG_REASON_COUNT];
static volatile uint32_t last_reason = 0xffffffffU;
static volatile uint32_t masks;
static volatile uint8_t tx_preview[16], rx_preview[16];
static volatile uint32_t tx_length, rx_length;
static uint32_t snapshot_max_cycles;

/* Called only by the owning UART ISR; no RTOS calls or output here. */
void uart_diagnostics_rx_error(volatile uart_isr_stats_t *s, uint32_t status, uint8_t byte)
{
    if (status & USART_SR_ORE) ++s->ore_errors;
    if (status & USART_SR_FE) ++s->fe_errors;
    if (status & USART_SR_NE) ++s->ne_errors;
    if (status & USART_SR_PE) ++s->pe_errors;
    s->last_error_sr = status;
    s->last_error_byte = byte;
    s->last_error_cycle = DWT->CYCCNT;
}

volatile uart_isr_stats_t *uart_diagnostics_port(unsigned port) { return &ports[port]; }
void uart_diagnostics_tx_byte(unsigned port) { ++tx_bytes[port]; }
void uart_diagnostics_tx_bytes(unsigned port, uint32_t count) { tx_bytes[port] += count; }
void uart_diagnostics_rx_tail_bytes(unsigned port, uint32_t count) { rx_tail_bytes[port] += count; }
void uart_diagnostics_rx_buffered(unsigned port, uint32_t count)
{ if (count > rx_task_peak[port]) rx_task_peak[port] = count; }
void uart_diagnostics_rx_overflow(unsigned port) { ++rx_task_overflows[port]; }
void uart_diagnostics_timeout(unsigned port) { ++timeouts[port]; }
void motor_transport_dma_mode(uint32_t capacity) { transport[1] = 1U; transport[12] = capacity; }
void motor_transport_record(enum motor_transport_event event) { ++transport[event]; }
void host_transport_rx_dma_mode(uint32_t capacity) { transport[13] = 1U; transport[14] = capacity; }
void host_transport_rx_dma_error(void) { ++transport[15]; }

void uart_diagnostics_get_stats(unsigned port, uart_stats_t *o)
{
    volatile uart_isr_stats_t *s = &ports[port];
    o->tx_bytes = tx_bytes[port]; o->timeouts = timeouts[port];
    o->rx_bytes = s->rx_bytes + rx_tail_bytes[port]; o->rx_interrupts = s->rx_interrupts;
    o->rx_errors = s->rx_errors;
    o->rx_overflows = s->rx_overflows + rx_task_overflows[port];
    const uint32_t task_peak = rx_task_peak[port], isr_peak = s->rx_peak;
    o->rx_peak = task_peak > isr_peak ? task_peak : isr_peak;
    o->isr_calls = s->isr_calls;
    /* Task cannot run halfway through its UART ISR on this single-core MCU.
     * If an ISR rolls low over between loads, retry high/low/high. Never mask
     * IRQs or wait on a writer flag; retain the last sample after 3 retries. */
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        uint32_t high = s->cycles_high;
        uint32_t low = s->cycles_low;
        if (high == s->cycles_high) {
            last_cycles[port] = ((uint64_t) high << 32U) | low;
            break;
        }
    }
    o->isr_cycles = last_cycles[port];
    o->isr_max_cycles = s->isr_max_cycles;
    o->isr_timing_enabled = s->isr_timing_enabled;
    o->ore_errors = s->ore_errors; o->fe_errors = s->fe_errors;
    o->ne_errors = s->ne_errors; o->pe_errors = s->pe_errors;
    /* Last-error tuple and trace are advisory: updates may occur between
     * individual loads. They are not a coherent packet/fault decision. */
    o->last_error_sr = s->last_error_sr;
    o->last_error_byte = s->last_error_byte;
    o->last_error_cycle = s->last_error_cycle;
}

void uart_isr_timing_init(volatile uart_isr_stats_t *s)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    const uint32_t before = DWT->CYCCNT;
    __NOP(); __NOP(); __NOP(); __NOP();
    s->isr_timing_enabled = DWT->CYCCNT != before;
}

void uart_isr_timing_finish(volatile uart_isr_stats_t *s, uint32_t start)
{
    const uint32_t elapsed = DWT->CYCCNT - start;
    ++s->isr_calls;
    if (s->isr_timing_enabled) {
        uint32_t low = s->cycles_low;
        s->cycles_low = low + elapsed;
        if (s->cycles_low < low) ++s->cycles_high;
        if (elapsed > s->isr_max_cycles) s->isr_max_cycles = elapsed;
    }
}

void motor_diagnostics_record(enum motor_diag_reason reason)
{
    if (reason < DIAG_REASON_COUNT) { ++reasons[reason]; last_reason = reason; }
}

void motor_diagnostics_tx(const uint8_t *data, size_t length)
{
    tx_length = (uint32_t) length;
    rx_length = 0U;
    for (unsigned i = 0; i < 16; ++i) {
        tx_preview[i] = i < length ? data[i] : 0U;
        rx_preview[i] = 0U;
    }
}

void motor_diagnostics_rx_byte(uint8_t byte)
{
    if (rx_length < sizeof(rx_preview)) rx_preview[rx_length] = byte;
    ++rx_length;
}

void motor_diagnostics_masks(uint16_t motion, uint16_t current)
{
    masks = motion | ((uint32_t) current << 16U);
}

static void pack(uint8_t *output, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) output[i] = (uint8_t) (value >> (8U * i));
}

void uart_diagnostics_send(void *context, uart_diag_sender_t send)
{
    /* Non-blocking SPSC reads. UART IRQs remain enabled throughout the copy.
     * Counters/trace can represent slightly different instants. */
    uart_stats_t snapshot[2];
    uint32_t counters[DIAG_REASON_COUNT], last, valid, sr, txlen, rxlen, uptime;
    uint8_t tx[16], rx[16];
    uint32_t registers[5];
    uint32_t transport_snapshot[16];
    const uint32_t snapshot_start = DWT->CYCCNT;
    uart_diagnostics_get_stats(1U, &snapshot[1]);
    uart_diagnostics_get_stats(0U, &snapshot[0]);
    for (unsigned i = 0; i < DIAG_REASON_COUNT; ++i) counters[i] = reasons[i];
    last = last_reason; valid = masks; sr = USART1->SR;
    txlen = tx_length; rxlen = rx_length;
    for (unsigned i = 0; i < 16; ++i) { tx[i] = tx_preview[i]; rx[i] = rx_preview[i]; }
    uptime = (uint32_t) (xTaskGetTickCount() * portTICK_PERIOD_MS);
    registers[0] = USART1->BRR; registers[1] = USART1->CR1;
    registers[2] = USART1->CR3; registers[3] = GPIOA->CRH;
    registers[4] = (NVIC->ISER[(unsigned) USART1_IRQn / 32U] >>
                    ((unsigned) USART1_IRQn % 32U)) & 1U;
    for (unsigned i = 1; i < 16; ++i) transport_snapshot[i] = transport[i];
    const uint32_t snapshot_cycles = DWT->CYCCNT - snapshot_start;
    if (snapshot_cycles > snapshot_max_cycles) snapshot_max_cycles = snapshot_cycles;
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
    pack(payload, uptime);
    for (unsigned i = 1; i < 16; ++i) pack(&payload[i*4], transport_snapshot[i]);
    send(context, 0x8AU, payload, 64U);
}
