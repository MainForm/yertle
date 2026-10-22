#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host_uart.h"
#include "host_rx_dma.h"
#include "uart_diagnostics.h"

DWT_Type test_dwt;
CoreDebug_Type test_debug;
USART_Type test_usart, test_usart2;
GPIO_Type test_gpio;
NVIC_Type test_nvic;
RCC_Type test_rcc;
DMA_Type test_dma;
DMA_Channel_Type test_dma6;
uint32_t SystemCoreClock = 64000000U;

void DMA1_Channel6_IRQHandler(void);
void USART2_IRQHandler(void);
uint32_t board_apb1_clock_hz(void) { return 32000000U; }

static unsigned barrier_count, inject_at;
static bool deliver_pending_wrap;
static unsigned overwrite_at;
static void feed(uint8_t value, bool deliver_irq);
void host_rx_dma_test_clear_flags(uint32_t flags)
{
    test_dma.ISR &= ~(flags & DMA_IFCR_CGIF6 ? 15U << 20 : flags);
}
void host_rx_dma_test_barrier(void)
{
    if (++barrier_count == inject_at && deliver_pending_wrap) {
        deliver_pending_wrap = false;
        DMA1_Channel6_IRQHandler();
    }
    if (barrier_count == overwrite_at) {
        overwrite_at = 0U;
        for (unsigned i = 0U; i <= HOST_RX_DMA_CAPACITY; ++i) feed(0x5aU, true);
    }
}

static void feed(uint8_t value, bool deliver_irq)
{
    assert(test_dma6.CCR & DMA_CCR_EN);
    const uint32_t position = HOST_RX_DMA_CAPACITY - test_dma6.CNDTR;
    ((volatile uint8_t *)test_dma6.CMAR)[position] = value;
    --test_dma6.CNDTR;
    if (test_dma6.CNDTR == HOST_RX_DMA_CAPACITY / 2U) test_dma.ISR |= DMA_ISR_HTIF6;
    if (test_dma6.CNDTR == 0U) {
        test_dma6.CNDTR = HOST_RX_DMA_CAPACITY;
        test_dma.ISR |= DMA_ISR_TCIF6;
    }
    if (deliver_irq && test_dma.ISR) DMA1_Channel6_IRQHandler();
}
static bool read_byte(uint8_t *value, bool *lost) { return host_uart_try_read(value, lost); }
static uint32_t read32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }
static bool metadata_checked;
static void check_metadata(void *context, uint8_t type, const uint8_t *data, uint16_t size)
{
    (void)context;
    if (type != 0x8aU) return;
    assert(size == 64U && read32(data + 52U) == 1U);
    assert(read32(data + 56U) == HOST_RX_DMA_CAPACITY);
    assert(read32(data + 60U) == 1U);
    metadata_checked = true;
}

int main(void)
{
    host_uart_init();
    assert(USART2->BRR == 32U);
    assert((USART2->CR1 & (USART_CR1_RXNEIE | USART_CR1_IDLEIE)) == 0U);
    assert(USART2->CR3 & USART_CR3_DMAR);
    assert((test_dma6.CCR & (DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_EN)) ==
           (DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_EN));
    uint8_t value = 0U;
    bool lost;
    assert(!read_byte(&value, &lost) && !lost);
    /* Short packets are visible without IDLE or DMA boundary interrupts. */
    for (unsigned i = 0; i < 17U; ++i) feed((uint8_t)i, true);
    for (unsigned i = 0; i < 17U; ++i) {
        assert(read_byte(&value, &lost) && !lost && value == i);
    }
    uart_stats_t stats;
    uart_diagnostics_get_stats(1U, &stats);
    assert(stats.rx_bytes == 17U && stats.rx_peak == 17U && stats.isr_calls == 0U);
    /* Stream across several DMA wraps, retaining byte order and counts. */
    for (unsigned i = 17U; i < 3U * HOST_RX_DMA_CAPACITY; ++i) {
        feed((uint8_t)i, true);
        assert(read_byte(&value, &lost) && !lost && value == (uint8_t)i);
    }
    uart_diagnostics_get_stats(1U, &stats);
    assert(stats.rx_bytes == 3U * HOST_RX_DMA_CAPACITY && stats.rx_overflows == 0U);
    assert(stats.isr_calls == 6U); /* half and full, never per byte */
    /* A pending full IRQ must not hide a wrap or double-count after service. */
    for (unsigned i = 0U; i < HOST_RX_DMA_CAPACITY - 1U; ++i) {
        feed(0x42U, true); assert(read_byte(&value, &lost) && !lost);
    }
    feed(0x43U, false);
    assert(test_dma.ISR & DMA_ISR_TCIF6);
    assert(read_byte(&value, &lost) && !lost && value == 0x43U);
    DMA1_Channel6_IRQHandler();
    assert(!read_byte(&value, &lost) && !lost);
    /* Also exercise TC preemption during the task's lock-free snapshot. */
    for (unsigned i = 0U; i < HOST_RX_DMA_CAPACITY - 1U; ++i) {
        feed(0x44U, true); assert(read_byte(&value, &lost) && !lost);
    }
    feed(0x45U, false);
    inject_at = barrier_count + 1U; deliver_pending_wrap = true;
    assert(read_byte(&value, &lost) && !lost && value == 0x45U);
    assert(!deliver_pending_wrap);
    /* DMA overwrite drops the affected frame and resumes after delimiter. */
    for (unsigned i = 0U; i < HOST_RX_DMA_CAPACITY + 1U; ++i) feed(0x55U, true);
    assert(!read_byte(&value, &lost) && lost);
    feed(0xaaU, true); feed(0U, true); feed(0x66U, true);
    assert(read_byte(&value, &lost) && !lost && value == 0x66U);
    uart_diagnostics_get_stats(1U, &stats);
    assert(stats.rx_overflows == 1U && stats.rx_peak == HOST_RX_DMA_CAPACITY);
    /* UART hardware error disables RX until task recovery; no task parsing ISR. */
    feed(0x77U, true);
    USART2->SR = USART_SR_ORE | USART_SR_FE;
    USART2->DR = 0x80U;
    USART2_IRQHandler();
    assert(!(test_dma6.CCR & DMA_CCR_EN));
    assert(!read_byte(&value, &lost) && lost);
    USART2->SR = 0U; /* mock SR/DR clear */
    assert(test_dma6.CCR & DMA_CCR_EN);
    feed(0x88U, true); feed(0U, true); feed(0x99U, true);
    assert(read_byte(&value, &lost) && !lost && value == 0x99U);
    uart_diagnostics_get_stats(1U, &stats);
    assert(stats.ore_errors == 1U && stats.fe_errors == 1U && stats.rx_errors == 1U);
    /* DMA transfer error recovers through the same delimiter-resync path. */
    test_dma.ISR |= DMA_ISR_TEIF6;
    DMA1_Channel6_IRQHandler();
    assert(!read_byte(&value, &lost) && lost);
    feed(0U, true); feed(0xabU, true);
    assert(read_byte(&value, &lost) && !lost && value == 0xabU);
    uart_diagnostics_get_stats(1U, &stats);
    assert(stats.rx_errors == 2U && stats.rx_interrupts == 1U);
    /* Preemption after the copy must not publish an overwritten candidate. */
    feed(0x12U, true);
    overwrite_at = barrier_count + 2U;
    assert(!read_byte(&value, &lost) && lost);
    feed(0U, true); feed(0x13U, true);
    assert(read_byte(&value, &lost) && !lost && value == 0x13U);
    assert((USART2->CR1 & (USART_CR1_RXNEIE | USART_CR1_IDLEIE)) == 0U);
    assert(!read_byte(NULL, &lost) && !read_byte(&value, NULL));
    uart_diagnostics_send(NULL, check_metadata);
    assert(metadata_checked);
    puts("USART2 circular DMA tests passed: short bursts, wraps, pending/preempted TC, overwrite, UART/DMA recovery and metadata");
}
