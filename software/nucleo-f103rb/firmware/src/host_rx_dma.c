#include "host_rx_dma.h"
#include <stddef.h>
#include "stm32f1xx.h"
#include "uart_diagnostics.h"

static volatile uint8_t rx_buffer[HOST_RX_DMA_CAPACITY];
/* Only the equal-priority channel-6 IRQ changes completed_bytes while active. */
static volatile uint32_t completed_bytes;
static volatile bool failed;
static uint32_t consumed, accounted;
static bool resync;
static volatile uart_isr_stats_t *stats;
#ifdef HOST_RX_DMA_TEST
extern void host_rx_dma_test_clear_flags(uint32_t flags);
#endif

static void clear_dma_flags(uint32_t flags)
{
    DMA1->IFCR = flags;
#ifdef HOST_RX_DMA_TEST
    host_rx_dma_test_clear_flags(flags);
#endif
}

/* CNDTR can reload before the TC interrupt executes. Include the pending wrap
 * exactly once, retrying if an IRQ or DMA wrap crosses this snapshot. A task
 * cannot execute halfway through its ISR. No IRQ masking in normal reads. */
static bool produced_snapshot(uint32_t *total)
{
    for (unsigned attempt = 0U; attempt < 8U; ++attempt) {
        const uint32_t base = completed_bytes;
        const uint32_t pending = DMA1->ISR & DMA_ISR_TCIF6;
        const uint32_t remaining = DMA1_Channel6->CNDTR;
        __DMB();
        if (base != completed_bytes || pending != (DMA1->ISR & DMA_ISR_TCIF6) ||
            remaining == 0U || remaining > HOST_RX_DMA_CAPACITY) continue;
        *total = base + (pending ? HOST_RX_DMA_CAPACITY : 0U) +
                 HOST_RX_DMA_CAPACITY - remaining;
        return true;
    }
    return false; /* Transient reload: retry on the next host service pass. */
}

static void account_progress(uint32_t produced)
{
    uart_diagnostics_rx_tail_bytes(1U, produced - accounted);
    accounted = produced;
    const uint32_t buffered = produced - consumed;
    uart_diagnostics_rx_buffered(1U, buffered > HOST_RX_DMA_CAPACITY ?
                                  HOST_RX_DMA_CAPACITY : buffered);
}

static void pause_rx(void)
{
    USART2->CR3 &= ~(USART_CR3_DMAR | USART_CR3_EIE);
    USART2->CR1 &= ~(USART_CR1_RE | USART_CR1_PEIE);
    DMA1_Channel6->CCR &= ~DMA_CCR_EN;
    failed = true;
}

/* IRQs remain for hardware errors only. DMA reads every normal DR byte.
 * Do not enable IDLEIE: polling CNDTR also sees short/partial bursts, and avoids
 * the F1 SR/DR IDLE-clear sequence competing with DMA for a new byte. */
void USART2_IRQHandler(void)
{
    const uint32_t started = DWT->CYCCNT;
    const uint32_t status = USART2->SR;
    if (status & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        pause_rx();
        const uint8_t byte = (uint8_t)USART2->DR; /* SR then DR clears errors. */
        ++stats->rx_interrupts;
        ++stats->rx_errors;
        uart_diagnostics_rx_error(stats, status, byte);
    }
    uart_isr_timing_finish(stats, started);
}

void DMA1_Channel6_IRQHandler(void)
{
    const uint32_t started = DWT->CYCCNT;
    const uint32_t flags = DMA1->ISR;
    if (flags & DMA_ISR_TCIF6) completed_bytes += HOST_RX_DMA_CAPACITY;
    clear_dma_flags(flags & (DMA_ISR_HTIF6 | DMA_ISR_TCIF6 | DMA_ISR_TEIF6));
    if (flags & DMA_ISR_TEIF6) {
        pause_rx();
        ++stats->rx_errors;
        host_transport_rx_dma_error();
    }
    uart_isr_timing_finish(stats, started);
}

/* Only init/error recovery reconfigures hardware under this brief peripheral-
 * specific guard. DMA/peripheral setup is protected, never a diagnostic copy,
 * a per-byte read or a transmit. Other IRQs remain enabled. */
static void restart_rx(void)
{
    NVIC_DisableIRQ(USART2_IRQn);
    NVIC_DisableIRQ(DMA1_Channel6_IRQn);
    USART2->CR3 &= ~USART_CR3_DMAR;
    USART2->CR1 &= ~USART_CR1_RE;
    DMA1_Channel6->CCR &= ~DMA_CCR_EN;
    volatile uint32_t discard = USART2->SR;
    discard = USART2->DR;
    (void)discard;
    completed_bytes = 0U;
    consumed = 0U;
    accounted = 0U;
    failed = false;
    clear_dma_flags(DMA_IFCR_CGIF6);
    DMA1_Channel6->CPAR = (uintptr_t)&USART2->DR;
    DMA1_Channel6->CMAR = (uintptr_t)rx_buffer;
    DMA1_Channel6->CNDTR = HOST_RX_DMA_CAPACITY;
    DMA1_Channel6->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL_1 |
                         DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_TEIE;
    NVIC_ClearPendingIRQ(USART2_IRQn);
    NVIC_ClearPendingIRQ(DMA1_Channel6_IRQn);
    __DMB();
    DMA1_Channel6->CCR |= DMA_CCR_EN;
    USART2->CR3 |= USART_CR3_DMAR | USART_CR3_EIE;
    USART2->CR1 |= USART_CR1_RE | USART_CR1_PEIE;
    NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    NVIC_EnableIRQ(USART2_IRQn);
}

void host_rx_dma_init(void)
{
    stats = uart_diagnostics_port(1U);
    uart_isr_timing_init(stats);
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    NVIC_SetPriority(USART2_IRQn, 5U);
    NVIC_SetPriority(DMA1_Channel6_IRQn, 5U);
    host_transport_rx_dma_mode(HOST_RX_DMA_CAPACITY);
    resync = false;
    restart_rx();
}

static void discard_overwritten(uint32_t produced, bool *discontinuity)
{
    consumed = produced;
    resync = true;
    *discontinuity = true;
    uart_diagnostics_rx_overflow(1U);
}

bool host_rx_dma_try_read(uint8_t *byte, bool *discontinuity)
{
    if (byte == NULL || discontinuity == NULL) return false;
    *discontinuity = false;
    uint32_t produced;
    if (failed) {
        if (produced_snapshot(&produced)) account_progress(produced);
        restart_rx();
        resync = true;
        *discontinuity = true;
        return false;
    }
    /* Bound recovery work too; discarded bytes never reach the frame parser. */
    for (unsigned skipped = 0U; skipped < 128U; ++skipped) {
        if (!produced_snapshot(&produced)) return false;
        account_progress(produced);
        if (produced - consumed > HOST_RX_DMA_CAPACITY) {
            discard_overwritten(produced, discontinuity);
            return false;
        }
        if (produced == consumed) return false;
        const uint32_t tail = consumed;
        const uint8_t candidate = rx_buffer[tail % HOST_RX_DMA_CAPACITY];
        __DMB();
        uint32_t after;
        if (!produced_snapshot(&after)) return false;
        if (failed) { *discontinuity = true; return false; }
        /* DMA can overwrite while CPU is preempted between snapshot and copy. */
        if (after - tail > HOST_RX_DMA_CAPACITY) {
            account_progress(after);
            discard_overwritten(after, discontinuity);
            return false;
        }
        consumed = tail + 1U;
        if (resync) {
            if (candidate == 0U) resync = false;
            continue;
        }
        *byte = candidate;
        return true;
    }
    return false;
}
