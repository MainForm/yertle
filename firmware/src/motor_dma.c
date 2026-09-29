/* F103 register port of Mowibox sts3215_hal.c's DMA transaction pattern:
 * RX armed before TX, persistent buffers, completion/error callbacks.
 * Original G4 HAL source stays unmodified in third_party. Unlike its IRQ
 * parser, parsing stays in task; multiple IDLE events do not end SyncRead.
 * No RS485 DE: FE-URT handles the TTL bus. */
#include "motor_dma.h"
#include <string.h>
#include "stm32f1xx.h"
#include "board_clock.h"
#include "uart_diagnostics.h"
#include "sts3215_protocol.h"
#define RX_CAPACITY 256U
#define UART_BAUD 1000000U
typedef enum { DMA_IDLE, DMA_TX_BUSY, DMA_RX_BUSY, DMA_ERROR } dma_state_t;
static uint8_t tx_buffer[STS3215_TX_BUF_SIZE], rx_buffer[RX_CAPACITY];
static volatile dma_state_t state;
static volatile bool active, tx_finished, failed, tx_dma_failed;
static volatile uint32_t rx_accounted;
static uint32_t rx_consumed, tx_length;
static bool tx_accounted;
static volatile uart_isr_stats_t *s;

static uint32_t received(void) {
    const uint32_t remaining = DMA1_Channel5->CNDTR;
    return remaining <= RX_CAPACITY ? RX_CAPACITY-remaining : 0U;
}
static void account_rx_irq(void) {
    if (!active) return;
    const uint32_t count = received();
    if (count >= rx_accounted) { s->rx_bytes += count-rx_accounted; rx_accounted = count; }
    if (count > s->rx_peak) s->rx_peak = count;
}
void USART1_IRQHandler(void) {
    const uint32_t begin = DWT->CYCCNT, sr = USART1->SR;
    const uint32_t errors = sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE);
    if (sr & (USART_SR_IDLE | USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) {
        const uint8_t byte = (uint8_t)USART1->DR; /* SR then DR clears IDLE/errors. */
        ++s->rx_interrupts; /* UART events, NOT bytes, in DMA mode. */
        account_rx_irq();
        if (sr & USART_SR_IDLE) motor_transport_record(TRANSPORT_RX_IDLE);
        if (errors) {
            ++s->rx_errors; uart_diagnostics_rx_error(s, sr, byte);
            if (active) { failed = true; state = DMA_ERROR; }
        }
    }
    uart_isr_timing_finish(s, begin);
}
void DMA1_Channel4_IRQHandler(void) {
    const uint32_t begin = DWT->CYCCNT, flags = DMA1->ISR;
    DMA1->IFCR = DMA_IFCR_CGIF4;
    if (flags & DMA_ISR_TEIF4) {
        tx_dma_failed = true; failed = true; state = DMA_ERROR; motor_transport_record(TRANSPORT_TX_DMA_ERROR);
    }
    if (flags & DMA_ISR_TCIF4) {
        DMA1_Channel4->CCR &= ~DMA_CCR_EN; tx_finished = true;
        if (!failed) state = DMA_RX_BUSY;
    }
    uart_isr_timing_finish(s, begin);
}
void DMA1_Channel5_IRQHandler(void) {
    const uint32_t begin = DWT->CYCCNT, flags = DMA1->ISR;
    DMA1->IFCR = DMA_IFCR_CGIF5;
    account_rx_irq();
    if (flags & DMA_ISR_TEIF5) {
        failed = true; state = DMA_ERROR; motor_transport_record(TRANSPORT_RX_DMA_ERROR);
    }
    if (flags & DMA_ISR_TCIF5) {
        /* Normal DMA exhausted: never silently overwrite unread bytes. */
        failed = true; state = DMA_ERROR; ++s->rx_overflows;
        motor_transport_record(TRANSPORT_RX_FULL);
    }
    uart_isr_timing_finish(s, begin);
}
static void account_tx_task(void) {
    if (active && !tx_accounted) {
        const uint32_t remaining = DMA1_Channel4->CNDTR;
        if (remaining <= tx_length) uart_diagnostics_tx_bytes(0U, tx_length-remaining);
        tx_accounted = true;
    }
}
void motor_dma_stop(void) {
    /* Peripheral-specific short boundary guard, NOT a diagnostic snapshot mask. */
    NVIC_DisableIRQ(USART1_IRQn); NVIC_DisableIRQ(DMA1_Channel4_IRQn); NVIC_DisableIRQ(DMA1_Channel5_IRQn);
    USART1->CR3 &= ~(USART_CR3_DMAR | USART_CR3_DMAT);
    DMA1_Channel4->CCR &= ~DMA_CCR_EN; DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    __DMB(); account_tx_task();
    if (active) {
        const uint32_t count = received();
        /* Keep ISR-owned RX counter single-writer; residue has task storage. */
        if (count >= rx_accounted) uart_diagnostics_rx_tail_bytes(0U, count-rx_accounted);
    }
    active = false;
    DMA1->IFCR = DMA_IFCR_CGIF4 | DMA_IFCR_CGIF5;
    volatile uint32_t discard = USART1->SR; discard = USART1->DR; (void)discard;
    NVIC_ClearPendingIRQ(USART1_IRQn); NVIC_ClearPendingIRQ(DMA1_Channel4_IRQn); NVIC_ClearPendingIRQ(DMA1_Channel5_IRQn);
    state = DMA_IDLE; __DMB();
    NVIC_EnableIRQ(DMA1_Channel4_IRQn); NVIC_EnableIRQ(DMA1_Channel5_IRQn); NVIC_EnableIRQ(USART1_IRQn);
}
void motor_dma_init(void) {
    s = uart_diagnostics_port(0U); uart_isr_timing_init(s);
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;
    GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9)) |
        GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1;
    GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10)) | GPIO_CRH_CNF10_0;
    USART1->BRR = (board_apb2_clock_hz()+UART_BAUD/2U)/UART_BAUD;
    USART1->CR3 = USART_CR3_EIE;
    /* RXNEIE removed: DMA reads normal bytes; UART IRQ handles IDLE/errors. */
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_IDLEIE | USART_CR1_PEIE;
    NVIC_SetPriority(USART1_IRQn,5U); NVIC_SetPriority(DMA1_Channel4_IRQn,5U); NVIC_SetPriority(DMA1_Channel5_IRQn,5U);
    /* Equal IRQ priority serializes the three ISR diagnostic writers. */
    motor_transport_dma_mode(RX_CAPACITY); motor_dma_stop();
}
bool motor_dma_start(const uint8_t *data, size_t length) {
    if (!data || !length || length>sizeof tx_buffer || state!=DMA_IDLE) return false;
    memcpy(tx_buffer,data,length);
    rx_accounted=0; rx_consumed=0; tx_length=(uint32_t)length;
    failed=false; tx_dma_failed=false; tx_finished=false; tx_accounted=false;
    DMA1->IFCR = DMA_IFCR_CGIF4 | DMA_IFCR_CGIF5;
    DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_PL | DMA_CCR_TCIE | DMA_CCR_TEIE;
    DMA1_Channel5->CPAR = (uintptr_t)&USART1->DR; DMA1_Channel5->CMAR = (uintptr_t)rx_buffer;
    DMA1_Channel5->CNDTR = RX_CAPACITY;
    DMA1_Channel4->CCR = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PL_1 | DMA_CCR_TCIE | DMA_CCR_TEIE;
    DMA1_Channel4->CPAR = (uintptr_t)&USART1->DR; DMA1_Channel4->CMAR = (uintptr_t)tx_buffer;
    DMA1_Channel4->CNDTR = tx_length;
    active=true; state=DMA_TX_BUSY; __DMB();
    DMA1_Channel5->CCR |= DMA_CCR_EN; USART1->CR3 |= USART_CR3_DMAR; /* RX before TX. */
    motor_transport_record(TRANSPORT_TX_STARTED);
    DMA1_Channel4->CCR |= DMA_CCR_EN; USART1->CR3 |= USART_CR3_DMAT;
    return true;
}
bool motor_dma_tx_complete(void) {
    if (failed || !tx_finished || !(USART1->SR & USART_SR_TC)) return false;
    if (!tx_accounted) { account_tx_task(); motor_transport_record(TRANSPORT_TX_COMPLETE); }
    return true;
}
bool motor_dma_failed(void) { return failed; }
bool motor_dma_tx_failed(void) { return tx_dma_failed; }
bool motor_dma_read_byte(uint8_t *byte) {
    if (!active || failed || !byte || rx_consumed>=received()) return false;
    __DMB(); *byte=rx_buffer[rx_consumed++]; return true;
}
