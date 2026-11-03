/* Serial motor DMA transport. Each bus owns its hardware and transaction
 * state. The motor_device mutex currently serializes whole transactions. */
#include "motor_dma.h"
#include <string.h>
#include "stm32f1xx.h"
#include "board_clock.h"
#include "uart_diagnostics.h"
#include "sts3215_protocol.h"

#define RX_CAPACITY 256U
#define UART_BAUD 1000000U
typedef struct {
    USART_TypeDef *uart;
    DMA_Channel_TypeDef *tx, *rx;
    IRQn_Type uart_irq, tx_irq, rx_irq;
    uint32_t tx_clear, rx_clear, tx_tc, tx_error, rx_tc, rx_error;
    uint8_t tx_buffer[STS3215_TX_BUF_SIZE], rx_buffer[RX_CAPACITY];
    volatile bool active, tx_finished, failed, tx_dma_failed;
    volatile uint32_t rx_accounted;
    uint32_t rx_consumed, tx_length;
    bool tx_accounted, initialized;
    volatile uart_isr_stats_t *stats;
    unsigned diagnostic_port;
} motor_dma_port_t;

static motor_dma_port_t ports[2] = {
    { .uart=USART1,.tx=DMA1_Channel4,.rx=DMA1_Channel5,
      .uart_irq=USART1_IRQn,.tx_irq=DMA1_Channel4_IRQn,.rx_irq=DMA1_Channel5_IRQn,
      .tx_clear=DMA_IFCR_CGIF4,.rx_clear=DMA_IFCR_CGIF5,.tx_tc=DMA_ISR_TCIF4,
      .tx_error=DMA_ISR_TEIF4,.rx_tc=DMA_ISR_TCIF5,.rx_error=DMA_ISR_TEIF5 },
    { .uart=USART3,.tx=DMA1_Channel2,.rx=DMA1_Channel3,
      .uart_irq=USART3_IRQn,.tx_irq=DMA1_Channel2_IRQn,.rx_irq=DMA1_Channel3_IRQn,
      .tx_clear=DMA_IFCR_CGIF2,.rx_clear=DMA_IFCR_CGIF3,.tx_tc=DMA_ISR_TCIF2,
      .tx_error=DMA_ISR_TEIF2,.rx_tc=DMA_ISR_TCIF3,.rx_error=DMA_ISR_TEIF3 }
};
static motor_dma_port_t *port(motor_bus_id_t bus) {
    return bus==MOTOR_BUS_USART1 ? &ports[0] : bus==MOTOR_BUS_USART3 ? &ports[1] : NULL;
}
static uint32_t received(const motor_dma_port_t *p) {
    uint32_t left=p->rx->CNDTR; return left<=RX_CAPACITY ? RX_CAPACITY-left : 0U;
}
static void account_rx_irq(motor_dma_port_t *p) {
    if (!p->active) return;
    const uint32_t count=received(p);
    if(count>=p->rx_accounted){p->stats->rx_bytes+=count-p->rx_accounted;p->rx_accounted=count;}
    if(count>p->stats->rx_peak)p->stats->rx_peak=count;
}
static void uart_irq(motor_dma_port_t *p) {
    const uint32_t begin=DWT->CYCCNT,sr=p->uart->SR;
    const uint32_t errors=sr&(USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE);
    if(sr&(USART_SR_IDLE|USART_SR_ORE|USART_SR_FE|USART_SR_NE|USART_SR_PE)) {
        const uint8_t byte=(uint8_t)p->uart->DR;
        ++p->stats->rx_interrupts;account_rx_irq(p);
        if(sr&USART_SR_IDLE)motor_transport_record_bus(p->diagnostic_port,TRANSPORT_RX_IDLE);
        if(errors){++p->stats->rx_errors;uart_diagnostics_rx_error(p->stats,sr,byte);if(p->active)p->failed=true;}
    }
    uart_isr_timing_finish(p->stats,begin);
}
static void tx_irq(motor_dma_port_t *p) {
    const uint32_t begin=DWT->CYCCNT, flags=DMA1->ISR;
    DMA1->IFCR=p->tx_clear;
    if(flags&p->tx_error){p->tx_dma_failed=true;p->failed=true;motor_transport_record_bus(p->diagnostic_port,TRANSPORT_TX_DMA_ERROR);}
    if(flags&p->tx_tc){p->tx->CCR&=~DMA_CCR_EN;p->tx_finished=true;}
    uart_isr_timing_finish(p->stats,begin);
}
static void rx_irq(motor_dma_port_t *p) {
    const uint32_t begin=DWT->CYCCNT,flags=DMA1->ISR;
    DMA1->IFCR=p->rx_clear;account_rx_irq(p);
    if(flags&p->rx_error){p->failed=true;motor_transport_record_bus(p->diagnostic_port,TRANSPORT_RX_DMA_ERROR);}
    if(flags&p->rx_tc){p->failed=true;++p->stats->rx_overflows;motor_transport_record_bus(p->diagnostic_port,TRANSPORT_RX_FULL);}
    uart_isr_timing_finish(p->stats,begin);
}
void USART1_IRQHandler(void){uart_irq(&ports[0]);}
void DMA1_Channel4_IRQHandler(void){tx_irq(&ports[0]);}
void DMA1_Channel5_IRQHandler(void){rx_irq(&ports[0]);}
void USART3_IRQHandler(void){uart_irq(&ports[1]);}
void DMA1_Channel2_IRQHandler(void){tx_irq(&ports[1]);}
void DMA1_Channel3_IRQHandler(void){rx_irq(&ports[1]);}

static void account_tx(motor_dma_port_t *p) {
    if(p->active&&!p->tx_accounted){uint32_t left=p->tx->CNDTR;
        if(left<=p->tx_length)uart_diagnostics_tx_bytes(p->diagnostic_port,p->tx_length-left);
        p->tx_accounted=true;}
}
void motor_dma_stop(motor_bus_id_t bus) {
    motor_dma_port_t *p=port(bus);if(!p||!p->initialized)return;
    NVIC_DisableIRQ(p->uart_irq);NVIC_DisableIRQ(p->tx_irq);NVIC_DisableIRQ(p->rx_irq);
    p->uart->CR3&=~(USART_CR3_DMAR|USART_CR3_DMAT);
    p->tx->CCR&=~DMA_CCR_EN;p->rx->CCR&=~DMA_CCR_EN;
    __DMB();account_tx(p);
    if(p->active){uint32_t count=received(p);if(count>=p->rx_accounted)uart_diagnostics_rx_tail_bytes(p->diagnostic_port,count-p->rx_accounted);}
    p->active=false;DMA1->IFCR=p->tx_clear|p->rx_clear;
    volatile uint32_t discard=p->uart->SR;discard=p->uart->DR;(void)discard;
    NVIC_ClearPendingIRQ(p->uart_irq);NVIC_ClearPendingIRQ(p->tx_irq);NVIC_ClearPendingIRQ(p->rx_irq);
    __DMB();NVIC_EnableIRQ(p->tx_irq);NVIC_EnableIRQ(p->rx_irq);NVIC_EnableIRQ(p->uart_irq);
}
void motor_dma_init(motor_bus_id_t bus) {
    motor_dma_port_t *p=port(bus);if(!p)return;
    p->diagnostic_port=bus==MOTOR_BUS_USART1?0U:2U;
    p->stats=uart_diagnostics_port(p->diagnostic_port);uart_isr_timing_init(p->stats);
    RCC->AHBENR|=RCC_AHBENR_DMA1EN;
    if(bus==MOTOR_BUS_USART1){
        RCC->APB2ENR|=RCC_APB2ENR_IOPAEN|RCC_APB2ENR_USART1EN;
        GPIOA->CRH=(GPIOA->CRH&~(GPIO_CRH_MODE9|GPIO_CRH_CNF9))|GPIO_CRH_MODE9_0|GPIO_CRH_MODE9_1|GPIO_CRH_CNF9_1;
        GPIOA->CRH=(GPIOA->CRH&~(GPIO_CRH_MODE10|GPIO_CRH_CNF10))|GPIO_CRH_CNF10_0;
        p->uart->BRR=(board_apb2_clock_hz()+UART_BAUD/2U)/UART_BAUD;
    }else{
        RCC->APB2ENR|=RCC_APB2ENR_IOPBEN;RCC->APB1ENR|=RCC_APB1ENR_USART3EN;
        /* Default mapping PB10 TX, PB11 RX; UART-side FE-URT logic 3.3 V. */
        GPIOB->CRH=(GPIOB->CRH&~(GPIO_CRH_MODE10|GPIO_CRH_CNF10|GPIO_CRH_MODE11|GPIO_CRH_CNF11))|
            GPIO_CRH_MODE10_0|GPIO_CRH_MODE10_1|GPIO_CRH_CNF10_1|GPIO_CRH_CNF11_0;
        p->uart->BRR=(board_apb1_clock_hz()+UART_BAUD/2U)/UART_BAUD;
    }
    p->uart->CR3=USART_CR3_EIE;
    p->uart->CR1=USART_CR1_TE|USART_CR1_RE|USART_CR1_UE|USART_CR1_IDLEIE|USART_CR1_PEIE;
    NVIC_SetPriority(p->uart_irq,5U);NVIC_SetPriority(p->tx_irq,5U);NVIC_SetPriority(p->rx_irq,5U);
    p->initialized=true;motor_transport_dma_mode(RX_CAPACITY);motor_dma_stop(bus);
}
bool motor_dma_start(motor_bus_id_t bus,const uint8_t *data,size_t length) {
    motor_dma_port_t *p=port(bus);
    if(!p||!p->initialized||!data||!length||length>sizeof p->tx_buffer||p->active)return false;
    memcpy(p->tx_buffer,data,length);p->rx_accounted=0;p->rx_consumed=0;p->tx_length=(uint32_t)length;
    p->failed=false;p->tx_dma_failed=false;p->tx_finished=false;p->tx_accounted=false;
    DMA1->IFCR=p->tx_clear|p->rx_clear;
    p->rx->CCR=DMA_CCR_MINC|DMA_CCR_PL|DMA_CCR_TCIE|DMA_CCR_TEIE;
    p->rx->CPAR=(uintptr_t)&p->uart->DR;p->rx->CMAR=(uintptr_t)p->rx_buffer;p->rx->CNDTR=RX_CAPACITY;
    p->tx->CCR=DMA_CCR_DIR|DMA_CCR_MINC|DMA_CCR_PL_1|DMA_CCR_TCIE|DMA_CCR_TEIE;
    p->tx->CPAR=(uintptr_t)&p->uart->DR;p->tx->CMAR=(uintptr_t)p->tx_buffer;p->tx->CNDTR=p->tx_length;
    p->active=true;__DMB();p->rx->CCR|=DMA_CCR_EN;p->uart->CR3|=USART_CR3_DMAR;
    motor_transport_record_bus(p->diagnostic_port,TRANSPORT_TX_STARTED);p->tx->CCR|=DMA_CCR_EN;p->uart->CR3|=USART_CR3_DMAT;
    return true;
}
bool motor_dma_tx_complete(motor_bus_id_t bus) {
    motor_dma_port_t *p=port(bus);if(!p||!p->initialized||p->failed||!p->tx_finished||!(p->uart->SR&USART_SR_TC))return false;
    if(!p->tx_accounted){account_tx(p);motor_transport_record_bus(p->diagnostic_port,TRANSPORT_TX_COMPLETE);}return true;
}
bool motor_dma_failed(motor_bus_id_t bus){motor_dma_port_t *p=port(bus);return !p||!p->initialized||p->failed;}
bool motor_dma_tx_failed(motor_bus_id_t bus){motor_dma_port_t *p=port(bus);return !p||!p->initialized||p->tx_dma_failed;}
bool motor_dma_read_byte(motor_bus_id_t bus,uint8_t *byte){
    motor_dma_port_t *p=port(bus);if(!p||!p->initialized||!p->active||p->failed||!byte||p->rx_consumed>=received(p))return false;
    __DMB();*byte=p->rx_buffer[p->rx_consumed++];return true;
}
