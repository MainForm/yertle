#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_dma.h"
#include "motor_uart.h"
#include "motor_device.h"
#include "uart_diagnostics.h"
#include "sts3215_protocol.h"
DWT_Type test_dwt;
CoreDebug_Type test_debug;
USART_Type test_usart;
GPIO_Type test_gpio;
NVIC_Type test_nvic;
RCC_Type test_rcc;
DMA_Type test_dma;
DMA_Channel_Type test_dma4,test_dma5;
uint32_t SystemCoreClock=64000000U;
static uint32_t tick=1000U;
static bool complete_tx, inject_sync;
static int omit_id;
static unsigned tx_simulations;
void USART1_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);
uint32_t board_apb2_clock_hz(void) { return 64000000U; }
uint32_t xTaskGetTickCount(void) { return tick; }
void host_uart_get_stats(uart_stats_t *s) { uart_diagnostics_get_stats(1,s); }
static void inject_bytes(const uint8_t *data,size_t size) {
    assert(size<=256);
    memcpy((void *)DMA1_Channel5->CMAR,data,size);
    DMA1_Channel5->CNDTR=256U-(uint32_t)size;
    USART1->SR=USART_SR_TC | USART_SR_IDLE;
    USART1_IRQHandler();
    USART1->SR=USART_SR_TC;
}
static size_t reply(uint8_t *p,uint8_t id,uint8_t len) {
    p[0]=0xff; p[1]=0xff; p[2]=id; p[3]=len+2; p[4]=0;
    memset(p+5,0,len); p[5]=id;
    if (len==8) { p[11]=64; p[12]=30; }
    unsigned sum=0;
    for (unsigned i=2;i<(unsigned)len+5;++i) sum+=p[i];
    p[5+len]=(uint8_t)~sum;
    return len+6U;
}
void vTaskDelay(uint32_t ticks) {
    tick+=ticks;
    if (complete_tx && (DMA1_Channel4->CCR & DMA_CCR_EN) && DMA1_Channel4->CNDTR) {
        /* Verify RX ready when TX starts, before generating a reply. */
        assert(DMA1_Channel5->CCR & DMA_CCR_EN);
        assert(USART1->CR3 & USART_CR3_DMAR);
        const uint8_t *q=(const uint8_t *)DMA1_Channel4->CMAR;
        DMA1_Channel4->CNDTR=0;
        DMA1->ISR=DMA_ISR_TCIF4; DMA1_Channel4_IRQHandler(); DMA1->ISR=0;
        USART1->SR=USART_SR_TC; ++tx_simulations;
        if (inject_sync && q[4]==0x82) {
            uint8_t packets[128]; size_t size=0;
            for (uint8_t id=1;id<=6;++id) if (id!=omit_id) size+=reply(packets+size,id,q[6]);
            inject_bytes(packets,size);
        }
    }
}
static uint32_t transport_words[16], diagnostic_words[16];
static uint32_t get32(const uint8_t *p) {
    return p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}
static void capture(void *ctx,uint8_t type,const uint8_t *p,uint16_t length) {
    (void)ctx;
    if (type==0x8a || type==0x87) {
        assert(length==64);
        uint32_t *words=type==0x8a ? transport_words : diagnostic_words;
        for (unsigned i=0;i<16;++i) words[i]=get32(p+i*4);
    }
}
int main(void) {
    motor_dma_init();
    assert(USART1->BRR==0x40);
    assert(!(USART1->CR1 & USART_CR1_RXNEIE));
    assert(USART1->CR1 & USART_CR1_IDLEIE);
    uint8_t request[14]={0xff,0xff,0xfe,10,0x82,0x38,8,1,2,3,4,5,6,0x20};
    assert(motor_dma_start(request,sizeof request));
    assert(!motor_dma_start(request,sizeof request)); /* no buffer reuse while busy */
    assert(!motor_dma_tx_complete());
    request[0]=0; assert(((uint8_t *)DMA1_Channel4->CMAR)[0]==0xff); /* owned TX copy */
    DMA1_Channel4->CNDTR=0; DMA1->ISR=DMA_ISR_TCIF4;
    DMA1_Channel4_IRQHandler(); DMA1->ISR=0;
    USART1->SR=0; assert(!motor_dma_tx_complete()); /* DMA TC is not UART wire TC */
    USART1->SR=USART_SR_TC; assert(motor_dma_tx_complete());
    uint8_t packets[128]; size_t size=0;
    for (uint8_t id=1;id<=6;++id) size+=reply(packets+size,id,8);
    inject_bytes(packets,14); /* first IDLE must not terminate the whole response */
    inject_bytes(packets,28);
    inject_bytes(packets,size);
    for (uint8_t id=1;id<=6;++id) {
        uint8_t frame[16]; size_t length=0; STS3215_Reply_t parsed;
        assert(motor_uart_read_frame(frame,sizeof frame,&length,5));
        assert(STS3215_ParseReply(frame,(uint16_t)length,&parsed)==STS3215_OK);
        assert(parsed.id==id && parsed.data_len==8);
    }
    uart_stats_t stats;
    motor_uart_get_stats(&stats); assert(stats.rx_bytes==84 && stats.tx_bytes==14);
    motor_dma_stop(); motor_uart_get_stats(&stats); assert(stats.rx_bytes==84);
    assert(motor_dma_start(request,sizeof request));
    /* Residue without IDLE still counted exactly once at stop. */
    DMA1_Channel5->CNDTR=253; motor_dma_stop();
    motor_uart_get_stats(&stats); assert(stats.rx_bytes==87);
    motor_dma_stop(); motor_uart_get_stats(&stats); assert(stats.rx_bytes==87);
    assert(motor_dma_start(request,sizeof request));
    DMA1_Channel5->CNDTR=0; DMA1->ISR=DMA_ISR_TCIF5;
    DMA1_Channel5_IRQHandler(); DMA1->ISR=0;
    assert(motor_dma_failed()); motor_uart_get_stats(&stats); assert(stats.rx_overflows==1);
    motor_dma_stop();
    assert(motor_dma_start(request,sizeof request));
    DMA1->ISR=DMA_ISR_TEIF4; DMA1_Channel4_IRQHandler(); DMA1->ISR=0;
    assert(motor_dma_failed() && motor_dma_tx_failed()); motor_dma_stop();
    assert(motor_dma_start(request,sizeof request));
    DMA1->ISR=DMA_ISR_TEIF5; DMA1_Channel5_IRQHandler(); DMA1->ISR=0;
    assert(motor_dma_failed()); motor_dma_stop();
    assert(motor_dma_start(request,sizeof request));
    USART1->SR=USART_SR_FE | USART_SR_NE;
    USART1_IRQHandler(); USART1->SR=0;
    assert(motor_dma_failed()); motor_uart_get_stats(&stats);
    assert(stats.fe_errors==1 && stats.ne_errors==1 && stats.rx_errors==1);
    motor_dma_stop();
    assert(!motor_uart_write(request,sizeof request,5)); /* TX timeout abort */
    complete_tx=true;
    assert(motor_uart_write(request,sizeof request,5));
    uint8_t frame[16]; size_t length=0;
    assert(!motor_uart_read_frame(frame,sizeof frame,&length,5)); /* no response */
    motor_dma_stop();
    assert(motor_uart_write(request,sizeof request,5));
    const uint8_t partial[]={0xff,0xff}; inject_bytes(partial,sizeof partial);
    assert(!motor_uart_read_frame(frame,sizeof frame,&length,5)); /* partial body */
    motor_dma_stop();
    uart_diagnostics_send(NULL,capture);
    assert(diagnostic_words[4+DIAG_NO_RESPONSE]==1 && diagnostic_words[4+DIAG_BODY_TIMEOUT]==1);
    assert(diagnostic_words[4+DIAG_TX_TIMEOUT]==1);
    assert(motor_device_init());
    inject_sync=true;
    motor_feedback_set_t feedback;
    assert(motor_device_read_feedback(&feedback)==MOTOR_DEVICE_OK);
    assert(feedback.valid_mask==0x3f && tx_simulations==4);
    for (unsigned i=0;i<6;++i) assert(feedback.motor[i].position==(int)i+1 && feedback.motor[i].current==(int)i+1);
    omit_id=6;
    assert(motor_device_read_feedback(&feedback)==MOTOR_DEVICE_PARTIAL_FEEDBACK);
    assert(feedback.valid_mask==0x1f);
    uart_diagnostics_send(NULL,capture);
    assert(transport_words[1]==1 && transport_words[12]==256);
    assert(transport_words[6]==1 && transport_words[7]==1 && transport_words[5]==1);
    assert(transport_words[8]==4 && transport_words[9]==2);
    assert(transport_words[10]==2 && transport_words[11]==1);
    puts("DMA driver tests passed: six replies/multiple IDLE, missing ID, TX wire completion, abort, errors, bounds, statistics (mock registers, not board)");
}
