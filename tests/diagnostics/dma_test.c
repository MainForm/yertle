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
static uint8_t motion_reply_error, current_reply_error;
static int omit_id;
static unsigned tx_simulations;
static bool inject_offset;
static uint16_t offset_raw;
static uint8_t offset_error, offset_reply_id, offset_data_length = 2U;
static bool offset_bad_checksum;
void USART1_IRQHandler(void);
void DMA1_Channel4_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);
uint32_t board_apb2_clock_hz(void) { return 64000000U; }
uint32_t xTaskGetTickCount(void) { return tick; }
static void inject_bytes(const uint8_t *data,size_t size) {
    assert(size<=256);
    memcpy((void *)DMA1_Channel5->CMAR,data,size);
    DMA1_Channel5->CNDTR=256U-(uint32_t)size;
    USART1->SR=USART_SR_TC | USART_SR_IDLE;
    USART1_IRQHandler();
    USART1->SR=USART_SR_TC;
}
static size_t reply(uint8_t *p,uint8_t id,uint8_t len) {
    p[0]=0xff; p[1]=0xff; p[2]=id; p[3]=len+2;
    p[4]=len==8 ? motion_reply_error : current_reply_error;
    memset(p+5,0,len); p[5]=id;
    if (len==8) {
        p[11]=64; p[12]=30;
        if (id==7) { p[7]=0x64; p[8]=0x80; } /* -100 speed, not -32668 */
        if (id==8) { p[9]=0x33; p[10]=0x04; } /* -51 load, not 1075 */
        if (id==12) { p[5]=0x0c; p[6]=0x80; } /* negative position */
    }
    if (len==2 && id==9) { p[5]=0x05; p[6]=0x80; } /* -5 current */
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
            uint8_t packets[256]; size_t size=0;
            for (uint8_t id=1;id<=YERTLE_MOTOR_COUNT;++id) if (id!=omit_id) size+=reply(packets+size,id,q[6]);
            inject_bytes(packets,size);
        }
        if (inject_offset && q[4]==0x02) {
            assert(q[5]==0x1f && q[6]==2U && q[3]==4U); /* EEPROM READ only */
            uint8_t packet[16];
            const size_t size=reply(packet,offset_reply_id ? offset_reply_id : q[2],offset_data_length);
            packet[4]=offset_error;
            packet[5]=(uint8_t)offset_raw;
            if (offset_data_length==2U) packet[6]=(uint8_t)(offset_raw>>8U);
            unsigned sum=0U;
            for (unsigned i=2U;i<size-1U;++i) sum+=packet[i];
            packet[size-1U]=(uint8_t)~sum;
            if (offset_bad_checksum) packet[size-1U]^=1U;
            inject_bytes(packet,size);
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
    motor_position_offset_t not_initialized;
    assert(motor_device_read_position_offset(1U,&not_initialized)==MOTOR_DEVICE_NOT_INITIALIZED);
    motor_dma_init();
    assert(USART1->BRR==0x40);
    assert(!(USART1->CR1 & USART_CR1_RXNEIE));
    assert(USART1->CR1 & USART_CR1_IDLEIE);
    const uint8_t ids[YERTLE_MOTOR_COUNT]={1,2,3,4,5,6,7,8,9,10,11,12};
    uint8_t request[YERTLE_MOTOR_COUNT+8U];
    assert(STS3215_BuildSyncRead(request,sizeof request,0x38,8,ids,YERTLE_MOTOR_COUNT)==sizeof request);
    assert(motor_dma_start(request,sizeof request));
    assert(!motor_dma_start(request,sizeof request)); /* no buffer reuse while busy */
    assert(!motor_dma_tx_complete());
    request[0]=0; assert(((uint8_t *)DMA1_Channel4->CMAR)[0]==0xff); /* owned TX copy */
    DMA1_Channel4->CNDTR=0; DMA1->ISR=DMA_ISR_TCIF4;
    DMA1_Channel4_IRQHandler(); DMA1->ISR=0;
    USART1->SR=0; assert(!motor_dma_tx_complete()); /* DMA TC is not UART wire TC */
    USART1->SR=USART_SR_TC; assert(motor_dma_tx_complete());
    uint8_t packets[256]; size_t size=0;
    for (uint8_t id=1;id<=YERTLE_MOTOR_COUNT;++id) size+=reply(packets+size,id,8);
    inject_bytes(packets,14); /* first IDLE must not terminate the whole response */
    inject_bytes(packets,28);
    inject_bytes(packets,size);
    for (uint8_t id=1;id<=YERTLE_MOTOR_COUNT;++id) {
        uint8_t frame[16]; size_t length=0; STS3215_Reply_t parsed;
        assert(motor_uart_read_frame(frame,sizeof frame,&length,5));
        assert(STS3215_ParseReply(frame,(uint16_t)length,&parsed)==STS3215_OK);
        assert(parsed.id==id && parsed.data_len==8);
    }
    uart_stats_t stats;
    uart_diagnostics_get_stats(0U,&stats); assert(stats.rx_bytes==168 && stats.tx_bytes==sizeof request);
    motor_dma_stop(); uart_diagnostics_get_stats(0U,&stats); assert(stats.rx_bytes==168);
    assert(motor_dma_start(request,sizeof request));
    /* Residue without IDLE still counted exactly once at stop. */
    DMA1_Channel5->CNDTR=253; motor_dma_stop();
    uart_diagnostics_get_stats(0U,&stats); assert(stats.rx_bytes==171);
    motor_dma_stop(); uart_diagnostics_get_stats(0U,&stats); assert(stats.rx_bytes==171);
    assert(motor_dma_start(request,sizeof request));
    DMA1_Channel5->CNDTR=0; DMA1->ISR=DMA_ISR_TCIF5;
    DMA1_Channel5_IRQHandler(); DMA1->ISR=0;
    assert(motor_dma_failed()); uart_diagnostics_get_stats(0U,&stats); assert(stats.rx_overflows==1);
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
    assert(motor_dma_failed()); uart_diagnostics_get_stats(0U,&stats);
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
    assert(feedback.valid_mask==YERTLE_ALL_MOTORS_MASK && tx_simulations==4);
    for (unsigned i=0;i<YERTLE_MOTOR_COUNT;++i) {
        assert(feedback.motor[i].position==(i==11U ? -12 : (int)i+1));
        assert(feedback.motor[i].current==(i==8U ? -5 : (int)i+1));
    }
    assert(feedback.motor[6].speed==-100);
    assert(feedback.motor[7].load==-51);
    omit_id=12;
    assert(motor_device_read_feedback(&feedback)==MOTOR_DEVICE_PARTIAL_FEEDBACK);
    assert(feedback.valid_mask==0x7ff);
    motor_target_set_t targets={0};
    for (unsigned i=0;i<YERTLE_MOTOR_COUNT;++i) {
        targets.motor[i].position=(uint16_t)(2048U+i);
        targets.motor[i].speed=200U;
        targets.motor[i].acceleration=10U;
    }
    assert(motor_device_validate_targets(&targets));
    assert(motor_device_write_targets(&targets)==MOTOR_DEVICE_OK);
    targets.motor[11].position=4096U;
    assert(!motor_device_validate_targets(&targets));
    uart_diagnostics_send(NULL,capture);
    assert(transport_words[1]==1 && transport_words[12]==256);
    assert(transport_words[6]==1 && transport_words[7]==1 && transport_words[5]==1);
    assert(transport_words[8]==4 && transport_words[9]==2);
    assert(transport_words[10]==2 && transport_words[11]==1);
    motor_position_offset_t offset;
    const unsigned before_invalid=tx_simulations;
    assert(motor_device_read_position_offset(0U,&offset)==MOTOR_DEVICE_INVALID_ARGUMENT);
    assert(motor_device_read_position_offset(13U,&offset)==MOTOR_DEVICE_INVALID_ARGUMENT);
    assert(motor_device_read_position_offset(1U,NULL)==MOTOR_DEVICE_INVALID_ARGUMENT);
    assert(tx_simulations==before_invalid);
    inject_offset=true;
    const uint16_t raws[]={0U,123U,2047U,0x087bU,0x0fffU,0x0800U};
    const int16_t expected[]={0,123,2047,-123,-2047,0};
    for (unsigned i=0U;i<6U;++i) {
        offset_raw=raws[i];
        assert(motor_device_read_position_offset(12U,&offset)==MOTOR_DEVICE_OK);
        assert(offset.raw==raws[i] && offset.offset_steps==expected[i] && offset.servo_error==0U);
        assert(!(DMA1_Channel5->CCR & DMA_CCR_EN));
    }
    offset_raw=0x1000U;
    assert(motor_device_read_position_offset(1U,&offset)==MOTOR_DEVICE_PROTOCOL_ERROR);
    offset_raw=42U; offset_reply_id=2U;
    assert(motor_device_read_position_offset(1U,&offset)==MOTOR_DEVICE_PROTOCOL_ERROR);
    offset_reply_id=0U; offset_data_length=1U;
    assert(motor_device_read_position_offset(1U,&offset)==MOTOR_DEVICE_PROTOCOL_ERROR);
    offset_data_length=2U; offset_error=4U;
    assert(motor_device_read_position_offset(1U,&offset)==MOTOR_DEVICE_PROTOCOL_ERROR);
    assert(offset.servo_error==4U);
    offset_error=0U; offset_bad_checksum=true;
    assert(motor_device_read_position_offset(1U,&offset)==MOTOR_DEVICE_PROTOCOL_ERROR);
    offset_bad_checksum=false; inject_offset=false;
    assert(motor_device_read_position_offset(1U,&offset)==MOTOR_DEVICE_UART_ERROR);
    assert(!(DMA1_Channel5->CCR & DMA_CCR_EN));
    complete_tx=false;
    assert(motor_device_read_position_offset(1U,&offset)==MOTOR_DEVICE_UART_ERROR);
    assert(!(DMA1_Channel4->CCR & DMA_CCR_EN) && !(DMA1_Channel5->CCR & DMA_CCR_EN));
    complete_tx=true;
    inject_offset=true;
    assert(motor_device_read_position_offset(1U,&offset)==MOTOR_DEVICE_OK && offset.offset_steps==42);
    inject_offset=false; omit_id=0;
    const uint8_t motion_errors[]={4U,4U,0U,0U};
    const uint8_t current_errors[]={0U,8U,8U,0U};
    const uint8_t merged_errors[]={4U,12U,8U,0U};
    for (unsigned test=0U;test<4U;++test) {
        motion_reply_error=motion_errors[test]; current_reply_error=current_errors[test];
        assert(motor_device_read_feedback(&feedback)==MOTOR_DEVICE_OK);
        assert(feedback.valid_mask==YERTLE_ALL_MOTORS_MASK);
        for (unsigned id=0U;id<YERTLE_MOTOR_COUNT;++id)
            assert(feedback.motor[id].servo_error==merged_errors[test]);
    }
    puts("DMA driver/offset tests passed (mock registers, not board)");
}
