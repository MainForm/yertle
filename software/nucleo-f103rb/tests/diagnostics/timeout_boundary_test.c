#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_uart.h"
#include "motor_dma.h"
#include "motor_failure.h"
#include "host_uart.h"
#include "host_rx_dma.h"
#include "FreeRTOS.h"
#include "task.h"
DWT_Type test_dwt;CoreDebug_Type test_debug;
USART_Type test_usart,test_usart2;
GPIO_Type test_gpio;RCC_Type test_rcc;
uint32_t SystemCoreClock=64000000U;
enum scenario { TX_EDGE, RX_EDGE, HOST_TXE_EDGE, HOST_TC_EDGE, SILENT, RX_ERROR_EDGE };
static enum scenario scenario;
static unsigned tick,reads,timeouts,failures,consumed,tx_checks;
static bool done,stopped,rx_ready,rx_failed;
static const uint8_t reply[]={0xff,0xff,1,2,0,0xfc};
uint32_t xTaskGetTickCount(void){
    if(++reads==2U){
        tick=21U;test_dwt.CYCCNT=tick*64000U;
        if(scenario==TX_EDGE)done=true;
        if(scenario==RX_EDGE||scenario==RX_ERROR_EDGE)rx_ready=true;
        if(scenario==RX_ERROR_EDGE)rx_failed=true;
        if(scenario==HOST_TXE_EDGE||scenario==HOST_TC_EDGE)USART2->SR=USART_SR_TXE|USART_SR_TC;
    }
    return tick;
}
void vTaskDelay(uint32_t n){tick+=n;}
uint32_t board_apb1_clock_hz(void){return 32000000U;}
void host_rx_dma_init(void){}
bool host_rx_dma_try_read(uint8_t *b,bool *d){(void)b;(void)d;return false;}
void motor_dma_init(motor_bus_id_t b){(void)b;}
bool motor_dma_start(motor_bus_id_t b,const uint8_t *p,size_t n){(void)b;(void)p;(void)n;return true;}
bool motor_dma_tx_complete(motor_bus_id_t b){(void)b;++tx_checks;return done&&!rx_failed;}
bool motor_dma_failed(motor_bus_id_t b){(void)b;return rx_failed;}
bool motor_dma_tx_failed(motor_bus_id_t b){(void)b;return false;}
bool motor_dma_read_byte(motor_bus_id_t b,uint8_t *v){(void)b;if(!rx_ready||rx_failed||consumed==sizeof reply)return false;*v=reply[consumed++];return true;}
void motor_dma_stop(motor_bus_id_t b){(void)b;stopped=true;}
void motor_dma_failure_snapshot(motor_bus_id_t b,motor_failure_snapshot_t *v){(void)b;(void)v;}
void motor_failure_record(unsigned b,motor_failure_snapshot_t *v){(void)b;(void)v;++failures;}
void motor_diagnostics_select_bus(unsigned b){(void)b;}
void motor_diagnostics_record(enum motor_diag_reason r){(void)r;}
void motor_diagnostics_tx(const uint8_t *p,size_t n){(void)p;(void)n;}
void motor_diagnostics_rx_byte(uint8_t b){(void)b;}
void uart_diagnostics_timeout(unsigned b){(void)b;++timeouts;}
void uart_diagnostics_tx_byte(unsigned b){(void)b;}
static void reset(enum scenario value){scenario=value;tick=reads=timeouts=failures=consumed=tx_checks=0;done=stopped=rx_ready=rx_failed=false;USART2->SR=0;}
int main(void){
    const uint8_t q[]={0xff,0xff,0xfe};uint8_t frame[20];size_t size;
    for(unsigned i=0;i<2;++i){const motor_bus_id_t bus=i?MOTOR_BUS_USART3:MOTOR_BUS_USART1;
        reset(TX_EDGE);motor_uart_failure_begin(bus,0x38,8,0x3f);
        assert(motor_uart_write(bus,q,sizeof q,5));assert(tx_checks==2&&!timeouts&&!failures&&!stopped);
        reset(RX_EDGE);motor_uart_failure_begin(bus,0x38,8,0x3f);
        assert(motor_uart_read_frame(bus,frame,sizeof frame,&size,5));
        assert(size==sizeof reply&&memcmp(frame,reply,size)==0&&!timeouts&&!failures);
        reset(SILENT);motor_uart_failure_begin(bus,0x38,8,0x3f);
        assert(!motor_uart_write(bus,q,sizeof q,5));assert(timeouts==1&&failures==1&&stopped);
        reset(SILENT);motor_uart_failure_begin(bus,0x38,8,0x3f);
        assert(!motor_uart_read_frame(bus,frame,sizeof frame,&size,5));assert(timeouts==1&&failures==1);
        reset(RX_ERROR_EDGE);motor_uart_failure_begin(bus,0x38,8,0x3f);
        assert(!motor_uart_read_frame(bus,frame,sizeof frame,&size,5));assert(consumed==0&&failures==1);
        motor_uart_failure_end(bus);
    }
    reset(HOST_TXE_EDGE);assert(host_uart_write_bytes(q,1));assert(!timeouts);
    reset(HOST_TC_EDGE);USART2->SR=USART_SR_TXE;assert(host_uart_write_bytes(q,1));assert(!timeouts);
    reset(SILENT);assert(!host_uart_write_bytes(q,1));assert(timeouts==1);
    puts("PASS production UART deadline edges: USART1/3 TX+RX, USART2 TXE+TC; silence and DMA errors still fail without deadline renewal");
}
