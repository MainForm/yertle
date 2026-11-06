#include "motor_uart.h"
#include "motor_dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "uart_diagnostics.h"
#include "motor_failure.h"
#include <string.h>
typedef struct {
    bool active,captured;uint32_t begin_cycle,wait_cycle,timeout_ms,request;
    uint16_t expected,received;
} failure_scope_t;
static failure_scope_t scopes[2];
static failure_scope_t *scope(motor_bus_id_t bus){return bus==MOTOR_BUS_USART1?&scopes[0]:bus==MOTOR_BUS_USART3?&scopes[1]:NULL;}
void motor_uart_failure_begin(motor_bus_id_t bus,uint8_t address,uint8_t length,uint16_t expected){
    failure_scope_t *s=scope(bus);if(!s)return;memset(s,0,sizeof *s);s->active=true;
    s->begin_cycle=DWT->CYCCNT;s->request=address|((uint32_t)length<<8U)|(0x82UL<<16U);s->expected=expected;
}
void motor_uart_failure_progress(motor_bus_id_t bus,uint16_t received){failure_scope_t *s=scope(bus);if(s)s->received=received;}
void motor_uart_failure_end(motor_bus_id_t bus){failure_scope_t *s=scope(bus);if(s)s->active=false;}
static void failure_wait(motor_bus_id_t bus,uint32_t timeout){failure_scope_t *s=scope(bus);if(s){s->wait_cycle=DWT->CYCCNT;s->timeout_ms=timeout;}}
void motor_uart_failure_note(motor_bus_id_t bus,enum motor_diag_reason reason){
    failure_scope_t *s=scope(bus);if(!s||!s->active||s->captured)return;
    s->captured=true;motor_failure_snapshot_t f={0};const uint32_t cycles=DWT->CYCCNT;
    const uint32_t cycles_per_us=SystemCoreClock/1000000U;
    f.meta[1]=(uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS);
    f.meta[2]=cycles_per_us?(cycles-s->begin_cycle)/cycles_per_us:0U;
    f.meta[3]=cycles_per_us?(cycles-s->wait_cycle)/cycles_per_us:0U;
    f.meta[4]=s->timeout_ms;f.meta[5]=(uint32_t)reason;f.meta[6]=s->request;
    f.meta[7]=s->expected;f.meta[8]=s->received;
    motor_dma_failure_snapshot(bus,&f);
    motor_failure_record(bus==MOTOR_BUS_USART1?0U:1U,&f);
}

void motor_uart_init(motor_bus_id_t bus) { motor_dma_init(bus); }
void motor_uart_discard_rx(motor_bus_id_t bus) { motor_diagnostics_select_bus(bus==MOTOR_BUS_USART1?0U:2U); motor_dma_stop(bus); }

bool motor_uart_write(motor_bus_id_t bus, const uint8_t *data, size_t length, uint32_t timeout_ms) {
    if (!data || !length) return false;
    failure_wait(bus,timeout_ms);
    motor_diagnostics_tx(data,length);
    if (!motor_dma_start(bus,data,length)) {
        motor_diagnostics_record(DIAG_BUILD_FAILED);motor_uart_failure_note(bus,DIAG_BUILD_FAILED); return false;
    }
    const TickType_t start=xTaskGetTickCount();
    TickType_t timeout=pdMS_TO_TICKS(timeout_ms);
    if (!timeout) timeout=1U;
    while (!motor_dma_tx_complete(bus)) {
        if (motor_dma_failed(bus)) {
            /* DMA TX failures already have a distinct transport counter. */
            if (!motor_dma_tx_failed(bus)) motor_diagnostics_record(DIAG_RX_ERROR);
            motor_uart_failure_note(bus,motor_dma_tx_failed(bus)?DIAG_TX_TIMEOUT:DIAG_RX_ERROR);
            motor_dma_stop(bus); return false;
        }
        if (xTaskGetTickCount()-start>=timeout) {
            uart_diagnostics_timeout(bus==MOTOR_BUS_USART1?0U:2U); motor_diagnostics_record(DIAG_TX_TIMEOUT);motor_uart_failure_note(bus,DIAG_TX_TIMEOUT);
            motor_dma_stop(bus); return false;
        }
        vTaskDelay(1U); /* DMA transfers while the owning task sleeps. */
    }
    return true;
}
static bool read_byte(motor_bus_id_t bus, uint8_t *byte, TickType_t start, TickType_t timeout, enum motor_diag_reason reason) {
    for (;;) {
        if (motor_dma_failed(bus)) { motor_diagnostics_record(DIAG_RX_ERROR);motor_uart_failure_note(bus,DIAG_RX_ERROR); return false; }
        if (motor_dma_read_byte(bus,byte)) { motor_diagnostics_rx_byte(*byte); return true; }
        if (xTaskGetTickCount()-start>=timeout) {
            uart_diagnostics_timeout(bus==MOTOR_BUS_USART1?0U:2U); motor_diagnostics_record(reason);motor_uart_failure_note(bus,reason); return false;
        }
        vTaskDelay(1U);
    }
}
bool motor_uart_read_frame(motor_bus_id_t bus, uint8_t *frame, size_t capacity, size_t *length, uint32_t timeout_ms) {
    if (!frame || !length || capacity<6U) return false;
    failure_wait(bus,timeout_ms);
    const TickType_t start=xTaskGetTickCount();
    TickType_t timeout=pdMS_TO_TICKS(timeout_ms);
    if (!timeout) timeout=1U;
    uint8_t byte=0;
    bool first_header=false, received_any=false;
    for (;;) {
        if (!read_byte(bus,&byte,start,timeout,received_any ? DIAG_HEADER_TIMEOUT : DIAG_NO_RESPONSE)) return false;
        received_any=true;
        if (byte==0xffU) {
            if (first_header) { frame[0]=0xffU; frame[1]=0xffU; break; }
            first_header=true;
        } else first_header=false;
    }
    if (!read_byte(bus,&frame[2],start,timeout,DIAG_BODY_TIMEOUT) ||
        !read_byte(bus,&frame[3],start,timeout,DIAG_BODY_TIMEOUT)) return false;
    const size_t total=(size_t)frame[3]+4U;
    if (total<6U || total>capacity) { motor_diagnostics_record(DIAG_BAD_LENGTH);motor_uart_failure_note(bus,DIAG_BAD_LENGTH); return false; }
    for (size_t i=4; i<total; ++i) if (!read_byte(bus,&frame[i],start,timeout,DIAG_BODY_TIMEOUT)) return false;
    *length=total;
    return true;
}
