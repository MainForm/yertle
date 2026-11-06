#include <assert.h>
#include <stdio.h>
#include "uart_diagnostics.h"
#include "motor_failure.h"
DWT_Type test_dwt;
CoreDebug_Type test_debug;
USART_Type test_usart;
USART_Type test_usart3;
GPIO_Type test_gpio;
NVIC_Type test_nvic;
uint32_t SystemCoreClock = 64000000U;
static unsigned frames;
static unsigned extended_frames;
static uint32_t read32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}
static void receive_extended(void *context, uint8_t type, const uint8_t *p, uint16_t len) {
    (void)context; assert(type==0x91U); assert(p[0]==1U && p[4]==0x34U && p[5]==0x12U);
    assert(p[1]==(extended_frames<9U?1U:3U)); assert(p[2]==extended_frames%9U);
    assert(len<=64U); assert(read32(p+8)==1000U);
    if(p[1]==3U && p[2]==0U){assert(read32(p+12)==7U);assert(read32(p+16)==9U);assert(read32(p+40)==0x0fc0U);}
    if(p[1]==3U && p[2]==2U)assert(read32(p+16+DIAG_NO_RESPONSE*4U)==1U);
    if(p[2]>=5U){
        assert(len==64U);
        if(p[1]==1U){
            assert(read32(p+12)==(p[2]<7U?1U:2U));
            if(p[2]==5U)assert(read32(p+16)==789U);
            if(p[2]==7U)assert(read32(p+16)==900U);
            if(p[2]==6U||p[2]==8U)assert(read32(p+16)==0xc0U);
        }else assert(read32(p+12)==0U);
    }
    ++extended_frames;
}
static void receive(void *context, uint8_t type, const uint8_t *p, uint16_t len) {
    (void)context;
    assert(type == 0x85U + frames++);
    assert(len == (type == 0x85 ? 60 : type == 0x86 ? 44 : 64));
    assert(read32(p) == 1000);
    if (type == 0x87) assert(read32(p+16+DIAG_VALID_REPLY*4) == 1);
    if (type == 0x88) {
        assert(read32(p+4) == 3 && read32(p+8) == 1);
        assert(p[12] == 0xff && p[28] == 0xab);
    }
}
int main(void) {
    volatile uart_isr_stats_t *s = uart_diagnostics_port(0);
    uart_stats_t copy;
    s->isr_timing_enabled = 1; s->cycles_high = 7; s->cycles_low = 0xfffffff0U;
    test_dwt.CYCCNT = 100;
    uart_isr_timing_finish(s, 68); /* low-word carry */
    uart_diagnostics_tx_byte(0); uart_diagnostics_timeout(0);
    uart_diagnostics_rx_error(s, USART_SR_ORE | USART_SR_FE, 0x80);
    uart_diagnostics_get_stats(0, &copy);
    assert(copy.isr_cycles == (((uint64_t)8<<32) | 16));
    assert(copy.isr_calls == 1 && copy.isr_max_cycles == 32);
    assert(copy.tx_bytes == 1 && copy.timeouts == 1);
    assert(copy.ore_errors == 1 && copy.fe_errors == 1 && copy.ne_errors == 0);
    uart_diagnostics_get_stats(1, &copy);
    assert(copy.tx_bytes == 0 && copy.isr_cycles == 0 && copy.timeouts == 0);
    const uint8_t packet[] = {0xff, 0xff, 1};
    motor_diagnostics_tx(packet, sizeof packet); motor_diagnostics_rx_byte(0xab);
    motor_diagnostics_record(DIAG_VALID_REPLY); motor_diagnostics_masks(0x0fff, 0x0fff);
    uart_diagnostics_tx_bytes(2,7); uart_diagnostics_port(2)->rx_bytes=9;
    motor_diagnostics_select_bus(2);motor_diagnostics_record(DIAG_NO_RESPONSE);
    uart_diagnostics_send(NULL, receive);
    assert(frames == 6);
    motor_failure_snapshot_t f={0},saved;
    f.meta[1]=789U;f.hw[1]=0xc0U;motor_failure_record(0,&f);
    f.meta[1]=900U;motor_failure_record(0,&f);
    assert(motor_failure_get(0,true,&saved)&&saved.meta[0]==1U&&saved.meta[1]==789U);
    assert(motor_failure_get(0,false,&saved)&&saved.meta[0]==2U&&saved.hw[0]==2U);
    uart_diagnostics_send_motor_buses(NULL, receive_extended, 0x1234U);
    assert(extended_frames==18U);
    puts("firmware diagnostics tests passed (ownership, carry, serialization; no hardware IRQ simulation)");
}
