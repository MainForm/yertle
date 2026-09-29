#include <assert.h>
#include <stdio.h>
#include "uart_diagnostics.h"
DWT_Type test_dwt;
CoreDebug_Type test_debug;
USART_Type test_usart;
GPIO_Type test_gpio;
NVIC_Type test_nvic;
uint32_t SystemCoreClock = 64000000U;
void host_uart_get_stats(uart_stats_t *s) { uart_diagnostics_get_stats(1, s); }
void motor_uart_get_stats(uart_stats_t *s) { uart_diagnostics_get_stats(0, s); }
static unsigned frames;
static uint32_t read32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
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
    motor_diagnostics_record(DIAG_VALID_REPLY); motor_diagnostics_masks(0x3f, 0x3f);
    uart_diagnostics_send(NULL, receive);
    assert(frames == 6);
    puts("firmware diagnostics tests passed (ownership, carry, serialization; no hardware IRQ simulation)");
}
