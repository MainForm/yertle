#include "uart_diagnostics.h"
#include "motor_failure.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/* 0=USART1, 1=host USART2, 2=USART3. */
static volatile uart_isr_stats_t ports[3];
static volatile uint32_t tx_bytes[3], timeouts[3];
static volatile uint32_t rx_tail_bytes[3]; /* Task-owned DMA progress/residue. */
static volatile uint32_t rx_task_peak[3], rx_task_overflows[3];
static volatile uint32_t transport[16]; /* Per-field single writer; word 0 packed uptime. */
static uint64_t last_cycles[3]; /* Sole reader's cache; bounded retry fallback. */
static volatile uint32_t reasons[DIAG_REASON_COUNT];
static volatile uint32_t last_reason = 0xffffffffU;
static volatile uint32_t masks;
static volatile uint8_t tx_preview[16], rx_preview[16];
static volatile uint32_t tx_length, rx_length;
static uint32_t snapshot_max_cycles;
/* Motor tasks are serialized by the motor bus mutex; IRQ counters are per bus. */
static unsigned selected_motor_port;
/* Bus-mutex-serialized task writer, bounded non-blocking host reader. Never
 * spin waiting for a preempted lower-priority writer. IRQs remain enabled. */
static motor_failure_snapshot_t failure_first[2],failure_latest[2];
static volatile uint32_t failure_version[2],failure_count[2];
void motor_failure_record(unsigned b,motor_failure_snapshot_t *value){
    if(b>1U||!value)return;
    ++failure_version[b];__DMB();
    uint32_t id=failure_count[b]+1U;if(!id)id=1U;
    value->meta[0]=id;value->hw[0]=id;
    if(!failure_count[b])failure_first[b]=*value;
    failure_latest[b]=*value;failure_count[b]=id;
    __DMB();++failure_version[b];
}
bool motor_failure_get(unsigned b,bool first,motor_failure_snapshot_t *out){
    if(b>1U||!out)return false;
    for(unsigned attempt=0;attempt<3U;++attempt){
        const uint32_t version=failure_version[b];if(version&1U)continue;
        __DMB();*out=first?failure_first[b]:failure_latest[b];__DMB();
        if(version==failure_version[b])return out->meta[0]!=0U;
    }
    memset(out,0,sizeof *out);return false;
}
static volatile uint32_t bus_transport[2][16], bus_reasons[2][DIAG_REASON_COUNT];
static volatile uint32_t bus_last_reason[2] = {0xffffffffU,0xffffffffU};
static volatile uint8_t bus_tx_preview[2][16], bus_rx_preview[2][16];
static volatile uint32_t bus_tx_length[2], bus_rx_length[2];
static unsigned bus_index(unsigned port) { return port==2U?1U:0U; }
void motor_diagnostics_select_bus(unsigned port) { selected_motor_port=port; }

/* Called only by the owning UART ISR; no RTOS calls or output here. */
void uart_diagnostics_rx_error(volatile uart_isr_stats_t *s, uint32_t status, uint8_t byte)
{
    if (status & USART_SR_ORE) ++s->ore_errors;
    if (status & USART_SR_FE) ++s->fe_errors;
    if (status & USART_SR_NE) ++s->ne_errors;
    if (status & USART_SR_PE) ++s->pe_errors;
    s->last_error_sr = status;
    s->last_error_byte = byte;
    s->last_error_cycle = DWT->CYCCNT;
}

volatile uart_isr_stats_t *uart_diagnostics_port(unsigned port) { return &ports[port]; }
void uart_diagnostics_tx_byte(unsigned port) { ++tx_bytes[port]; }
void uart_diagnostics_tx_bytes(unsigned port, uint32_t count) { tx_bytes[port] += count; }
void uart_diagnostics_rx_tail_bytes(unsigned port, uint32_t count) { rx_tail_bytes[port] += count; }
void uart_diagnostics_rx_buffered(unsigned port, uint32_t count)
{ if (count > rx_task_peak[port]) rx_task_peak[port] = count; }
void uart_diagnostics_rx_overflow(unsigned port) { ++rx_task_overflows[port]; }
void uart_diagnostics_timeout(unsigned port) { ++timeouts[port]; }
void motor_transport_dma_mode(uint32_t capacity) { transport[1] = 1U; transport[12] = capacity; }
void motor_transport_record_bus(unsigned port, enum motor_transport_event event)
{ if ((port==0U||port==2U) && event>=2U && event<=9U) ++bus_transport[bus_index(port)][event]; }
void motor_transport_record(enum motor_transport_event event)
{ if(event>=2U && event<=9U) motor_transport_record_bus(selected_motor_port,event); else ++transport[event]; }
void host_transport_rx_dma_mode(uint32_t capacity) { transport[13] = 1U; transport[14] = capacity; }
void host_transport_rx_dma_error(void) { ++transport[15]; }

void uart_diagnostics_get_stats(unsigned port, uart_stats_t *o)
{
    volatile uart_isr_stats_t *s = &ports[port];
    o->tx_bytes = tx_bytes[port]; o->timeouts = timeouts[port];
    o->rx_bytes = s->rx_bytes + rx_tail_bytes[port]; o->rx_interrupts = s->rx_interrupts;
    o->rx_errors = s->rx_errors;
    o->rx_overflows = s->rx_overflows + rx_task_overflows[port];
    const uint32_t task_peak = rx_task_peak[port], isr_peak = s->rx_peak;
    o->rx_peak = task_peak > isr_peak ? task_peak : isr_peak;
    o->isr_calls = s->isr_calls;
    /* Task cannot run halfway through its UART ISR on this single-core MCU.
     * If an ISR rolls low over between loads, retry high/low/high. Never mask
     * IRQs or wait on a writer flag; retain the last sample after 3 retries. */
    for (unsigned attempt = 0; attempt < 3; ++attempt) {
        uint32_t high = s->cycles_high;
        uint32_t low = s->cycles_low;
        if (high == s->cycles_high) {
            last_cycles[port] = ((uint64_t) high << 32U) | low;
            break;
        }
    }
    o->isr_cycles = last_cycles[port];
    o->isr_max_cycles = s->isr_max_cycles;
    o->isr_timing_enabled = s->isr_timing_enabled;
    o->ore_errors = s->ore_errors; o->fe_errors = s->fe_errors;
    o->ne_errors = s->ne_errors; o->pe_errors = s->pe_errors;
    /* Last-error tuple and trace are advisory: updates may occur between
     * individual loads. They are not a coherent packet/fault decision. */
    o->last_error_sr = s->last_error_sr;
    o->last_error_byte = s->last_error_byte;
    o->last_error_cycle = s->last_error_cycle;
}

void uart_isr_timing_init(volatile uart_isr_stats_t *s)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    const uint32_t before = DWT->CYCCNT;
    __NOP(); __NOP(); __NOP(); __NOP();
    s->isr_timing_enabled = DWT->CYCCNT != before;
}

void uart_isr_timing_finish(volatile uart_isr_stats_t *s, uint32_t start)
{
    const uint32_t elapsed = DWT->CYCCNT - start;
    ++s->isr_calls;
    if (s->isr_timing_enabled) {
        uint32_t low = s->cycles_low;
        s->cycles_low = low + elapsed;
        if (s->cycles_low < low) ++s->cycles_high;
        if (elapsed > s->isr_max_cycles) s->isr_max_cycles = elapsed;
    }
}

void motor_diagnostics_record(enum motor_diag_reason reason)
{
    if (reason < DIAG_REASON_COUNT) { unsigned b=bus_index(selected_motor_port); ++reasons[reason]; last_reason = reason; ++bus_reasons[b][reason]; bus_last_reason[b]=reason; }
}

void motor_diagnostics_tx(const uint8_t *data, size_t length)
{
    tx_length = (uint32_t) length;
    rx_length = 0U;
    const unsigned b=bus_index(selected_motor_port); bus_tx_length[b]=(uint32_t)length; bus_rx_length[b]=0U;
    for (unsigned i = 0; i < 16; ++i) {
        tx_preview[i] = i < length ? data[i] : 0U;
        rx_preview[i] = 0U;
        bus_tx_preview[b][i] = i < length ? data[i] : 0U; bus_rx_preview[b][i]=0U;
    }
}

void motor_diagnostics_rx_byte(uint8_t byte)
{
    if (rx_length < sizeof(rx_preview)) rx_preview[rx_length] = byte;
    ++rx_length;
    const unsigned b=bus_index(selected_motor_port); if(bus_rx_length[b]<16U)bus_rx_preview[b][bus_rx_length[b]]=byte; ++bus_rx_length[b];
}

void motor_diagnostics_masks(uint16_t motion, uint16_t current)
{
    masks = motion | ((uint32_t) current << 16U);
}

static void pack(uint8_t *output, uint32_t value)
{
    for (unsigned i = 0; i < 4; ++i) output[i] = (uint8_t) (value >> (8U * i));
}

void uart_diagnostics_send(void *context, uart_diag_sender_t send)
{
    /* Non-blocking SPSC reads. UART IRQs remain enabled throughout the copy.
     * Counters/trace can represent slightly different instants. */
    uart_stats_t snapshot[2], motor3;
    uint32_t counters[DIAG_REASON_COUNT], last, valid, sr, txlen, rxlen, uptime;
    uint8_t tx[16], rx[16];
    uint32_t registers[5];
    uint32_t transport_snapshot[16];
    const uint32_t snapshot_start = DWT->CYCCNT;
    uart_diagnostics_get_stats(1U, &snapshot[1]);
    uart_diagnostics_get_stats(0U, &snapshot[0]);
    uart_diagnostics_get_stats(2U, &motor3);
    /* Preserve the existing motor-bus aggregate on the legacy wire format. */
#define ADD_FIELD(f) snapshot[0].f += motor3.f
    ADD_FIELD(tx_bytes); ADD_FIELD(rx_bytes); ADD_FIELD(rx_interrupts); ADD_FIELD(rx_errors);
    ADD_FIELD(rx_overflows); ADD_FIELD(timeouts); ADD_FIELD(isr_calls); ADD_FIELD(isr_cycles);
    ADD_FIELD(ore_errors); ADD_FIELD(fe_errors); ADD_FIELD(ne_errors); ADD_FIELD(pe_errors);
#undef ADD_FIELD
    if(motor3.rx_peak>snapshot[0].rx_peak)snapshot[0].rx_peak=motor3.rx_peak;
    if(motor3.isr_max_cycles>snapshot[0].isr_max_cycles)snapshot[0].isr_max_cycles=motor3.isr_max_cycles;
    for (unsigned i = 0; i < DIAG_REASON_COUNT; ++i) counters[i] = reasons[i];
    last = last_reason; valid = masks; sr = USART1->SR;
    txlen = tx_length; rxlen = rx_length;
    for (unsigned i = 0; i < 16; ++i) { tx[i] = tx_preview[i]; rx[i] = rx_preview[i]; }
    uptime = (uint32_t) (xTaskGetTickCount() * portTICK_PERIOD_MS);
    registers[0] = USART1->BRR; registers[1] = USART1->CR1;
    registers[2] = USART1->CR3; registers[3] = GPIOA->CRH;
    registers[4] = (NVIC->ISER[(unsigned) USART1_IRQn / 32U] >>
                    ((unsigned) USART1_IRQn % 32U)) & 1U;
    for (unsigned i = 1; i < 16; ++i) transport_snapshot[i] = transport[i];
    for (unsigned i = 2; i <= 9; ++i) transport_snapshot[i] = bus_transport[0][i]+bus_transport[1][i];
    const uint32_t snapshot_cycles = DWT->CYCCNT - snapshot_start;
    if (snapshot_cycles > snapshot_max_cycles) snapshot_max_cycles = snapshot_cycles;
    uint8_t payload[64];
    pack(payload, uptime);
    for (unsigned p = 0; p < 2; ++p) {
        const uart_stats_t *s = &snapshot[p];
        const uint32_t fields[] = {s->tx_bytes, s->rx_bytes, s->rx_interrupts,
            s->rx_errors, s->rx_overflows, s->rx_peak, s->timeouts};
        for (unsigned i = 0; i < 7; ++i) pack(&payload[4 + p * 28 + i * 4], fields[i]);
    }
    send(context, 0x85U, payload, 60U);
    pack(payload, uptime); pack(&payload[4], SystemCoreClock);
    for (unsigned p = 0; p < 2; ++p) {
        const uart_stats_t *s = &snapshot[p];
        unsigned o = 8 + p * 16;
        pack(&payload[o], s->isr_calls); pack(&payload[o + 4], (uint32_t) s->isr_cycles);
        pack(&payload[o + 8], (uint32_t) (s->isr_cycles >> 32U));
        pack(&payload[o + 12], s->isr_max_cycles);
    }
    pack(&payload[40], snapshot[0].isr_timing_enabled | (snapshot[1].isr_timing_enabled << 1U));
    send(context, 0x86U, payload, 44U);
    pack(payload, uptime); pack(&payload[4], last); pack(&payload[8], valid);
    pack(&payload[12], sr);
    for (unsigned i = 0; i < DIAG_REASON_COUNT; ++i) pack(&payload[16 + i * 4], counters[i]);
    send(context, 0x87U, payload, 64U);
    pack(payload, uptime); pack(&payload[4], txlen); pack(&payload[8], rxlen);
    memcpy(&payload[12], tx, 16); memcpy(&payload[28], rx, 16);
    for (unsigned i = 0; i < 5; ++i) pack(&payload[44 + i * 4], registers[i]);
    send(context, 0x88U, payload, 64U);
    pack(payload, uptime);
    for (unsigned p = 0; p < 2; ++p) {
        const uart_stats_t *s = &snapshot[p];
        const uint32_t fields[] = {s->ore_errors, s->fe_errors, s->ne_errors,
            s->pe_errors, s->last_error_sr, s->last_error_byte, s->last_error_cycle};
        for (unsigned i = 0; i < 7; ++i) pack(&payload[4 + p * 28 + i * 4], fields[i]);
    }
    pack(&payload[60], snapshot_max_cycles);
    send(context, 0x89U, payload, 64U);
    pack(payload, uptime);
    for (unsigned i = 1; i < 16; ++i) pack(&payload[i*4], transport_snapshot[i]);
    send(context, 0x8AU, payload, 64U);
}

/* Optional v1 per-bus extension. All fields little endian, sectioned <=64 B.
 * Separate call leaves six legacy response frames untouched. */
void uart_diagnostics_send_motor_buses(void *context, uart_diag_sender_t send, uint16_t request_sequence)
{
    uint8_t p[64];
    for(unsigned b=0;b<2U;++b){
        uart_stats_t s; const unsigned port=b?2U:0U;
        uart_diagnostics_get_stats(port,&s);
        const uint32_t now=(uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS);
        for(unsigned section=0;section<5U;++section){
            memset(p,0,sizeof p); p[0]=1U; p[1]=(uint8_t)(b?3U:1U); p[2]=(uint8_t)section;
            p[4]=(uint8_t)request_sequence; p[5]=(uint8_t)(request_sequence>>8U); pack(p+8,now);
            unsigned length=0;
            if(section==0U){const uint32_t busmask=b?0x0fc0U:0x003fU;
                const uint32_t valid=masks;const uint32_t f[]={s.tx_bytes,s.rx_bytes,s.rx_interrupts,s.rx_errors,s.rx_overflows,s.rx_peak,s.timeouts,busmask,valid&busmask,(valid>>16U)&busmask};
                for(unsigned i=0;i<10U;++i){pack(p+12U+i*4U,f[i]);} length=52U;}
            else if(section==1U){const uint32_t f[]={s.isr_calls,(uint32_t)s.isr_cycles,(uint32_t)(s.isr_cycles>>32U),s.isr_max_cycles,s.isr_timing_enabled,s.ore_errors,s.fe_errors,s.ne_errors,s.pe_errors,s.last_error_sr,s.last_error_byte,s.last_error_cycle};
                for(unsigned i=0;i<12U;++i){pack(p+12U+i*4U,f[i]);} length=60U;}
            else if(section==2U){pack(p+12,bus_last_reason[b]);for(unsigned i=0;i<DIAG_REASON_COUNT;++i)pack(p+16U+i*4U,bus_reasons[b][i]);length=64U;}
            else if(section==3U){pack(p+12,bus_tx_length[b]);pack(p+16,bus_rx_length[b]);for(unsigned i=0;i<16U;++i){p[20+i]=bus_tx_preview[b][i];p[36+i]=bus_rx_preview[b][i];}length=52U;}
            else {USART_TypeDef *u=b?USART3:USART1;pack(p+12,u->SR);pack(p+16,u->BRR);pack(p+20,u->CR1);pack(p+24,u->CR3);
                for(unsigned i=2;i<=9U;++i){pack(p+28U+(i-2U)*4U,bus_transport[b][i]);}length=60U;}
            send(context,0x91U,p,(uint16_t)length);
        }
        for(unsigned kind=0;kind<2U;++kind){
            motor_failure_snapshot_t failure={0};(void)motor_failure_get(b,kind==0U,&failure);
            for(unsigned half=0;half<2U;++half){
                memset(p,0,sizeof p);p[0]=1U;p[1]=(uint8_t)(b?3U:1U);
                p[2]=(uint8_t)(5U+kind*2U+half);p[4]=(uint8_t)request_sequence;p[5]=(uint8_t)(request_sequence>>8U);pack(p+8,now);
                const uint32_t *words=half?failure.hw:failure.meta;
                for(unsigned i=0;i<13U;++i)pack(p+12U+i*4U,words[i]);
                send(context,0x91U,p,64U);
            }
        }
    }
}
