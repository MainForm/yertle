#include "pmw3901_spi.h"
#include "pmw3901_spi_diagnostic.h"
#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* NUCLEO-F103RB: PA5/6/7 SPI1, PA4 CS, PA0 active-low reset.
 * PB6 is IMU SCL; SPI DMA1 channels 2/3 belong to USART3.
 * Each flag wait has its own deadline. The sensor task supplies a separate
 * conservative call budget, checked cooperatively even when flags are ready.
 * Diagnostic snapshots never read DR or mask interrupts. */
typedef struct {
    uint32_t begin,wait_begin,max_poll_gap,operation,address,length,byte_index;
    uint32_t tx_bytes,rx_bytes,last_tx,last_rx,stage;
} spi_operation_t;
static spi_operation_t op;
static uint8_t device_state,register_bank=255U;
static uint16_t config_index;
static uint8_t expected_id;
static pmw_spi_failure_t first_failure,latest_failure;
static volatile uint32_t failure_version,failure_count;
static bool call_active,call_failed;
static uint32_t call_begin,call_budget;
#define AUTO_CAPACITY 4U
static pmw_spi_failure_t automatic[AUTO_CAPACITY];
static volatile uint32_t auto_write,auto_read,auto_dropped;
static unsigned auto_half;


#ifdef PMW_SPI_TEST
extern uint32_t pmw_spi_test_cycles(void);
#endif
static uint32_t spi_cycles(void){
#ifdef PMW_SPI_TEST
    return pmw_spi_test_cycles();
#else
    return DWT->CYCCNT;
#endif
}
static uint32_t cycles_us(uint32_t us){return (SystemCoreClock/1000000U)*us;}
static uint32_t as_us(uint32_t cycles){const uint32_t unit=SystemCoreClock/1000000U;return unit?cycles/unit:0U;}
static void delay_us(uint32_t us){uint32_t begin=spi_cycles();while((uint32_t)(spi_cycles()-begin)<cycles_us(us)){} }
void pmw3901_spi_context(uint8_t state,uint16_t index){device_state=state;config_index=index;}
static void begin_operation(unsigned operation,unsigned address,size_t length){
    memset(&op,0,sizeof op);op.operation=operation;op.address=address;op.length=(uint32_t)length;op.last_tx=op.last_rx=256U;op.begin=spi_cycles();op.wait_begin=op.begin;
}
static void capture_failure(unsigned stage,uint32_t timeout_us){
    pmw_spi_failure_t f={0};const uint32_t now=spi_cycles();
    /* These reads are advisory, not an atomic hardware snapshot. Never read DR
     * merely for diagnostics: that could consume RX data or clear OVR/MODF. */
    f.hardware[1]=now;f.hardware[2]=SPI1->SR;f.hardware[3]=SPI1->CR1;f.hardware[4]=SPI1->CR2;
    f.hardware[5]=GPIOA->IDR;f.hardware[6]=GPIOA->ODR;f.hardware[7]=RCC->APB2ENR;
    f.hardware[8]=RCC->APB2RSTR;f.hardware[9]=AFIO->MAPR;f.hardware[10]=SystemCoreClock;
    f.hardware[11]=op.last_tx;f.hardware[12]=op.last_rx;f.hardware[13]=as_us(op.max_poll_gap);
    f.meta[1]=(uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS);f.meta[2]=as_us(now-op.begin);
    f.meta[3]=as_us(now-(stage==PMW_SPI_CALL_BUDGET?call_begin:op.wait_begin));f.meta[4]=timeout_us;
    uint32_t call_us=call_active?as_us(now-call_begin):0U;if(call_us>0xffffffU)call_us=0xffffffU;
    f.meta[5]=op.operation|(call_us<<8U);
    f.meta[6]=op.address;f.meta[7]=op.length;f.meta[8]=op.byte_index;f.meta[9]=op.tx_bytes;f.meta[10]=op.rx_bytes;
    f.meta[11]=stage;f.meta[12]=(uint32_t)device_state|(op.stage<<8U);f.meta[13]=(uint32_t)config_index|((uint32_t)register_bank<<16U)|((uint32_t)expected_id<<24U);
    ++failure_version;__DMB();uint32_t id=failure_count+1U;if(!id)id=1U;f.meta[0]=f.hardware[0]=id;
    if(!failure_count){first_failure=f;}
    latest_failure=f;failure_count=id;__DMB();++failure_version;
    if(call_active)call_failed=true;
    /* Single sensor producer / single host consumer. A slot is immutable
     * until the host acknowledges both halves. Keep UART out of this path. */
    const uint32_t write=auto_write;
    if((uint32_t)(write-auto_read)<AUTO_CAPACITY){automatic[write%AUTO_CAPACITY]=f;__DMB();auto_write=write+1U;}
    else if(auto_dropped!=UINT32_MAX)++auto_dropped;
}
bool pmw3901_spi_failure_get(bool first,pmw_spi_failure_t*out){
    if(!out)return false;
    for(unsigned attempt=0;attempt<3U;++attempt){const uint32_t version=failure_version;if(version&1U)continue;__DMB();*out=first?first_failure:latest_failure;__DMB();if(version==failure_version)return out->meta[0]!=0U;}
    memset(out,0,sizeof *out);return false;
}
void pmw3901_spi_call_begin(uint32_t timeout_us){
    begin_operation(0U,256U,0U);call_begin=spi_cycles();call_budget=timeout_us;
    call_failed=false;call_active=true;
}
static bool call_expired(uint32_t now){
    return call_active&&(uint32_t)(now-call_begin)>=cycles_us(call_budget);
}
bool pmw3901_spi_call_end(void){
    const bool expired=call_expired(spi_cycles());
    if(expired&&!call_failed)capture_failure(PMW_SPI_CALL_BUDGET,call_budget);
    call_active=false;return !expired;
}
void pmw3901_spi_id_error(uint8_t expected){
    expected_id=expected;capture_failure(PMW_SPI_SENSOR_ID,0U);expected_id=0U;
}
static bool wait_flag(uint32_t flag,bool set,unsigned stage){
    op.stage=stage;op.wait_begin=spi_cycles();uint32_t previous=op.wait_begin;
    for(;;){
        const bool ready=((SPI1->SR&flag)!=0U)==set;
        const uint32_t now=spi_cycles(),gap=now-previous;previous=now;
        if(gap>op.max_poll_gap)op.max_poll_gap=gap;
        if(call_expired(now)){capture_failure(PMW_SPI_CALL_BUDGET,call_budget);return false;}
        if(ready)return true;
        if((uint32_t)(now-op.wait_begin)>=cycles_us(PMW_SPI_STAGE_TIMEOUT_US)){
            /* Completion can occur after the initial SR predicate. */
            if((((SPI1->SR&flag)!=0U)==set))return true;
            capture_failure(stage,PMW_SPI_STAGE_TIMEOUT_US);return false;
        }
    }
}
void pmw3901_spi_select(bool selected){GPIOA->BSRR=selected?(1U<<(4+16)):(1U<<4);}
void pmw3901_spi_reset(bool asserted){GPIOA->BSRR=asserted?(1U<<16):(1U<<0);if(asserted)register_bank=255U;}
void pmw3901_spi_init(void){
    RCC->APB2ENR|=RCC_APB2ENR_IOPAEN|RCC_APB2ENR_AFIOEN|RCC_APB2ENR_SPI1EN;
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    GPIOA->BSRR=(1U<<0)|(1U<<4); /* high BEFORE output mode */
    GPIOA->CRL=(GPIOA->CRL&~((15U<<0)|(15U<<16)|(15U<<20)|(15U<<24)|(15U<<28)))|
        (2U<<0)|(2U<<16)|(11U<<20)|(4U<<24)|(11U<<28);
    /* PC7/MOT is optional: input, initially polled SPI; no EXTI ownership. */
    RCC->APB2ENR|=RCC_APB2ENR_IOPCEN;
    GPIOC->CRL=(GPIOC->CRL&~(15U<<28))|(4U<<28);
    AFIO->MAPR&=~AFIO_MAPR_SPI1_REMAP;
    SPI1->CR1=0;SPI1->CR2=0;
    /* APB2=64MHz /64=1MHz, mode3, MSB-first, software NSS. */
    SPI1->CR1=SPI_CR1_MSTR|SPI_CR1_SSM|SPI_CR1_SSI|SPI_CR1_CPOL|SPI_CR1_CPHA|
        SPI_CR1_BR_2|SPI_CR1_BR_0|SPI_CR1_SPE;
}
static bool exchange(uint8_t value,uint8_t *out){
    if(!wait_flag(SPI_SR_TXE,true,PMW_SPI_WAIT_TXE))return false;
    *(__IO uint8_t*)&SPI1->DR=value;++op.tx_bytes;op.last_tx=value;
    if(!wait_flag(SPI_SR_RXNE,true,PMW_SPI_WAIT_RXNE))return false;
    *out=*(__IO uint8_t*)&SPI1->DR;++op.rx_bytes;op.last_rx=*out;return true;
}
static bool finish(bool ok){
    if(ok)ok=wait_flag(SPI_SR_BSY,false,PMW_SPI_WAIT_BSY);
    delay_us(50);pmw3901_spi_select(false);delay_us(200);
    if(!ok){RCC->APB2RSTR|=RCC_APB2RSTR_SPI1RST;RCC->APB2RSTR&=~RCC_APB2RSTR_SPI1RST;pmw3901_spi_init();}
    return ok;
}
bool pmw3901_spi_read(uint8_t address,uint8_t *data,size_t n,bool burst){
    begin_operation(burst?3U:1U,address,n);
    if(!data||!n||n>12){capture_failure(PMW_SPI_BAD_ARGUMENT,0U);return false;}
    uint8_t ignored;
    pmw3901_spi_select(true);delay_us(50);
    bool ok=exchange(address&0x7fU,&ignored);delay_us(burst?50U:500U);
    for(size_t i=0;ok&&i<n;i++){op.byte_index=(uint32_t)i+1U;ok=exchange(0,&data[i]);}
    return finish(ok);
}
bool pmw3901_spi_write(uint8_t address,uint8_t value){
    begin_operation(2U,address,1U);uint8_t ignored;
    pmw3901_spi_select(true);delay_us(50);
    bool ok=exchange(address|0x80U,&ignored);delay_us(50);
    if(ok){op.byte_index=1U;ok=exchange(value,&ignored);}
    ok=finish(ok);if(ok&&address==0x7fU)register_bank=value;return ok;
}
static void spi_pack32(uint8_t *p,uint32_t v){for(unsigned i=0;i<4;++i)p[i]=(uint8_t)(v>>(i*8U));}
void pmw3901_spi_send_diagnostics(void *context,pmw_spi_diag_sender_t send,uint16_t request){
    uint8_t payload[64];
    for(unsigned kind=0;kind<2U;++kind){pmw_spi_failure_t f={0};(void)pmw3901_spi_failure_get(kind==0U,&f);
        for(unsigned half=0;half<2U;++half){memset(payload,0,sizeof payload);payload[0]=2U;payload[1]=(uint8_t)(kind*2U+half);payload[2]=(uint8_t)request;payload[3]=(uint8_t)(request>>8U);
            spi_pack32(payload+4,(uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS));
            const uint32_t *words=half?f.hardware:f.meta;for(unsigned i=0;i<14U;++i)spi_pack32(payload+8U+i*4U,words[i]);
            send(context,PMW_SPI_DIAGNOSTIC_TYPE,payload,sizeof payload);
        }
    }
}

bool pmw3901_spi_auto_payload(uint8_t payload[64]){
    if(auto_read==auto_write)return false;
    __DMB();const pmw_spi_failure_t *f=&automatic[auto_read%AUTO_CAPACITY];
    memset(payload,0,64U);payload[0]=2U;payload[1]=(uint8_t)(4U+auto_half);
    const uint32_t lost=auto_dropped>65535U?65535U:auto_dropped;
    payload[2]=(uint8_t)lost;payload[3]=(uint8_t)(lost>>8U);
    spi_pack32(payload+4,(uint32_t)(xTaskGetTickCount()*portTICK_PERIOD_MS));
    const uint32_t *words=auto_half?f->hardware:f->meta;
    for(unsigned i=0;i<14U;++i)spi_pack32(payload+8U+i*4U,words[i]);
    return true;
}
void pmw3901_spi_auto_result(bool sent){
    if(!sent||auto_read==auto_write)return;
    if(auto_half==0U)auto_half=1U;
    else{auto_half=0U;__DMB();++auto_read;}
}
