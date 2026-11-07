#ifdef NDEBUG
#undef NDEBUG
#endif
#include "stm32f1xx.h"
#include "pmw3901_spi.h"
#include "pmw3901_spi_diagnostic.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
SPI_Type spi;GPIO_Type gpioa,gpioc;RCC_Type rcc;AFIO_Type afio;
DWT_Type dwt;Debug_Type debug;uint32_t SystemCoreClock=64000000U;
static unsigned jump_us,jump_wait,step_us=1,frames;static uint32_t latest_id;
uint32_t xTaskGetTickCount(void){return 1234;}
uint32_t pmw_spi_test_cycles(void){
 dwt.CYCCNT+=64*step_us;
 if(jump_us&&spi.DR==0x12&&!(spi.SR&SPI_SR_RXNE)&&++jump_wait==2){
  dwt.CYCCNT+=64*jump_us;spi.SR|=SPI_SR_RXNE;jump_us=0;
 }return dwt.CYCCNT;
}
static uint32_t word(const uint8_t*p){return p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void send(void*c,uint8_t type,const uint8_t*p,uint16_t n){
 (void)c;assert(type==0x93&&n==64&&p[0]==2&&p[1]==frames);
 assert(p[2]==0x34&&p[3]==0x12&&word(p+4)==1234);
 assert(word(p+8)==(frames<2?1U:latest_id));++frames;
}
static void drain(void){uint8_t p[64];while(pmw3901_spi_auto_payload(p))pmw3901_spi_auto_result(true);}
int main(void){
 uint8_t data[12],p[64],retry[64];pmw_spi_failure_t f,first;
 pmw3901_spi_init();assert(!pmw3901_spi_failure_get(true,&f));
 assert(!pmw3901_spi_auto_payload(p));
 pmw3901_spi_context(9,77);spi.SR=0;spi.CR2=0x123;gpioa.IDR=0x55;
 pmw3901_spi_call_begin(20000);assert(!pmw3901_spi_read(0x16,data,12,true));assert(pmw3901_spi_call_end());
 assert(pmw3901_spi_failure_get(true,&first));
 assert(first.meta[11]==PMW_SPI_WAIT_TXE&&first.meta[4]==500&&first.meta[3]>=500);
 assert((first.meta[5]&255)==3&&first.meta[6]==0x16&&first.meta[7]==12);
 assert(first.meta[9]==0&&first.meta[10]==0&&(first.meta[12]&255)==9);
 assert((first.meta[13]&65535)==77&&first.hardware[4]==0x123&&first.hardware[5]==0x55);
 assert(spi.CR2==0);
 /* Automatic event exists without manual query, and failed TX retains half. */
 assert(pmw3901_spi_auto_payload(p)&&p[1]==4&&word(p+8)==1);
 pmw3901_spi_auto_result(false);assert(pmw3901_spi_auto_payload(retry)&&!memcmp(p,retry,64));
 pmw3901_spi_auto_result(true);assert(pmw3901_spi_auto_payload(p)&&p[1]==5&&word(p+8)==1);
 pmw3901_spi_auto_result(true);assert(!pmw3901_spi_auto_payload(p));
 spi.SR=SPI_SR_TXE;assert(!pmw3901_spi_read(0x11,data,1,false));
 assert(pmw3901_spi_failure_get(false,&f)&&f.meta[11]==PMW_SPI_WAIT_RXNE);
 assert(f.meta[3]>=500&&f.meta[2]>=550&&f.meta[9]==1&&f.meta[10]==0);
 spi.SR=SPI_SR_TXE|SPI_SR_RXNE|SPI_SR_BSY;assert(!pmw3901_spi_read(0x16,data,12,true));
 assert(pmw3901_spi_failure_get(false,&f)&&f.meta[11]==PMW_SPI_WAIT_BSY);
 assert(f.meta[3]>=500&&f.meta[9]==13&&f.meta[10]==13&&f.meta[8]==12);
 drain();
 /* Total >2ms succeeds under task budget; delays do not spend flag budget. */
 spi.SR=SPI_SR_TXE|SPI_SR_RXNE;step_us=100;uint32_t begin=dwt.CYCCNT;
 pmw3901_spi_call_begin(20000);assert(pmw3901_spi_read(0x16,data,12,true));assert(pmw3901_spi_call_end());
 assert((dwt.CYCCNT-begin)/64>2000);step_us=1;
 /* A ready-after-preemption flag wins over the narrow stage deadline. */
 spi.SR=SPI_SR_TXE;spi.DR=0;jump_us=3000;jump_wait=0;
 pmw3901_spi_call_begin(20000);assert(pmw3901_spi_read(0x12,data,1,false));assert(pmw3901_spi_call_end());
 assert(!pmw3901_spi_auto_payload(p));
 /* Conservative task budget still wins even when hardware is ready. */
 spi.SR=SPI_SR_TXE;spi.DR=0;jump_us=25000;jump_wait=0;
 pmw3901_spi_call_begin(20000);assert(!pmw3901_spi_read(0x12,data,1,false));assert(!pmw3901_spi_call_end());
 assert(pmw3901_spi_failure_get(false,&f)&&f.meta[11]==PMW_SPI_CALL_BUDGET&&f.meta[4]==20000);
 assert((f.meta[12]>>8)==PMW_SPI_WAIT_RXNE&&f.meta[3]>=25000&&f.hardware[13]>=25000);
 drain();
 /* Late call return, even without I/O, is classified separately. */
 pmw3901_spi_call_begin(20000);dwt.CYCCNT+=64*21000;assert(!pmw3901_spi_call_end());
 assert(pmw3901_spi_failure_get(false,&f)&&f.meta[11]==PMW_SPI_CALL_BUDGET);
 drain();
 spi.SR=SPI_SR_TXE|SPI_SR_RXNE;assert(pmw3901_spi_read(0,data,1,false));pmw3901_spi_id_error(0x49);
 assert(pmw3901_spi_failure_get(false,&f)&&f.meta[11]==PMW_SPI_SENSOR_ID&&(f.meta[13]>>24)==0x49);
 drain();
 /* Bounded overflow is observable; manual latest remains available. */
 for(unsigned i=0;i<6;++i){spi.SR=0;assert(!pmw3901_spi_write(0x13,0));}
 assert(pmw3901_spi_auto_payload(p)&&p[2]==2&&p[3]==0);drain();
 assert(pmw3901_spi_failure_get(false,&f));latest_id=f.meta[0];
 uint32_t dr=spi.DR;pmw3901_spi_send_diagnostics(0,send,0x1234);assert(frames==4&&spi.DR==dr);
 assert(pmw3901_spi_failure_get(true,&f)&&!memcmp(&first,&f,sizeof f));
 pmw3901_spi_init();assert(pmw3901_spi_failure_get(false,&f)&&f.meta[0]==latest_id);
 puts("PASS renewed stage deadlines, task budget, final flag recheck, ID error, automatic queue/retry/overflow and retention");return 0;
}
