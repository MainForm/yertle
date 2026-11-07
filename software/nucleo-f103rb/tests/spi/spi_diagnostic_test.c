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
static unsigned jump,jump_wait,frames;static uint32_t first_id;
uint32_t xTaskGetTickCount(void){return 1234;}
uint32_t pmw_spi_test_cycles(void){
 dwt.CYCCNT+=64;
 if(jump&&spi.DR==0x12&&!(spi.SR&SPI_SR_RXNE)&&++jump_wait==2){
  dwt.CYCCNT+=64000U*3U;spi.SR|=SPI_SR_RXNE;jump=0;
 }return dwt.CYCCNT;
}
static uint32_t word(const uint8_t*p){return p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void send(void*c,uint8_t type,const uint8_t*p,uint16_t n){
 (void)c;assert(type==0x93&&n==64&&p[0]==1&&p[1]==frames);
 assert(p[2]==0x34&&p[3]==0x12&&word(p+4)==1234);
 assert(word(p+8)==(frames<2?first_id:4U));++frames;
}
int main(void){
 uint8_t data[12];pmw_spi_failure_t f,first;
 pmw3901_spi_init();assert(!pmw3901_spi_failure_get(true,&f));
 pmw3901_spi_context(9,77);spi.SR=0;spi.CR2=0x123;gpioa.IDR=0x55;
 assert(!pmw3901_spi_read(0x16,data,12,true));
 assert(pmw3901_spi_failure_get(true,&first));first_id=first.meta[0];
 assert(first.meta[11]==PMW_SPI_WAIT_TXE&&first.meta[4]==2000);
 assert(first.meta[5]==3&&first.meta[6]==0x16&&first.meta[7]==12);
 assert(first.meta[9]==0&&first.meta[10]==0&&first.meta[12]==9);
 assert((first.meta[13]&65535)==77&&first.hardware[4]==0x123&&first.hardware[5]==0x55);
 assert(spi.CR2==0); /* captured before reset */
 spi.SR=SPI_SR_TXE;assert(!pmw3901_spi_write(0x12,0x34));
 assert(pmw3901_spi_failure_get(false,&f)&&f.meta[11]==PMW_SPI_WAIT_RXNE);
 assert(f.meta[9]==1&&f.meta[10]==0&&f.meta[8]==0&&f.hardware[11]==0x92);
 spi.SR=SPI_SR_TXE|SPI_SR_RXNE|SPI_SR_BSY;
 assert(!pmw3901_spi_read(0x16,data,12,true));
 assert(pmw3901_spi_failure_get(false,&f)&&f.meta[11]==PMW_SPI_WAIT_BSY);
 assert(f.meta[9]==13&&f.meta[10]==13&&f.meta[8]==12);
 spi.SR=SPI_SR_TXE|SPI_SR_RXNE;assert(pmw3901_spi_write(0x7f,7));
 pmw3901_spi_init();assert(pmw3901_spi_failure_get(true,&f)&&f.meta[0]==first_id);
 spi.SR=SPI_SR_TXE;spi.DR=0;jump=1;
 assert(!pmw3901_spi_read(0x12,data,1,false));
 assert(pmw3901_spi_failure_get(false,&f)&&f.meta[0]==4&&f.meta[11]==PMW_SPI_WAIT_RXNE);
 assert(f.hardware[2]&SPI_SR_RXNE);assert(f.hardware[13]>=3000);
 assert((f.meta[13]>>16)==7); /* retained bank and ready-after-preemption evidence */
 uint32_t dr=spi.DR;pmw3901_spi_send_diagnostics(0,send,0x1234);
 assert(frames==4&&spi.DR==dr); /* diagnostic getters never consume data */
 assert(pmw3901_spi_failure_get(true,&f)&&!memcmp(&first,&f,sizeof f));
 puts("PASS SPI TXE/RXNE/BSY, pre-reset capture, retained history, preemption boundary, 4 wire sections");
 return 0;
}
