#include "pmw3901_spi.h"
#include "stm32f1xx.h"

/* NUCLEO-F103RB: PA5/6/7 SPI1, PA4 CS, PA0 active-low reset.
 * PB6 is already IMU SCL. SPI1 DMA1 channels 2/3 belong to USART3;
 * all SPI I/O below is polling with interrupts enabled and a cycle deadline. */
static uint32_t cycles_us(uint32_t us){return (SystemCoreClock/1000000U)*us;}
static void delay_us(uint32_t us){uint32_t begin=DWT->CYCCNT;while((uint32_t)(DWT->CYCCNT-begin)<cycles_us(us)){} }
static bool wait_flag(uint32_t flag,uint32_t begin){
    while(!(SPI1->SR&flag))if((uint32_t)(DWT->CYCCNT-begin)>cycles_us(2000U))return false;
    return true;
}
void pmw3901_spi_select(bool selected){GPIOA->BSRR=selected?(1U<<(4+16)):(1U<<4);}
void pmw3901_spi_reset(bool asserted){GPIOA->BSRR=asserted?(1U<<16):(1U<<0);}
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
static bool exchange(uint8_t value,uint8_t *out,uint32_t begin){
    if(!wait_flag(SPI_SR_TXE,begin))return false;
    *(__IO uint8_t*)&SPI1->DR=value;
    if(!wait_flag(SPI_SR_RXNE,begin))return false;
    *out=*(__IO uint8_t*)&SPI1->DR;return true;
}
static bool finish(bool ok,uint32_t begin){
    while(ok&&(SPI1->SR&SPI_SR_BSY))if((uint32_t)(DWT->CYCCNT-begin)>cycles_us(2000U))ok=false;
    delay_us(50);pmw3901_spi_select(false);delay_us(200);
    if(!ok){RCC->APB2RSTR|=RCC_APB2RSTR_SPI1RST;RCC->APB2RSTR&=~RCC_APB2RSTR_SPI1RST;pmw3901_spi_init();}
    return ok;
}
bool pmw3901_spi_read(uint8_t address,uint8_t *data,size_t n,bool burst){
    if(!data||!n||n>12)return false;
    uint32_t begin=DWT->CYCCNT;uint8_t ignored;
    pmw3901_spi_select(true);delay_us(50);
    bool ok=exchange(address&0x7fU,&ignored,begin);delay_us(burst?50U:500U);
    for(size_t i=0;ok&&i<n;i++)ok=exchange(0,&data[i],begin);
    return finish(ok,begin);
}
bool pmw3901_spi_write(uint8_t address,uint8_t value){
    uint32_t begin=DWT->CYCCNT;uint8_t ignored;
    pmw3901_spi_select(true);delay_us(50);
    bool ok=exchange(address|0x80U,&ignored,begin);delay_us(50);
    if(ok)ok=exchange(value,&ignored,begin);
    return finish(ok,begin);
}
