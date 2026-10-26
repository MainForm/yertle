#include "mode_flash.h"
#include "calibration_store.h"
#include "host_protocol.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx.h"
#include <stddef.h>
extern uint8_t __mode_store_start__,__mode_store_end__;
#define MODE_BASE 0x0801f800UL
#define MODE_END 0x08020000UL
#define FLASH_ERROR_BITS (FLASH_SR_PGERR|FLASH_SR_WRPRTERR)
static bool region_valid(void){
    return (uintptr_t)&__mode_store_start__==MODE_BASE&&(uintptr_t)&__mode_store_end__==MODE_END&&
        *(volatile const uint16_t *)0x1ffff7e0UL==128U;
}
/* The whole busy interval executes in SRAM, with IRQs masked. DWT rather than
 * SysTick bounds waits. No library call or Flash-resident callback is allowed. */
__attribute__((section(".ramfunc"),noinline))
static bool flash_operation(uint32_t address,uint16_t value,bool erase_page){
    const uint32_t primask=__get_PRIMASK();__disable_irq();
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    uint32_t started=DWT->CYCCNT;bool ok=false;
    /* Flash write/erase requires HSI ON, even though SYSCLK uses PLL/HSE. */
    RCC->CR|=RCC_CR_HSION;
    while(!(RCC->CR&RCC_CR_HSIRDY))if((uint32_t)(DWT->CYCCNT-started)>720000U)goto done;
    if(FLASH->SR&FLASH_SR_BSY)goto done;
    if(FLASH->CR&FLASH_CR_LOCK){FLASH->KEYR=0x45670123UL;FLASH->KEYR=0xcdef89abUL;}
    if(FLASH->CR&FLASH_CR_LOCK)goto done;
    FLASH->SR=FLASH_ERROR_BITS|FLASH_SR_EOP;
    if(erase_page){FLASH->CR=FLASH_CR_PER;FLASH->AR=address;FLASH->CR|=FLASH_CR_STRT;}
    else{FLASH->CR=FLASH_CR_PG;*(volatile uint16_t *)(uintptr_t)address=value;}
    __DSB();started=DWT->CYCCNT;
    while(FLASH->SR&FLASH_SR_BSY)if((uint32_t)(DWT->CYCCNT-started)>7200000U)goto done;
    ok=(FLASH->SR&FLASH_ERROR_BITS)==0;
done:
    if(!(FLASH->SR&FLASH_SR_BSY)){FLASH->CR=FLASH_CR_LOCK;FLASH->SR=FLASH_ERROR_BITS|FLASH_SR_EOP;}
    __DSB();__ISB();__set_PRIMASK(primask);return ok;
}
bool mode_flash_read(uint32_t offset,uint16_t *value){
    if(!value||!region_valid()||(offset&1U)||offset>=MODE_FLASH_BYTES)return false;
    *value=*(volatile const uint16_t *)(uintptr_t)(MODE_BASE+offset);return true;
}
bool mode_flash_program(uint32_t offset,uint16_t value){
    uint16_t old;if(!mode_flash_read(offset,&old)||old!=0xffffU)return false;
    if(!flash_operation(MODE_BASE+offset,value,false))return false;
    uint16_t verify;return mode_flash_read(offset,&verify)&&verify==value;
}
bool mode_flash_erase(unsigned page){
    if(page>=2||!region_valid())return false;
    return flash_operation(MODE_BASE+page*MODE_FLASH_PAGE_BYTES,0,true);
}

extern uint8_t __calibration_store_start__,__calibration_store_end__;
#define CALIBRATION_BASE 0x0801f000UL
static bool calibration_region_valid(void){
    return region_valid()&&(uintptr_t)&__calibration_store_start__==CALIBRATION_BASE&&
        (uintptr_t)&__calibration_store_end__==MODE_BASE;
}
static bool calibration_write_allowed(void){
    return host_protocol_get_state()==HOST_FSM_DISARMED&&!host_protocol_is_stop_requested();
}
bool calibration_flash_read(uint32_t offset,uint16_t *value){
    if(!value||!calibration_region_valid()||(offset&1U)||offset>=CALIBRATION_FLASH_BYTES)return false;
    *value=*(volatile const uint16_t *)(uintptr_t)(CALIBRATION_BASE+offset);return true;
}
bool calibration_flash_program(uint32_t offset,uint16_t value){
    uint16_t old,verify;bool ok=false;
    /* Keep the DISARMED test atomic with starting Flash programming. */
    taskENTER_CRITICAL();
    if(calibration_write_allowed()&&calibration_flash_read(offset,&old)&&old==0xffffU)
        ok=flash_operation(CALIBRATION_BASE+offset,value,false);
    taskEXIT_CRITICAL();
    return ok&&calibration_flash_read(offset,&verify)&&verify==value;
}
bool calibration_flash_erase(unsigned page){
    if(page>=2||!calibration_region_valid())return false;
    taskENTER_CRITICAL();
    const bool ok=calibration_write_allowed()&&flash_operation(CALIBRATION_BASE+page*CALIBRATION_FLASH_PAGE_BYTES,0,true);
    taskEXIT_CRITICAL();return ok;
}
