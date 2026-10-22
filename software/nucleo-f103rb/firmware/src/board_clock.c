#include "board_clock.h"

#include "stm32f1xx.h"

#define BOARD_APB1_HZ 32000000U
#define BOARD_APB2_HZ 64000000U

void board_clock_init(void)
{
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U) {
    }

    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_2;

    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2 |
                   RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PLLMULL16;

    RCC->CR |= RCC_CR_PLLON;
    while ((RCC->CR & RCC_CR_PLLRDY) == 0U) {
    }

    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
    }
    SystemCoreClockUpdate();
}

uint32_t board_apb1_clock_hz(void)
{
    return BOARD_APB1_HZ;
}

uint32_t board_apb2_clock_hz(void)
{
    return BOARD_APB2_HZ;
}
