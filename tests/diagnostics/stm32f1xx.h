#ifndef TEST_STM32_H
#define TEST_STM32_H
#include <stdint.h>
typedef struct { volatile uint32_t CTRL, CYCCNT; } DWT_Type;
typedef struct { volatile uint32_t DEMCR; } CoreDebug_Type;
typedef struct { volatile uint32_t SR, BRR, CR1, CR3; } USART_Type;
typedef struct { volatile uint32_t CRH; } GPIO_Type;
typedef struct { volatile uint32_t ISER[2]; } NVIC_Type;
extern DWT_Type test_dwt;
extern CoreDebug_Type test_debug;
extern USART_Type test_usart;
extern GPIO_Type test_gpio;
extern NVIC_Type test_nvic;
extern uint32_t SystemCoreClock;
#define DWT (&test_dwt)
#define CoreDebug (&test_debug)
#define USART1 (&test_usart)
#define GPIOA (&test_gpio)
#define NVIC (&test_nvic)
#define USART1_IRQn 37U
#define USART_SR_ORE 8U
#define USART_SR_FE 2U
#define USART_SR_NE 4U
#define USART_SR_PE 1U
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define __DSB() ((void)0)
#define __NOP() ((void)0)
#endif
