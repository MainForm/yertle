#ifndef TEST_STM32_H
#define TEST_STM32_H
#include <stdint.h>
typedef struct { volatile uint32_t CTRL, CYCCNT; } DWT_Type;
typedef struct { volatile uint32_t DEMCR; } CoreDebug_Type;
typedef struct { volatile uint32_t SR, BRR, CR1, CR3, DR; } USART_Type;
typedef struct { volatile uint32_t CRH, CRL; } GPIO_Type;
typedef struct { volatile uint32_t ISER[2]; } NVIC_Type;
extern DWT_Type test_dwt;
extern CoreDebug_Type test_debug;
extern USART_Type test_usart;
extern USART_Type test_usart2;
extern GPIO_Type test_gpio;
extern NVIC_Type test_nvic;
extern uint32_t SystemCoreClock;
#define DWT (&test_dwt)
#define CoreDebug (&test_debug)
#define USART1 (&test_usart)
#define USART2 (&test_usart2)
#define GPIOA (&test_gpio)
#define NVIC (&test_nvic)
#define USART1_IRQn 37U
#define USART2_IRQn 38U
#define USART_SR_ORE 8U
#define USART_SR_FE 2U
#define USART_SR_NE 4U
#define USART_SR_PE 1U
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define __DSB() ((void)0)
#define __NOP() ((void)0)
typedef struct { volatile uint32_t AHBENR, APB2ENR, APB1ENR; } RCC_Type;
typedef struct { volatile uint32_t ISR, IFCR; } DMA_Type;
typedef struct { volatile uint32_t CCR, CNDTR; volatile uintptr_t CPAR, CMAR; } DMA_Channel_Type;
extern RCC_Type test_rcc;
extern DMA_Type test_dma;
extern DMA_Channel_Type test_dma4, test_dma5;
extern DMA_Channel_Type test_dma6;
#define RCC (&test_rcc)
#define DMA1 (&test_dma)
#define DMA1_Channel4 (&test_dma4)
#define DMA1_Channel5 (&test_dma5)
#define DMA1_Channel6 (&test_dma6)
#define DMA1_Channel6_IRQn 16U
#define DMA1_Channel4_IRQn 14U
#define DMA1_Channel5_IRQn 15U
#define USART_SR_IDLE (1U<<4)
#define USART_SR_TC (1U<<6)
#define USART_SR_TXE (1U<<7)
#define USART_SR_RXNE (1U<<5)
#define USART_CR3_EIE 1U
#define USART_CR3_DMAR (1U<<6)
#define USART_CR3_DMAT (1U<<7)
#define USART_CR1_RE (1U<<2)
#define USART_CR1_TE (1U<<3)
#define USART_CR1_IDLEIE (1U<<4)
#define USART_CR1_RXNEIE (1U<<5)
#define USART_CR1_PEIE (1U<<8)
#define USART_CR1_UE (1U<<13)
#define DMA_CCR_EN 1U
#define DMA_CCR_TCIE (1U<<1)
#define DMA_CCR_HTIE (1U<<2)
#define DMA_CCR_CIRC (1U<<5)
#define DMA_CCR_TEIE (1U<<3)
#define DMA_CCR_DIR (1U<<4)
#define DMA_CCR_MINC (1U<<7)
#define DMA_CCR_PL_1 (1U<<13)
#define DMA_CCR_PL (3U<<12)
#define DMA_ISR_TCIF4 (1U<<13)
#define DMA_ISR_TEIF4 (1U<<15)
#define DMA_ISR_TCIF5 (1U<<17)
#define DMA_ISR_TEIF5 (1U<<19)
#define DMA_IFCR_CGIF4 (1U<<12)
#define DMA_IFCR_CGIF5 (1U<<16)
#define DMA_ISR_TCIF6 (1U<<21)
#define DMA_ISR_HTIF6 (1U<<22)
#define DMA_ISR_TEIF6 (1U<<23)
#define DMA_IFCR_CGIF6 (1U<<20)
#define RCC_AHBENR_DMA1EN 1U
#define RCC_APB2ENR_IOPAEN (1U<<2)
#define RCC_APB2ENR_USART1EN (1U<<14)
#define RCC_APB1ENR_USART2EN (1U<<17)
#define GPIO_CRL_MODE2 (3U<<8)
#define GPIO_CRL_CNF2 (3U<<10)
#define GPIO_CRL_MODE2_0 (1U<<8)
#define GPIO_CRL_MODE2_1 (1U<<9)
#define GPIO_CRL_CNF2_1 (1U<<11)
#define GPIO_CRL_MODE3 (3U<<12)
#define GPIO_CRL_CNF3 (3U<<14)
#define GPIO_CRL_CNF3_0 (1U<<14)
#define GPIO_CRH_MODE9 (3U<<4)
#define GPIO_CRH_CNF9 (3U<<6)
#define GPIO_CRH_MODE9_0 (1U<<4)
#define GPIO_CRH_MODE9_1 (1U<<5)
#define GPIO_CRH_CNF9_1 (1U<<7)
#define GPIO_CRH_MODE10 (3U<<8)
#define GPIO_CRH_CNF10 (3U<<10)
#define GPIO_CRH_CNF10_0 (1U<<10)
#ifdef HOST_RX_DMA_TEST
void host_rx_dma_test_barrier(void);
#define __DMB() host_rx_dma_test_barrier()
#else
#define __DMB() ((void)0)
#endif
static inline void NVIC_DisableIRQ(unsigned n) { test_nvic.ISER[n/32U] &= ~(1U<<(n%32U)); }
static inline void NVIC_EnableIRQ(unsigned n) { test_nvic.ISER[n/32U] |= 1U<<(n%32U); }
static inline void NVIC_ClearPendingIRQ(unsigned n) { (void)n; }
static inline void NVIC_SetPriority(unsigned n,unsigned p) { (void)n; (void)p; }
#endif
