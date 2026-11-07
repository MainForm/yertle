#pragma once
#include <stdint.h>
#define __IO volatile
typedef struct {volatile uint32_t SR,CR1,CR2,DR;} SPI_Type;
typedef struct {volatile uint32_t IDR,ODR,BSRR,CRL;} GPIO_Type;
typedef struct {volatile uint32_t APB2ENR,APB2RSTR;} RCC_Type;
typedef struct {volatile uint32_t MAPR;} AFIO_Type;
typedef struct {volatile uint32_t CTRL,CYCCNT;} DWT_Type;
typedef struct {volatile uint32_t DEMCR;} Debug_Type;
extern SPI_Type spi;extern GPIO_Type gpioa,gpioc;extern RCC_Type rcc;
extern AFIO_Type afio;extern DWT_Type dwt;extern Debug_Type debug;
extern uint32_t SystemCoreClock;
#define SPI1 (&spi)
#define GPIOA (&gpioa)
#define GPIOC (&gpioc)
#define RCC (&rcc)
#define AFIO (&afio)
#define DWT (&dwt)
#define CoreDebug (&debug)
#define __DMB() ((void)0)
#define CoreDebug_DEMCR_TRCENA_Msk 1U
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define RCC_APB2ENR_IOPAEN (1U<<2)
#define RCC_APB2ENR_IOPCEN (1U<<4)
#define RCC_APB2ENR_AFIOEN 1U
#define RCC_APB2ENR_SPI1EN (1U<<12)
#define RCC_APB2RSTR_SPI1RST (1U<<12)
#define AFIO_MAPR_SPI1_REMAP 1U
#define SPI_SR_TXE (1U<<1)
#define SPI_SR_RXNE 1U
#define SPI_SR_BSY (1U<<7)
#define SPI_CR1_MSTR (1U<<2)
#define SPI_CR1_SSM (1U<<9)
#define SPI_CR1_SSI (1U<<8)
#define SPI_CR1_CPOL (1U<<1)
#define SPI_CR1_CPHA 1U
#define SPI_CR1_BR_2 (1U<<5)
#define SPI_CR1_BR_0 (1U<<3)
#define SPI_CR1_SPE (1U<<6)
