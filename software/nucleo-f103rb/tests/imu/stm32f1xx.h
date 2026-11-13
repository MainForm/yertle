#ifndef IMU_TEST_STM32F1XX_H
#define IMU_TEST_STM32F1XX_H

#include <stdint.h>

typedef struct { volatile uint32_t CR1, CR2, OAR1, DR, SR1, SR2, CCR, TRISE; } I2C_Type;
typedef struct { volatile uint32_t CRL, IDR, BSRR, BRR; } GPIO_Type;
typedef struct { volatile uint32_t APB1ENR, APB2ENR; } RCC_Type;
typedef struct { volatile uint32_t DEMCR; } Debug_Type;
typedef struct { volatile uint32_t CTRL; } DWT_Type;

extern I2C_Type i2c;
extern GPIO_Type gpiob;
extern RCC_Type rcc;
extern Debug_Type debug;
extern DWT_Type dwt;
extern uint32_t SystemCoreClock;

#define I2C1 (&i2c)
#define GPIOB (&gpiob)
#define RCC (&rcc)
#define CoreDebug (&debug)
#define DWT (&dwt)
#define CoreDebug_DEMCR_TRCENA_Msk (1U << 24)
#define DWT_CTRL_CYCCNTENA_Msk 1U
#define RCC_APB1ENR_I2C1EN (1U << 21)
#define RCC_APB2ENR_IOPBEN (1U << 3)
#define I2C_CR1_PE 1U
#define I2C_CR1_START (1U << 8)
#define I2C_CR1_STOP (1U << 9)
#define I2C_CR1_ACK (1U << 10)
#define I2C_CR1_SWRST (1U << 15)
#define I2C_SR1_SB 1U
#define I2C_SR1_ADDR (1U << 1)
#define I2C_SR1_BTF (1U << 2)
#define I2C_SR1_RXNE (1U << 6)
#define I2C_SR1_BERR (1U << 8)
#define I2C_SR1_ARLO (1U << 9)
#define I2C_SR1_AF (1U << 10)
#define I2C_SR1_OVR (1U << 11)
#define I2C_SR1_TIMEOUT (1U << 14)
#define I2C_SR2_MSL 1U
#define I2C_SR2_BUSY (1U << 1)

uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);

#endif
