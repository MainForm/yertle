#ifndef MOTOR_DMA_H
#define MOTOR_DMA_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* F103 port of Mowibox DMA transaction model; bus mutex held by caller. */
void motor_dma_init(void);
bool motor_dma_start(const uint8_t *data, size_t length);
bool motor_dma_tx_complete(void);
bool motor_dma_failed(void);
bool motor_dma_tx_failed(void);
bool motor_dma_read_byte(uint8_t *byte);
void motor_dma_stop(void);
#endif
