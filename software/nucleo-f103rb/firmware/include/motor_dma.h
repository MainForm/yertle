#ifndef MOTOR_DMA_H
#define MOTOR_DMA_H
#include "motor_bus_map.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
/* F103 port of Mowibox DMA transaction model; bus mutex held by caller. */
void motor_dma_init(motor_bus_id_t bus);
bool motor_dma_start(motor_bus_id_t bus, const uint8_t *data, size_t length);
bool motor_dma_tx_complete(motor_bus_id_t bus);
bool motor_dma_failed(motor_bus_id_t bus);
bool motor_dma_tx_failed(motor_bus_id_t bus);
bool motor_dma_read_byte(motor_bus_id_t bus, uint8_t *byte);
void motor_dma_stop(motor_bus_id_t bus);
#endif
