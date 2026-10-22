#ifndef HOST_RX_DMA_H
#define HOST_RX_DMA_H
#include <stdbool.h>
#include <stdint.h>

#define HOST_RX_DMA_CAPACITY 1024U

/* USART2 RX, DMA1 channel 6. Single task consumer; no per-byte RX interrupt. */
void host_rx_dma_init(void);
bool host_rx_dma_try_read(uint8_t *byte, bool *discontinuity);
#endif
