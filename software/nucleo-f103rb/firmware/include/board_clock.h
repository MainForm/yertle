#ifndef BOARD_CLOCK_H
#define BOARD_CLOCK_H

#include <stdint.h>

/* HSI/2 * 16 = 64 MHz; APB1 = 32 MHz, APB2 = 64 MHz. */
void board_clock_init(void);
uint32_t board_apb1_clock_hz(void);
uint32_t board_apb2_clock_hz(void);

#endif /* BOARD_CLOCK_H */
