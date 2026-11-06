#ifndef YERTLE_PMW3901_SPI_H
#define YERTLE_PMW3901_SPI_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
void pmw3901_spi_init(void);
void pmw3901_spi_reset(bool asserted);
void pmw3901_spi_select(bool selected);
bool pmw3901_spi_read(uint8_t address, uint8_t *data, size_t length, bool burst);
bool pmw3901_spi_write(uint8_t address, uint8_t value);
#endif
