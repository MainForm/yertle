#ifndef YERTLE_PMW_SPI_DIAGNOSTIC_H
#define YERTLE_PMW_SPI_DIAGNOSTIC_H
#include <stdint.h>
#include <stdbool.h>
/* Optional GET_UART_STATS response; no new request or periodic traffic. */
#define PMW_SPI_DIAGNOSTIC_TYPE 0x93U
#define PMW_SPI_TIMEOUT_US 2000U
enum {PMW_SPI_WAIT_TXE=1,PMW_SPI_WAIT_RXNE,PMW_SPI_WAIT_BSY,PMW_SPI_BAD_ARGUMENT};
typedef struct {uint32_t meta[14],hardware[14];} pmw_spi_failure_t;
typedef void (*pmw_spi_diag_sender_t)(void*,uint8_t,const uint8_t*,uint16_t);
void pmw3901_spi_context(uint8_t device_state,uint16_t config_index);
bool pmw3901_spi_failure_get(bool first,pmw_spi_failure_t *out);
void pmw3901_spi_send_diagnostics(void *context,pmw_spi_diag_sender_t send,uint16_t request_sequence);
#endif
