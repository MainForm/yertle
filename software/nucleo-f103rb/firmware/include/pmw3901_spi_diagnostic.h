#ifndef YERTLE_PMW_SPI_DIAGNOSTIC_H
#define YERTLE_PMW_SPI_DIAGNOSTIC_H
#include <stdint.h>
#include <stdbool.h>
#define PMW_SPI_DIAGNOSTIC_TYPE 0x93U
#define PMW_SPI_STAGE_TIMEOUT_US 500U
enum {PMW_SPI_WAIT_TXE=1,PMW_SPI_WAIT_RXNE,PMW_SPI_WAIT_BSY,
      PMW_SPI_BAD_ARGUMENT,PMW_SPI_CALL_BUDGET,PMW_SPI_SENSOR_ID};
typedef struct {uint32_t meta[14],hardware[14];} pmw_spi_failure_t;
typedef void (*pmw_spi_diag_sender_t)(void*,uint8_t,const uint8_t*,uint16_t);
/* Sensor task supplies its cooperative call budget; no asynchronous kill. */
void pmw3901_spi_call_begin(uint32_t timeout_us);
bool pmw3901_spi_call_end(void);
void pmw3901_spi_context(uint8_t device_state,uint16_t config_index);
void pmw3901_spi_id_error(uint8_t expected);
bool pmw3901_spi_failure_get(bool first,pmw_spi_failure_t *out);
void pmw3901_spi_send_diagnostics(void *context,pmw_spi_diag_sender_t send,uint16_t request_sequence);
/* Host task sends at most one queued half per pass. False TX retains it. */
bool pmw3901_spi_auto_payload(uint8_t payload[64]);
void pmw3901_spi_auto_result(bool sent);
#endif
