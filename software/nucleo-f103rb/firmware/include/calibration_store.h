#ifndef YERTLE_CALIBRATION_STORE_H
#define YERTLE_CALIBRATION_STORE_H
#include <stdbool.h>
#include <stdint.h>
#define CALIBRATION_FLASH_PAGE_BYTES 1024U
#define CALIBRATION_FLASH_BYTES 2048U
typedef struct { uint16_t valid_mask,raw[12]; } calibration_snapshot_t;
typedef enum { CALIBRATION_ABSENT,CALIBRATION_VALID,CALIBRATION_READ_ERROR } calibration_result_t;
/* Single writer: motor bus owner. No motor EEPROM writes or feedback transforms. */
calibration_result_t calibration_store_load(calibration_snapshot_t *snapshot);
bool calibration_store_save(const calibration_snapshot_t *snapshot);
void calibration_cache_init(void);
bool calibration_cache_observe(uint8_t id,uint8_t address,const uint8_t *data,uint8_t length);
bool calibration_cache_get(uint8_t id,uint16_t *raw,int16_t *steps);
bool calibration_flash_read(uint32_t offset,uint16_t *value);
bool calibration_flash_program(uint32_t offset,uint16_t value);
bool calibration_flash_erase(unsigned page);
#endif
