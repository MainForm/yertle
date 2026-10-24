#ifndef YERTLE_MODE_FLASH_H
#define YERTLE_MODE_FLASH_H
#include <stdbool.h>
#include <stdint.h>
#define MODE_FLASH_PAGE_BYTES 1024U
#define MODE_FLASH_BYTES (2U*MODE_FLASH_PAGE_BYTES)
/* Offsets are relative to the linker-reserved region, never absolute addresses. */
bool mode_flash_read(uint32_t offset,uint16_t *value);
bool mode_flash_program(uint32_t offset,uint16_t value);
bool mode_flash_erase(unsigned page);
#endif
