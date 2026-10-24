#ifndef TEST_MODE_MEMORY_H
#define TEST_MODE_MEMORY_H
#include "mode_flash.h"
extern uint8_t mode_memory_bytes[MODE_FLASH_BYTES];
extern int mode_memory_budget;
extern bool mode_memory_read_ok,mode_memory_partial_erase;
extern unsigned mode_memory_programs,mode_memory_erases;
void mode_memory_reset(void);
#endif
