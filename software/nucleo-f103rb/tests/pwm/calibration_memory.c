#include "calibration_store.h"
#include <string.h>
uint8_t calibration_memory[CALIBRATION_FLASH_BYTES];
int calibration_budget=-1;unsigned calibration_programs,calibration_erases;
bool calibration_read_ok=true,calibration_allowed=true;
void calibration_memory_reset(void){memset(calibration_memory,255,sizeof calibration_memory);
 calibration_budget=-1;calibration_programs=calibration_erases=0;calibration_read_ok=calibration_allowed=true;}
static bool allowed(void){if(!calibration_allowed||!calibration_budget)return false;
 if(calibration_budget>0)--calibration_budget;return true;}
bool calibration_flash_read(uint32_t offset,uint16_t *value){
 if(!calibration_read_ok||!value||(offset&1U)||offset>=sizeof calibration_memory)return false;
 *value=(uint16_t)(calibration_memory[offset]|(uint16_t)calibration_memory[offset+1]<<8);return true;}
bool calibration_flash_program(uint32_t offset,uint16_t value){uint16_t old;
 if(!calibration_flash_read(offset,&old)||old!=65535||!allowed())return false;
 calibration_memory[offset]=(uint8_t)value;calibration_memory[offset+1]=(uint8_t)(value>>8);++calibration_programs;return true;}
bool calibration_flash_erase(unsigned page){if(page>=2||!allowed())return false;
 memset(calibration_memory+page*1024,255,1024);++calibration_erases;return true;}
