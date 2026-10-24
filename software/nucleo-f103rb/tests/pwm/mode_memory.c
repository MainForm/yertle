#include "mode_memory.h"
#include <string.h>
uint8_t mode_memory_bytes[MODE_FLASH_BYTES];
int mode_memory_budget=-1;
bool mode_memory_read_ok=true,mode_memory_partial_erase;
unsigned mode_memory_programs,mode_memory_erases;
void mode_memory_reset(void){memset(mode_memory_bytes,255,sizeof mode_memory_bytes);mode_memory_budget=-1;
 mode_memory_read_ok=true;mode_memory_partial_erase=false;mode_memory_programs=mode_memory_erases=0;}
static bool allowed(void){if(!mode_memory_budget)return false;if(mode_memory_budget>0)--mode_memory_budget;return true;}
bool mode_flash_read(uint32_t offset,uint16_t *value){
 if(!value||!mode_memory_read_ok||(offset&1U)||offset>=MODE_FLASH_BYTES)return false;
 *value=(uint16_t)(mode_memory_bytes[offset]|((uint16_t)mode_memory_bytes[offset+1]<<8));return true;
}
bool mode_flash_program(uint32_t offset,uint16_t value){
 uint16_t old;if(!mode_flash_read(offset,&old)||old!=0xffff||!allowed())return false;
 mode_memory_bytes[offset]=(uint8_t)value;mode_memory_bytes[offset+1]=(uint8_t)(value>>8);++mode_memory_programs;return true;
}
bool mode_flash_erase(unsigned page){
 if(page>=2||!allowed())return false;
 ++mode_memory_erases;
 memset(mode_memory_bytes+page*MODE_FLASH_PAGE_BYTES,255,mode_memory_partial_erase?MODE_FLASH_PAGE_BYTES/2:MODE_FLASH_PAGE_BYTES);
 return !mode_memory_partial_erase;
}
