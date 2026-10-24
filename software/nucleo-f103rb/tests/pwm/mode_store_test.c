#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "mode_store.h"
#include "mode_memory.h"
#include "pwm_contract.h"
static void expect(uint8_t backend,uint16_t mask){mode_setting_t s={255,0};assert(mode_store_load(&s)&&s.backend==backend&&s.mask==mask);}
static uint32_t crc(const uint8_t *b){uint32_t c=0xffffffffU;for(unsigned i=0;i<16;++i){c^=b[i];for(unsigned k=0;k<8;++k)c=(c>>1)^((c&1)?0xedb88320U:0);}return ~c;}
int main(void){
 mode_setting_t s={YPWM_PID,8},read={0};mode_memory_reset();assert(!mode_store_load(&read));
 assert(!mode_store_save(NULL));s.mask=0;assert(!mode_store_save(&s));s.mask=0x1000;assert(!mode_store_save(&s));
 s.backend=2;s.mask=8;assert(!mode_store_save(&s));s.backend=YPWM_POSITION;assert(!mode_store_save(&s));
 s.mask=YPWM_ALL_MASK;assert(mode_store_save(&s));expect(YPWM_POSITION,YPWM_ALL_MASK);
 const unsigned writes=mode_memory_programs;assert(mode_store_save(&s)&&mode_memory_programs==writes&&!mode_memory_erases);
 s.backend=YPWM_PID;s.mask=8;assert(mode_store_save(&s));expect(YPWM_PID,8);
 /* Every body/CRC byte and the commit marker is independently protected. */
 uint8_t saved[MODE_FLASH_BYTES];memcpy(saved,mode_memory_bytes,sizeof saved);
 for(unsigned i=0;i<32;++i)if(i<20||i>=30){memcpy(mode_memory_bytes,saved,sizeof saved);mode_memory_bytes[32+i]^=1;
  expect(YPWM_POSITION,YPWM_ALL_MASK);}
 for(unsigned field=4;field<=6;field+=2){memcpy(mode_memory_bytes,saved,sizeof saved);ypwm_put16(mode_memory_bytes+32+field,2);
  ypwm_put32(mode_memory_bytes+32+16,crc(mode_memory_bytes+32));expect(YPWM_POSITION,YPWM_ALL_MASK);}
 memcpy(mode_memory_bytes,saved,sizeof saved);mode_memory_bytes[32+8]=0;mode_memory_bytes[32+9]=0;
 ypwm_put32(mode_memory_bytes+32+16,crc(mode_memory_bytes+32));expect(YPWM_POSITION,YPWM_ALL_MASK);
 /* Torn append: old record survives every incomplete halfword sequence. */
 for(int cut=0;cut<=11;++cut){mode_memory_reset();mode_memory_budget=cut;s=(mode_setting_t){YPWM_PID,4};
  const bool ok=mode_store_save(&s);if(cut<11)assert(!ok&&mode_store_read(&read)==MODE_STORE_ABSENT);else assert(ok&&mode_store_load(&read));}
 for(int cut=0;cut<=11;++cut){mode_memory_reset();s=(mode_setting_t){YPWM_POSITION,YPWM_ALL_MASK};assert(mode_store_save(&s));
  mode_memory_budget=cut;s=(mode_setting_t){YPWM_PID,4};const bool ok=mode_store_save(&s);
  if(cut<11){assert(!ok);expect(YPWM_POSITION,YPWM_ALL_MASK);}else{assert(ok);expect(YPWM_PID,4);}}
 /* Fill a page, then interrupt erase/append on the OTHER page. */
 mode_memory_reset();for(unsigned i=0;i<32;++i){s=(mode_setting_t){YPWM_PID,(uint16_t)(1U<<(i%12))};assert(mode_store_save(&s));}
 memcpy(saved,mode_memory_bytes,sizeof saved);const uint16_t oldmask=s.mask;
 for(int cut=0;cut<=12;++cut){mode_memory_reset();memcpy(mode_memory_bytes,saved,sizeof saved);mode_memory_budget=cut;
  s=(mode_setting_t){YPWM_POSITION,YPWM_ALL_MASK};const bool ok=mode_store_save(&s);
  assert(!memcmp(mode_memory_bytes,saved,MODE_FLASH_PAGE_BYTES));
  if(cut<12){assert(!ok);expect(YPWM_PID,oldmask);}else{assert(ok);expect(YPWM_POSITION,YPWM_ALL_MASK);}}
 mode_memory_reset();memcpy(mode_memory_bytes,saved,sizeof saved);mode_memory_partial_erase=true;
 assert(!mode_store_save(&s));expect(YPWM_PID,oldmask);
 /* Exercise repeated rotation and wear avoidance. */
 mode_memory_reset();for(unsigned i=0;i<160;++i){s=(mode_setting_t){YPWM_PID,(uint16_t)(1U<<(i%12))};assert(mode_store_save(&s));expect(s.backend,s.mask);}
 assert(mode_memory_erases==4);
 /* Sequence wrap must prefer the next committed record, not the larger integer. */
 mode_memory_reset();s=(mode_setting_t){YPWM_PID,1};assert(mode_store_save(&s));ypwm_put32(mode_memory_bytes+12,0xfffffffeU);
 ypwm_put32(mode_memory_bytes+16,crc(mode_memory_bytes));s.mask=2;assert(mode_store_save(&s));expect(YPWM_PID,2);
 s.mask=4;assert(mode_store_save(&s));expect(YPWM_PID,4);
 mode_memory_read_ok=false;assert(mode_store_read(&read)==MODE_STORE_READ_ERROR&&!mode_store_load(&read)&&!mode_store_save(&s));
 puts("Mode journal: CRC/version/mask, no-op writes, torn records/erase, page rotation and sequence wrap passed");return 0;
}
