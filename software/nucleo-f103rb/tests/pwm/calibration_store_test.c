#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "calibration_store.h"
extern uint8_t calibration_memory[];
extern int calibration_budget;
extern unsigned calibration_programs,calibration_erases;
extern bool calibration_read_ok,calibration_allowed;
void calibration_memory_reset(void);
int main(void){
 calibration_snapshot_t s={1,{3011}},r={0};uint16_t raw;int16_t steps;
 calibration_memory_reset();calibration_cache_init();assert(!calibration_cache_get(1,&raw,&steps));
 assert(calibration_store_load(&r)==CALIBRATION_ABSENT);
 assert(calibration_store_save(&s));unsigned writes=calibration_programs;
 assert(calibration_store_save(&s)&&writes==calibration_programs);
 calibration_cache_init();assert(calibration_cache_get(1,&raw,&steps)&&raw==3011&&steps==-963);
 uint8_t b[]={0x30,0x05};assert(calibration_cache_observe(2,31,b,1));
 assert(!calibration_cache_get(2,&raw,&steps));assert(calibration_cache_observe(2,32,b+1,1));
 assert(calibration_cache_get(2,&raw,&steps)&&raw==1328&&steps==1328);
 uint8_t block[32]={0};block[31]=0x98;assert(calibration_cache_observe(3,0,block,32));
 block[0]=5;assert(calibration_cache_observe(3,32,block,1));
 calibration_cache_init();assert(calibration_cache_get(3,&raw,&steps)&&raw==1432&&steps==1432);
 uint8_t invalid[]={0xff,0xff};assert(!calibration_cache_observe(1,31,invalid,2));
 assert(calibration_cache_get(1,&raw,&steps)&&raw==3011);
 /* Power cut at EVERY body/commit programming boundary. */
 uint8_t backup[2048];memcpy(backup,calibration_memory,sizeof backup);
 assert(calibration_store_load(&r)==CALIBRATION_VALID);s=r;s.raw[0]=100;
 for(int cut=0;cut<=21;++cut){calibration_memory_reset();memcpy(calibration_memory,backup,sizeof backup);calibration_budget=cut;
  bool ok=calibration_store_save(&s);calibration_budget=-1;assert(calibration_store_load(&r)==CALIBRATION_VALID);
  assert(ok?(r.raw[0]==100):(r.raw[0]==3011));}
 /* CRC corruption rejects the newest record, previous snapshot survives. */
 calibration_memory_reset();s=(calibration_snapshot_t){1,{20}};assert(calibration_store_save(&s));s.raw[0]=21;assert(calibration_store_save(&s));
 calibration_memory[64+12]^=1;assert(calibration_store_load(&r)==CALIBRATION_VALID&&r.raw[0]==20);
 /* Rotate both pages; old page survives interrupted erase/program. */
 calibration_memory_reset();for(unsigned i=0;i<16;++i){s.raw[0]=(uint16_t)i;assert(calibration_store_save(&s));}
 memcpy(backup,calibration_memory,sizeof backup);s.raw[0]=999;
 for(int cut=0;cut<=22;++cut){calibration_memory_reset();memcpy(calibration_memory,backup,sizeof backup);calibration_budget=cut;
  bool ok=calibration_store_save(&s);calibration_budget=-1;assert(!memcmp(backup,calibration_memory,1024));
  assert(calibration_store_load(&r)==CALIBRATION_VALID);assert(ok?r.raw[0]==999:r.raw[0]==15);}
 calibration_memory_reset();for(unsigned i=0;i<80;++i){s.raw[0]=(uint16_t)i;assert(calibration_store_save(&s));}
 assert(calibration_erases==4);calibration_read_ok=false;assert(calibration_store_load(&r)==CALIBRATION_READ_ERROR&&!calibration_store_save(&s));
 calibration_read_ok=true;calibration_allowed=false;s.raw[0]=90;assert(!calibration_store_save(&s));
 puts("Calibration journal/cache: boot, split/block reads, signed offsets, no-op writes, power cuts, CRC, rotation and storage errors passed");return 0;
}
