#include "mode_store.h"
#include "mode_flash.h"
#include "pwm_contract.h"
#include <string.h>
#define RECORD_BYTES 32U
#define RECORDS_PER_PAGE (MODE_FLASH_PAGE_BYTES/RECORD_BYTES)
#define RECORD_MAGIC 0x594d4f44UL
#define RECORD_VERSION 1U
#define RECORD_COMMIT 0xa55aU
typedef struct { bool found; uint32_t sequence,offset; mode_setting_t setting; } latest_t;
static bool valid_setting(const mode_setting_t *s){return s&&s->backend<=YPWM_PID&&
    (s->backend==YPWM_PID?ypwm_mask(s->mask):s->mask==YPWM_ALL_MASK);}
static uint32_t checksum(const uint8_t *p,unsigned n){
    uint32_t crc=0xffffffffUL;
    for(unsigned i=0;i<n;++i){crc^=p[i];for(unsigned b=0;b<8;++b)crc=(crc>>1)^((crc&1U)?0xedb88320UL:0U);}
    return ~crc;
}
static bool read_record(uint32_t offset,uint8_t bytes[RECORD_BYTES]){
    for(unsigned i=0;i<RECORD_BYTES;i+=2){uint16_t word;if(!mode_flash_read(offset+i,&word))return false;ypwm_put16(bytes+i,word);}
    return true;
}
static bool erased(const uint8_t *b){for(unsigned i=0;i<RECORD_BYTES;++i)if(b[i]!=255)return false;return true;}
static bool scan(latest_t *latest){
    memset(latest,0,sizeof *latest);uint8_t b[RECORD_BYTES];
    for(uint32_t offset=0;offset<MODE_FLASH_BYTES;offset+=RECORD_BYTES){
        if(!read_record(offset,b))return false;
        if(ypwm_u16(b+30)!=RECORD_COMMIT||ypwm_u32(b)!=RECORD_MAGIC||ypwm_u16(b+4)!=RECORD_VERSION||
           ypwm_u16(b+6)>YPWM_PID||ypwm_u16(b+10)!=0||ypwm_u32(b+16)!=checksum(b,16))continue;
        const mode_setting_t setting={(uint8_t)ypwm_u16(b+6),ypwm_u16(b+8)};
        if(!valid_setting(&setting))continue;
        const uint32_t seq=ypwm_u32(b+12),delta=seq-latest->sequence;
        if(!latest->found||(delta&&delta<0x80000000UL)){
            latest->found=true;latest->sequence=seq;latest->offset=offset;latest->setting=setting;
        }
    }
    return true;
}
mode_store_result_t mode_store_read(mode_setting_t *setting){
    latest_t latest;if(!setting||!scan(&latest))return MODE_STORE_READ_ERROR;
    if(!latest.found)return MODE_STORE_ABSENT;
    *setting=latest.setting;return MODE_STORE_VALID;
}
bool mode_store_load(mode_setting_t *setting){return mode_store_read(setting)==MODE_STORE_VALID;}
bool mode_store_save(const mode_setting_t *setting){
    if(!valid_setting(setting))return false;
    latest_t latest;if(!scan(&latest))return false;
    if(latest.found&&latest.setting.backend==setting->backend&&latest.setting.mask==setting->mask)return true;
    unsigned page=latest.found?(unsigned)(latest.offset/MODE_FLASH_PAGE_BYTES):0U;
    uint32_t offset=MODE_FLASH_BYTES;uint8_t b[RECORD_BYTES];
    for(unsigned i=0;i<RECORDS_PER_PAGE;++i){
        const uint32_t candidate=page*MODE_FLASH_PAGE_BYTES+i*RECORD_BYTES;
        if(!read_record(candidate,b))return false;
        if(erased(b)){offset=candidate;break;}
    }
    if(offset==MODE_FLASH_BYTES){
        /* Never erase the page holding the newest valid record. */
        if(latest.found)page=1U-page;
        if(!mode_flash_erase(page))return false;
        offset=page*MODE_FLASH_PAGE_BYTES;
        for(unsigned i=0;i<RECORDS_PER_PAGE;++i)
            if(!read_record(offset+i*RECORD_BYTES,b)||!erased(b))return false;
    }
    memset(b,255,sizeof b);ypwm_put32(b,RECORD_MAGIC);ypwm_put16(b+4,RECORD_VERSION);
    ypwm_put16(b+6,setting->backend);ypwm_put16(b+8,setting->mask);ypwm_put16(b+10,0);
    ypwm_put32(b+12,latest.found?latest.sequence+1U:1U);ypwm_put32(b+16,checksum(b,16));
    for(unsigned i=0;i<20;i+=2){uint16_t verify;
        if(!mode_flash_program(offset+i,ypwm_u16(b+i))||!mode_flash_read(offset+i,&verify)||verify!=ypwm_u16(b+i))return false;
    }
    /* Last halfword is the commit marker: torn records never supersede old data. */
    if(!mode_flash_program(offset+30,RECORD_COMMIT))return false;
    latest_t check;
    return scan(&check)&&check.found&&check.offset==offset&&check.setting.backend==setting->backend&&check.setting.mask==setting->mask;
}
