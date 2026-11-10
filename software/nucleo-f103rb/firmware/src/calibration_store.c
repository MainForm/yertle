#include "calibration_store.h"
#include "pwm_contract.h"
#include <string.h>
#define RECORD_BYTES 64U
#define RECORDS_PER_PAGE (CALIBRATION_FLASH_PAGE_BYTES/RECORD_BYTES)
#define RECORD_MAGIC 0x5943414cUL
#define RECORD_VERSION 1U
#define RECORD_COMMIT 0xa55aU
typedef struct { bool found; uint32_t sequence,offset; calibration_snapshot_t setting; } latest_t;
static bool valid_setting(const calibration_snapshot_t *s){
    if(!s||(s->valid_mask&~0x0fffU))return false;
    for(unsigned i=0;i<12;++i)if((s->valid_mask&(1U<<i))&&s->raw[i]>4095U)return false;
    return true;
}
static bool same(const calibration_snapshot_t *a,const calibration_snapshot_t *b){
    if(a->valid_mask!=b->valid_mask)return false;
    for(unsigned i=0;i<12;++i)if((a->valid_mask&(1U<<i))&&a->raw[i]!=b->raw[i])return false;
    return true;
}
static uint32_t checksum(const uint8_t *p,unsigned n){
    uint32_t crc=0xffffffffUL;
    for(unsigned i=0;i<n;++i){crc^=p[i];for(unsigned b=0;b<8;++b)crc=(crc>>1)^((crc&1U)?0xedb88320UL:0U);}
    return ~crc;
}
static bool read_record(uint32_t offset,uint8_t bytes[RECORD_BYTES]){
    for(unsigned i=0;i<RECORD_BYTES;i+=2){uint16_t word;if(!calibration_flash_read(offset+i,&word))return false;ypwm_put16(bytes+i,word);}
    return true;
}
static bool erased(const uint8_t *b){for(unsigned i=0;i<RECORD_BYTES;++i)if(b[i]!=255)return false;return true;}
static bool calibration_scan(latest_t *latest){
    memset(latest,0,sizeof *latest);uint8_t b[RECORD_BYTES];
    for(uint32_t offset=0;offset<CALIBRATION_FLASH_BYTES;offset+=RECORD_BYTES){
        if(!read_record(offset,b))return false;
        if(ypwm_u16(b+62)!=RECORD_COMMIT||ypwm_u32(b)!=RECORD_MAGIC||ypwm_u16(b+4)!=RECORD_VERSION||
           ypwm_u32(b+36)!=checksum(b,36))continue;
        calibration_snapshot_t setting={0};setting.valid_mask=ypwm_u16(b+6);
        for(unsigned i=0;i<12;++i)setting.raw[i]=ypwm_u16(b+12+2*i);
        if(!valid_setting(&setting))continue;
        const uint32_t seq=ypwm_u32(b+8),delta=seq-latest->sequence;
        if(!latest->found||(delta&&delta<0x80000000UL)){
            latest->found=true;latest->sequence=seq;latest->offset=offset;latest->setting=setting;
        }
    }
    return true;
}
calibration_result_t calibration_store_load(calibration_snapshot_t *setting){
    latest_t latest;if(!setting||!calibration_scan(&latest))return CALIBRATION_READ_ERROR;
    if(!latest.found)return CALIBRATION_ABSENT;
    *setting=latest.setting;return CALIBRATION_VALID;
}
bool calibration_store_save(const calibration_snapshot_t *setting){
    if(!valid_setting(setting))return false;
    latest_t latest;if(!calibration_scan(&latest))return false;
    if(latest.found&&same(&latest.setting,setting))return true;
    unsigned page=latest.found?(unsigned)(latest.offset/CALIBRATION_FLASH_PAGE_BYTES):0U;
    uint32_t offset=CALIBRATION_FLASH_BYTES;uint8_t b[RECORD_BYTES];
    for(unsigned i=0;i<RECORDS_PER_PAGE;++i){
        const uint32_t candidate=page*CALIBRATION_FLASH_PAGE_BYTES+i*RECORD_BYTES;
        if(!read_record(candidate,b))return false;
        if(erased(b)){offset=candidate;break;}
    }
    if(offset==CALIBRATION_FLASH_BYTES){
        /* Never erase the page holding the newest valid record. */
        if(latest.found)page=1U-page;
        if(!calibration_flash_erase(page))return false;
        offset=page*CALIBRATION_FLASH_PAGE_BYTES;
        for(unsigned i=0;i<RECORDS_PER_PAGE;++i)
            if(!read_record(offset+i*RECORD_BYTES,b)||!erased(b))return false;
    }
    memset(b,255,sizeof b);ypwm_put32(b,RECORD_MAGIC);ypwm_put16(b+4,RECORD_VERSION);
    ypwm_put16(b+6,setting->valid_mask);ypwm_put32(b+8,latest.found?latest.sequence+1U:1U);
    for(unsigned i=0;i<12;++i)ypwm_put16(b+12+2*i,setting->raw[i]);
    ypwm_put32(b+36,checksum(b,36));
    for(unsigned i=0;i<40;i+=2){uint16_t verify;
        if(!calibration_flash_program(offset+i,ypwm_u16(b+i))||!calibration_flash_read(offset+i,&verify)||verify!=ypwm_u16(b+i))return false;
    }
    /* Last halfword is the commit marker: torn records never supersede old data. */
    if(!calibration_flash_program(offset+62,RECORD_COMMIT))return false;
    latest_t check;
    return calibration_scan(&check)&&check.found&&check.offset==offset&&same(&check.setting,setting);
}

static calibration_snapshot_t cached;
static uint8_t pending[12][2],pending_mask[12];
void calibration_cache_init(void){
    memset(&cached,0,sizeof cached);memset(pending_mask,0,sizeof pending_mask);
    if(calibration_store_load(&cached)!=CALIBRATION_VALID)memset(&cached,0,sizeof cached);
}
bool calibration_cache_invalidate(uint8_t id){
    if(id<1||id>12)return false;
    const unsigned index=id-1U;
    calibration_snapshot_t updated=cached;
    updated.valid_mask&=(uint16_t)~(1U<<index);
    if(!calibration_store_save(&updated))return false;
    cached=updated;
    pending_mask[index]=0U;
    return true;
}
bool calibration_cache_get(uint8_t id,uint16_t *raw,int16_t *steps){
    if(id<1||id>12||!raw||!steps||!(cached.valid_mask&(1U<<(id-1U))))return false;
    *raw=cached.raw[id-1U];const int16_t magnitude=(int16_t)(*raw&0x7ffU);
    *steps=(*raw&0x800U)?(int16_t)-magnitude:magnitude;return true;
}
bool calibration_cache_observe(uint8_t id,uint8_t address,const uint8_t *data,uint8_t length){
    if(id<1||id>12||!data)return false;
    if((unsigned)address+length<=31U||address>32U)return true;
    const unsigned index=id-1U;
    for(unsigned i=0;i<length;++i)if(address+i==31U||address+i==32U){
        unsigned part=address+i-31U;pending[index][part]=data[i];pending_mask[index]|=(uint8_t)(1U<<part);
    }
    if(pending_mask[index]!=3U)return true;
    pending_mask[index]=0;
    const uint16_t raw=ypwm_u16(pending[index]);
    /* Match the currently supported servo revision/sign-magnitude contract. */
    if(raw>4095U)return false;
    calibration_snapshot_t updated=cached;
    updated.raw[index]=raw;updated.valid_mask|=(uint16_t)(1U<<index);
    if(!calibration_store_save(&updated))return false;
    cached=updated;
    return true;
}

bool calibration_cache_position(uint8_t id,int16_t wire_position,int16_t *position){
    uint16_t raw;int16_t offset;
    if(!position||wire_position<0||wire_position>4095||!calibration_cache_get(id,&raw,&offset))return false;
    int32_t value=((int32_t)wire_position-offset)%4096;
    if(value<0)value+=4096;
    *position=(int16_t)value;return true;
}
