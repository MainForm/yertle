#include "pmw3901_device.h"
#include "pmw3901_spi.h"
#include "pmw3901_spi_diagnostic.h"
#include <string.h>
#include "pmw3901_init_sequence.h"
static void fail(pmw3901_device_t*d,uint32_t now,uint8_t error){
    ++d->feedback.errors;d->feedback.last_error=error;d->feedback.valid=false;
    d->feedback.ready=false;d->feedback.interval_valid=false;d->previous_read=false;
    d->state=PMW_RETRY;d->deadline=now+1000U;
}
void pmw3901_device_init(pmw3901_device_t*d){memset(d,0,sizeof *d);pmw3901_spi_init();}
void pmw3901_device_service(pmw3901_device_t*d,uint32_t now){
    if((int32_t)(now-d->deadline)<0)return;
    pmw3901_spi_context(d->state,d->config_index);
    uint8_t byte=0;
    switch(d->state){
    case PMW_START: case PMW_RETRY:
        d->feedback.valid=false;d->feedback.ready=false;d->feedback.interval_valid=false;
        d->previous_read=false;pmw3901_spi_reset(true);++d->feedback.epoch;
        d->state=PMW_RELEASE_RESET;d->deadline=now+2U;break;
    case PMW_RELEASE_RESET:pmw3901_spi_reset(false);pmw3901_spi_select(false);
        d->state=PMW_CS_LOW;d->deadline=now+40U;break;
    case PMW_CS_LOW:pmw3901_spi_select(true);d->state=PMW_CS_HIGH;d->deadline=now+2U;break;
    case PMW_CS_HIGH:pmw3901_spi_select(false);d->state=PMW_ID;d->deadline=now+2U;d->clear_address=0;break;
    case PMW_ID:
        if(!pmw3901_spi_read(d->clear_address?0x5f:0,&byte,1,false))fail(d,now,PMW_ERROR_SPI);
        else if(byte!=(d->clear_address?0xb6:0x49))fail(d,now,PMW_ERROR_ID);
        else if(d->clear_address)d->state=PMW_RESET;else d->clear_address=1;
        break;
    case PMW_RESET:
        if(!pmw3901_spi_write(0x3a,0x5a))fail(d,now,PMW_ERROR_SPI);
        else{d->state=PMW_CLEAR;d->clear_address=2;d->deadline=now+5U;}break;
    case PMW_CLEAR:
        if(!pmw3901_spi_read(d->clear_address,&byte,1,false))fail(d,now,PMW_ERROR_SPI);
        else if(++d->clear_address==7){d->state=PMW_CONFIG;d->config_index=0;d->deadline=now+1U;}
        break;
    case PMW_CONFIG:{
        const uint8_t *pair=pmw3901_init_sequence[d->config_index];
        if(pair[0]==0xff){++d->config_index;d->deadline=now+pair[1];}
        else if(!pmw3901_spi_write(pair[0],pair[1]))fail(d,now,PMW_ERROR_SPI);
        else if(++d->config_index==sizeof(pmw3901_init_sequence)/sizeof(pmw3901_init_sequence[0]))d->state=PMW_DISCARD;
        break;}
    case PMW_DISCARD:case PMW_READY:
        if(d->state==PMW_READY&&(uint32_t)(now-d->last_health_ms)>=1000U){
            d->last_health_ms=now;
            if(!pmw3901_spi_read(0,&byte,1,false))fail(d,now,PMW_ERROR_SPI);
            else if(byte!=0x49)fail(d,now,PMW_ERROR_ID);
            break;
        }
        if(!pmw3901_spi_read(0x16,d->feedback.raw,12,true))fail(d,now,PMW_ERROR_SPI);
        else{
            ++d->feedback.sequence;d->feedback.uptime_ms=now;
            d->feedback.interval_ms=d->previous_read?now-d->previous_read_ms:0;
            d->feedback.interval_valid=d->previous_read&&d->feedback.interval_ms>0&&d->feedback.interval_ms<=100;
            d->feedback.valid=d->feedback.interval_valid;d->feedback.ready=true;d->feedback.last_error=0;
            d->previous_read_ms=now;d->previous_read=true;d->state=PMW_READY;
        }break;
    default:fail(d,now,PMW_ERROR_SPI);break;
    }
    d->feedback.state=d->state;
}
