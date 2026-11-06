#include "pmw3901_device.h"
#include "pmw3901_spi.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static unsigned calls;static bool bad_id,fail_spi;
void pmw3901_spi_init(void){}
void pmw3901_spi_reset(bool b){(void)b;}
void pmw3901_spi_select(bool b){(void)b;}
bool pmw3901_spi_write(uint8_t a,uint8_t v){(void)a;(void)v;++calls;return !fail_spi;}
bool pmw3901_spi_read(uint8_t a,uint8_t*d,size_t n,bool burst){
 ++calls;if(fail_spi)return false;memset(d,0,n);
 if(a==0)*d=bad_id?0:0x49;if(a==0x5f)*d=0xb6;
 if(burst){d[0]=0x80;d[2]=0xff;d[3]=0xff;d[6]=40;}return true;
}
int main(void){pmw3901_device_t d;pmw3901_device_init(&d);uint32_t now;
 for(now=0;now<5000&&!d.feedback.valid;now+=20){calls=0;pmw3901_device_service(&d,now);assert(calls<=1);}
 assert(d.feedback.ready&&d.feedback.valid&&d.feedback.interval_ms>=20&&d.feedback.sequence==2);
 unsigned seq=d.feedback.sequence;fail_spi=true;pmw3901_device_service(&d,now);
 assert(!d.feedback.valid&&d.feedback.errors==1&&d.feedback.sequence==seq);
 fail_spi=false;bad_id=true;for(unsigned i=0;i<80;i++){now+=20;calls=0;pmw3901_device_service(&d,now);assert(calls<=1);}
 assert(!d.feedback.ready&&d.feedback.errors>=2);
 bad_id=false;for(unsigned i=0;i<250&&!d.feedback.valid;i++){now+=20;pmw3901_device_service(&d,now);}
 assert(d.feedback.valid&&d.feedback.epoch>=3&&d.feedback.sequence>seq);
 puts("PASS: bounded step initialization, invalid first interval, SPI failure, absent ID, recovery epoch");}
