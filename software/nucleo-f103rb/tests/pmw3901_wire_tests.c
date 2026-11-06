#ifdef NDEBUG
#undef NDEBUG
#endif
#include "optical_flow_wire.h"
#include <assert.h>
#include <stdio.h>
int main(void){
 pmw3901_feedback_t f={0};uint8_t p[OPTICAL_FLOW_PAYLOAD_SIZE];
 f.valid=f.ready=f.interval_valid=true;f.sequence=3;f.uptime_ms=135;f.interval_ms=20;f.epoch=0x1234;f.state=PMW_READY;
 f.raw[2]=0xff;f.raw[3]=0xff;f.raw[4]=0;f.raw[5]=0x80;f.raw[6]=44;f.raw[10]=0x12;f.raw[11]=0x34;
 optical_encode(p,0x4321,7,140,&f);
 assert(sizeof p==44&&p[0]==0x21&&p[1]==0x43&&p[2]==1&&p[3]==7);
 assert(p[4]==7&&p[8]==140&&p[12]==3&&p[16]==135&&p[20]==20);
 assert(p[26]==0xff&&p[27]==0xff&&p[28]==0&&p[29]==0x80&&p[30]==44);
 assert(p[34]==0x12&&p[35]==0x34&&p[40]==0x34&&p[41]==0x12&&p[42]==PMW_READY);
 f.valid=false;f.interval_valid=false;optical_encode(p,0,8,160,&f);assert(p[3]==2);
 puts("PASS firmware optical wire offsets, parent time, validity, raw endian and epoch");
}
