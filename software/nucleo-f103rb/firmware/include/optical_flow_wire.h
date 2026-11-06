#ifndef YERTLE_OPTICAL_FLOW_WIRE_H
#define YERTLE_OPTICAL_FLOW_WIRE_H
#include "pmw3901_device.h"
#include <stddef.h>
#define OPTICAL_FLOW_MESSAGE_TYPE 0x92U
#define OPTICAL_FLOW_PAYLOAD_SIZE 44U
static inline void optical_put32(uint8_t*p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
static inline void optical_encode(uint8_t p[OPTICAL_FLOW_PAYLOAD_SIZE],uint16_t request,
    uint32_t snapshot,uint32_t snapshot_ms,const pmw3901_feedback_t*f){
    p[0]=(uint8_t)request;p[1]=(uint8_t)(request>>8);p[2]=1;
    p[3]=(f->valid?1U:0U)|(f->ready?2U:0U)|(f->interval_valid?4U:0U);
    optical_put32(p+4,snapshot);optical_put32(p+8,snapshot_ms);
    optical_put32(p+12,f->sequence);optical_put32(p+16,f->uptime_ms);optical_put32(p+20,f->interval_ms);
    for(unsigned i=0;i<12;i++)p[24+i]=f->raw[i];
    optical_put32(p+36,f->errors);p[40]=(uint8_t)f->epoch;p[41]=(uint8_t)(f->epoch>>8);
    p[42]=f->state;p[43]=f->last_error;
}
#endif
