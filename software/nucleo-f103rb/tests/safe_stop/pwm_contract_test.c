#undef NDEBUG
#include "pwm_contract.h"
#include <assert.h>
#include <stdio.h>
int main(void){ypwm_mode_request r;uint8_t b[8]={2,1};ypwm_put16(b+2,YPWM_ALL_MASK);ypwm_put32(b+4,1);
 assert(ypwm_decode_mode(b,8,&r)&&r.mask==YPWM_ALL_MASK);b[0]=1;assert(!ypwm_decode_mode(b,8,&r));b[0]=2;
 ypwm_put16(b+2,3);assert(ypwm_decode_mode(b,8,&r));ypwm_put16(b+2,0);assert(!ypwm_decode_mode(b,8,&r));
 ypwm_put16(b+2,0x1000);assert(!ypwm_decode_mode(b,8,&r));ypwm_put16(b+2,YPWM_ALL_MASK);b[1]=0;assert(ypwm_decode_mode(b,8,&r));
 ypwm_put16(b+2,3);assert(!ypwm_decode_mode(b,8,&r));const uint8_t release[1]={2};assert(ypwm_decode_release(release,1));assert(!ypwm_decode_release(release,0));
 assert(ypwm_servo_word(-100)==0x0464&&ypwm_servo_word(100)==100);puts("PID schema2 rejects old direct PWM payloads; mode masks/release/BIT10 passed");return 0;}
