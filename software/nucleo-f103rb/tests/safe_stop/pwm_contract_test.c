#undef NDEBUG
#include "pwm_contract.h"
#include <assert.h>
#include <stdio.h>
int main(void){ypwm_mode_request r;uint8_t b[9]={YPWM_SCHEMA,YPWM_PREPARE,YPWM_PID};
 ypwm_put16(b+3,YPWM_ALL_MASK);ypwm_put32(b+5,1);
 assert(ypwm_decode_mode(b,9,&r)&&r.mask==YPWM_ALL_MASK&&r.operation==YPWM_PREPARE);
 b[0]=2;assert(!ypwm_decode_mode(b,9,&r));b[0]=YPWM_SCHEMA;
 b[1]=YPWM_COMMIT;assert(ypwm_decode_mode(b,9,&r));b[1]=YPWM_CANCEL;assert(ypwm_decode_mode(b,9,&r));
 b[1]=3;assert(!ypwm_decode_mode(b,9,&r));b[1]=0;
 ypwm_put16(b+3,0);assert(!ypwm_decode_mode(b,9,&r));ypwm_put16(b+3,0x1000);assert(!ypwm_decode_mode(b,9,&r));
 ypwm_put16(b+3,3);assert(ypwm_decode_mode(b,9,&r));b[2]=YPWM_POSITION;assert(!ypwm_decode_mode(b,9,&r));
 ypwm_put16(b+3,YPWM_ALL_MASK);assert(ypwm_decode_mode(b,9,&r));assert(!ypwm_decode_mode(b,8,&r));
 assert(YPWM_DUTY_LIMIT==1000&&ypwm_servo_word(-1000)==0x07e8&&ypwm_servo_word(1000)==0x03e8);
 assert(ypwm_servo_word(-100)==0x0464&&ypwm_servo_word(100)==100);
 puts("schema3 PREPARE/COMMIT/CANCEL masks; retired operation3 rejected, old peer rejection, PWM encoding passed");return 0;
}
