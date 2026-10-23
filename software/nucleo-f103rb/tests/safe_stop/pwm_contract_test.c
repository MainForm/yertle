#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include "pwm_contract.h"
int main(void){
 uint8_t m[YPWM_MODE_LENGTH]={1,1,1,0,1,2,3,4};ypwm_mode_request mode;
 assert(ypwm_decode_mode(m,sizeof m,&mode)&&mode.epoch==0x04030201UL);
 m[2]=3;assert(!ypwm_decode_mode(m,sizeof m,&mode));m[2]=1;
 m[0]=2;assert(!ypwm_decode_mode(m,sizeof m,&mode));m[0]=1;
 uint8_t p[YPWM_TARGET_LENGTH]={0};ypwm_target t;
 ypwm_put32(p,0x04030201UL);ypwm_put32(p+4,1);ypwm_put16(p+8,1);ypwm_put16(p+10,100);
 ypwm_put16(p+13,(uint16_t)(int16_t)-100);
 assert(ypwm_decode_target(p,sizeof p,&t)&&t.pwm[0]==-100);
 assert(ypwm_servo_word(-100)==0x0464&&ypwm_servo_word(100)==100&&ypwm_servo_word(0)==0);
 assert(!ypwm_decode_target(p,sizeof p-1,&t));
 ypwm_put16(p+13,101);assert(!ypwm_decode_target(p,sizeof p,&t));
 ypwm_put16(p+13,0x8000);assert(!ypwm_decode_target(p,sizeof p,&t));
 ypwm_put16(p+13,0);ypwm_put16(p+15,1);assert(!ypwm_decode_target(p,sizeof p,&t));
 ypwm_put16(p+15,0);p[12]=2;assert(!ypwm_decode_target(p,sizeof p,&t));
 p[12]=1;assert(!ypwm_decode_target(p,sizeof p,&t));ypwm_put16(p+8,0);ypwm_put16(p+10,0);
 assert(ypwm_decode_target(p,sizeof p,&t));
 puts("PASS PWM contract vectors: signed range, direction, schema, masks, release, lengths");return 0;
}
