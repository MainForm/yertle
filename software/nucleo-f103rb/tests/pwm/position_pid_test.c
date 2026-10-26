#undef NDEBUG
#include "position_pid.h"
#include "pwm_contract.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void motor_polarity_tests(void){
 const int8_t expected[12]={-1,-1,1,1,1,-1,1,1,1,-1,-1,-1};
 assert(position_pid_motor_polarity(0)==0&&position_pid_motor_polarity(13)==0&&position_pid_motor_polarity(255)==0);
 for(uint8_t id=1;id<=12;++id)for(int sign=-1;sign<=1;sign+=2){
  position_pid_config_t config=position_pid_default_config();position_pid_t pid={0};int16_t out=0;
  config.polarity=position_pid_motor_polarity(id);assert(config.polarity==expected[id-1]);
  assert(position_pid_step(&pid,&config,(uint16_t)(2048+sign*100),30000,0,2048,.02f,&out));
  assert(pid.trace_error==sign*100&&pid.trace_p==sign*200&&pid.trace_d==0);
  assert(out*sign*expected[id-1]>0); /* signed error retained, only final output reverses */
  assert(((pid.trace_flags&32)!=0)==(expected[id-1]<0));
  position_pid_reset(&pid,2048);
  assert(position_pid_step(&pid,&config,2048,200,0,2048,.02f,&out)&&out==0);
 }
}
int main(void){
 motor_polarity_tests();
 position_pid_config_t c=position_pid_default_config();position_pid_t p={0};int16_t out;
 assert(YPWM_DUTY_LIMIT==800&&c.duty_limit==800);
 assert(position_pid_step(&p,&c,2148,200,0,2048,.02f,&out));assert(out==8&&p.velocity==0&&p.reference<=2052.01f);
 const float previous=p.reference;assert(position_pid_step(&p,&c,1948,200,0,2048,.02f,&out));assert(p.reference<previous&&p.velocity==0);
 p=(position_pid_t){0};c.kp=100;c.ki=20;
 assert(position_pid_step(&p,&c,2148,30000,0,2048,.02f,&out));assert(out==800&&(p.trace_flags&2)&&!(p.trace_flags&4)); /* no ramp from zero */
 for(unsigned i=0;i<100;++i){assert(position_pid_step(&p,&c,2148,30000,0,2048,.02f,&out));assert(out<=800&&out>=-800);}
 assert(out==800);assert((p.trace_flags&3)==3&&!(p.trace_flags&4)&&p.trace_p>800&&p.trace_d==0);
 assert(fabsf(p.integral)<.01f); /* saturating P must not accumulate I */
 assert(position_pid_step(&p,&c,1948,30000,0,2048,.02f,&out));assert(out==-800&&!(p.trace_flags&4)); /* immediate full reversal */
 for(unsigned i=0;i<100;++i){assert(position_pid_step(&p,&c,1948,30000,0,2048,.02f,&out));assert(out<=800&&out>=-800);}
 assert(out==-800&&fabsf(p.integral)<.01f);
 c.duty_limit=801;assert(!position_pid_step(&p,&c,2148,30000,0,2048,.02f,&out));
 c=position_pid_default_config();c.kp=0;c.kd=0;c.ki=10;p=(position_pid_t){0};
 for(unsigned i=0;i<100;++i)assert(position_pid_step(&p,&c,2058,30000,0,2048,.02f,&out));
 assert(p.integral>19.9f&&p.integral<=20.0f);
 position_pid_reset(&p,2048);assert(p.integral==0&&p.velocity==0&&p.output==0);
 c=position_pid_default_config();p=(position_pid_t){0};
 assert(position_pid_step(&p,&c,3000,200,10,2048,.02f,&out));assert(p.reference-2048<.5f); /* ACC ramp */
 assert(!position_pid_step(&p,&c,4096,200,0,2048,.02f,&out));
 assert(!position_pid_step(&p,&c,2048,0,0,2048,.02f,&out));
 assert(!position_pid_step(&p,&c,2048,200,0,2048,NAN,&out));
 assert(!position_pid_step(&p,&c,2048,200,0,2048,.051f,&out));
 assert(position_pid_step(&p,&c,2048,200,0,2048,50*.001f,&out));
 c.polarity=-1;p=(position_pid_t){0};assert(position_pid_step(&p,&c,2148,200,0,2048,.02f,&out)&&out<0);
 /* D follows the current measured derivative immediately, with no filter tail. */
 c=position_pid_default_config();c.kp=c.ki=0;p=(position_pid_t){0};
 assert(position_pid_step(&p,&c,2048,30000,0,2048,.02f,&out)&&out==0);
 assert(position_pid_step(&p,&c,2048,30000,0,2049,.02f,&out));
 assert(p.velocity==50&&p.trace_d==-2.5f&&out==-3);
 assert(position_pid_step(&p,&c,2048,30000,0,2049,.02f,&out)&&p.velocity==0&&out==0);
 assert(position_pid_step(&p,&c,2048,30000,0,2048,.01f,&out)&&p.velocity==-100&&p.trace_d==5&&out==5);
 c=position_pid_default_config();p=(position_pid_t){0};float q=2048,v=0;
 for(unsigned i=0;i<1000;++i){assert(position_pid_step(&p,&c,2148,400,30,(int16_t)lroundf(q),.02f,&out));
  v+=(20.0f*out-10.0f*v)*.02f;q+=v*.02f;assert(q>1900&&q<2300);}
 assert(fabsf(q-2148)<10); /* synthetic plant only, not hardware validation */
 puts("PID: immediate duty/reversal, unfiltered D, duty anti-windup, reset, profile, polarity and synthetic loop passed");return 0;
}
