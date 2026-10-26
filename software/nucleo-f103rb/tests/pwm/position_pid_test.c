#undef NDEBUG
#include "position_pid.h"
#include "pwm_contract.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void){
 position_pid_config_t c=position_pid_default_config();position_pid_t p={0};int16_t out;
 assert(YPWM_DUTY_LIMIT==800&&c.duty_limit==800);
 assert(position_pid_step(&p,&c,2148,200,0,2048,.02f,&out));assert(out>0&&out<=40&&p.velocity==0&&p.reference<=2052.01f);
 const float previous=p.reference;assert(position_pid_step(&p,&c,1948,200,0,2048,.02f,&out));assert(p.reference<previous&&p.velocity==0);
 p=(position_pid_t){0};c.kp=100;c.ki=20;
 assert(position_pid_step(&p,&c,2148,30000,0,2048,.02f,&out));assert((p.trace_flags&6)==6);
 for(unsigned i=0;i<100;++i){assert(position_pid_step(&p,&c,2148,30000,0,2048,.02f,&out));assert(out<=800&&out>=-800);}
 assert(out==800);assert((p.trace_flags&3)==3&&!(p.trace_flags&4)&&p.trace_p>800&&p.trace_d==0);
 assert(fabsf(p.integral)<.01f); /* saturating P must not accumulate I */
 for(unsigned i=0;i<100;++i){assert(position_pid_step(&p,&c,1948,30000,0,2048,.02f,&out));assert(out<=800&&out>=-800);}
 assert(out==-800&&fabsf(p.integral)<.01f);
 c.duty_limit=801;assert(!position_pid_step(&p,&c,2148,30000,0,2048,.02f,&out));
 c=position_pid_default_config();c.kp=0;c.kd=0;c.ki=10;c.slew_per_s=10000;p=(position_pid_t){0};
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
 c=position_pid_default_config();p=(position_pid_t){0};float q=2048,v=0;
 for(unsigned i=0;i<1000;++i){assert(position_pid_step(&p,&c,2148,400,30,(int16_t)lroundf(q),.02f,&out));
  v+=(20.0f*out-10.0f*v)*.02f;q+=v*.02f;assert(q>1900&&q<2300);}
 assert(fabsf(q-2148)<10); /* synthetic plant only, not hardware validation */
 puts("PID: P/I/D, anti-windup, reset, no derivative kick, profile, polarity and synthetic loop passed");return 0;
}
