#include "position_pid.h"
#include "pwm_contract.h"
#include <math.h>
#include <string.h>
static float clamp(float value,float low,float high){return value<low?low:(value>high?high:value);}
position_pid_config_t position_pid_default_config(void){
    const position_pid_config_t config={2.0f,0.5f,0.05f,20.0f,YPWM_DUTY_LIMIT,2000.0f,0.04f,1};
    return config;
}
void position_pid_reset(position_pid_t *pid,float measured){
    memset(pid,0,sizeof *pid);pid->reference=measured;pid->previous_position=measured;pid->initialized=true;
}
bool position_pid_step(position_pid_t *pid,const position_pid_config_t *c,uint16_t goal,
    uint16_t speed,uint8_t acceleration,int16_t measured,float dt,int16_t *duty){
    if(!pid||!c||!duty||goal>4095||measured<0||measured>4095||!speed||speed>32767||
       !isfinite(dt)||dt<=0||dt>0.050001f||!isfinite(c->kp)||!isfinite(c->ki)||!isfinite(c->kd)||
       c->kp<0||c->ki<0||c->kd<0||!isfinite(c->integral_limit)||c->integral_limit<0||
       !isfinite(c->duty_limit)||c->duty_limit<=0||c->duty_limit>YPWM_DUTY_LIMIT||
       !isfinite(c->slew_per_s)||c->slew_per_s<=0||!isfinite(c->derivative_tau_s)||c->derivative_tau_s<0||
       (c->polarity!=1&&c->polarity!=-1))return false;
    const bool reset=!pid->initialized;
    if(reset)position_pid_reset(pid,(float)measured);
    /* Speed is explicitly encoder steps/s in this backend. ACC uses the STS
     * documented 8.7deg/s^2 per unit; zero means no acceleration ramp. */
    const float distance=(float)goal-pid->reference;
    const float a=(float)acceleration*(8.7f*4096.0f/360.0f);
    const float braking=acceleration?sqrtf(2.0f*a*fabsf(distance)):(float)speed;
    const float vmax=fminf((float)speed,braking);
    const float desired_velocity=distance<0?-vmax:vmax;
    pid->reference_velocity=acceleration?
        pid->reference_velocity+clamp(desired_velocity-pid->reference_velocity,-a*dt,a*dt):desired_velocity;
    float travel=pid->reference_velocity*dt;
    if((distance>=0&&travel>=distance)||(distance<=0&&travel<=distance)){
        travel=distance;pid->reference_velocity=0;
    }
    pid->reference=clamp(pid->reference+travel,0,4095);
    const float raw_velocity=((float)measured-pid->previous_position)/dt;
    pid->velocity+=dt/(c->derivative_tau_s+dt)*(raw_velocity-pid->velocity);
    pid->previous_position=(float)measured;
    const float error=pid->reference-(float)measured;
    if(fabsf(error)>(float)YPWM_FOLLOWING_ERROR_LIMIT)return false;
    const float integral_unclipped=pid->integral+c->ki*error*dt;
    const float candidate=clamp(integral_unclipped,-c->integral_limit,c->integral_limit);
    const float without_i=c->kp*error-c->kd*pid->velocity;
    const float unsaturated=without_i+candidate;
    /* Conditional integration prevents windup at duty and slew saturation. */
    const float physical_previous=pid->output*(float)c->polarity;
    const float low=fmaxf(-c->duty_limit,physical_previous-c->slew_per_s*dt);
    const float high=fminf(c->duty_limit,physical_previous+c->slew_per_s*dt);
    if(!((unsaturated>high&&error>0)||(unsaturated<low&&error<0)))pid->integral=candidate;
    const float requested=clamp(without_i+pid->integral,low,high);
    pid->output=(float)c->polarity*requested;
    if(!isfinite(pid->output))return false;
    /* Trace arithmetic is observational: never feeds the controller. */
    pid->trace_error=error;pid->trace_p=c->kp*error;pid->trace_d=-c->kd*pid->velocity;
    const float requested_unclipped=without_i+pid->integral;
    const float duty_clamped=clamp(requested_unclipped,-c->duty_limit,c->duty_limit);
    pid->trace_flags=(uint8_t)(1U|(fabsf(requested_unclipped)>c->duty_limit?2U:0U)|
        (fabsf(duty_clamped-physical_previous)>c->slew_per_s*dt?4U:0U)|
        (fabsf(integral_unclipped)>c->integral_limit?8U:0U)|
        ((unsaturated>high&&error>0)||(unsaturated<low&&error<0)?16U:0U)|
        (c->polarity<0?32U:0U)|(reset?64U:0U));
    *duty=(int16_t)lroundf(pid->output);return true;
}
