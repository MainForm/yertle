#include "position_pid.h"
#include "pwm_contract.h"
#include <math.h>
#include <string.h>
static float clamp(float value,float low,float high){return value<low?low:(value>high?high:value);}
position_pid_config_t position_pid_default_config(void){
    const position_pid_config_t config={10.0f,3.0f,1.0f,100.0f,YPWM_DUTY_LIMIT,1};
    return config;
}
int8_t position_pid_motor_polarity(uint8_t motor_id){
    /* Bench logs robot_20261026_192238_20012 (H) and
     * robot_20261026_203423_16352 (U): sustained opposite raw motion.
     * robot_20261026_213947_37532 (H): ID3 moved away with negative PWM.
     * Reverse ID3; ID9 is also reversed by explicit user direction,
     * pending a physical direction check (its logged motion was too small).
     * This is NOT the joint-angle-to-raw direction_sign calibration. */
    static const int8_t polarity[12]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    return motor_id>=1U&&motor_id<=12U?polarity[motor_id-1U]:0;
}
void position_pid_reset(position_pid_t *pid,float measured){
    memset(pid,0,sizeof *pid);pid->reference=measured;pid->previous_position=measured;pid->initialized=true;
}
bool position_pid_step(position_pid_t *pid,const position_pid_config_t *c,uint16_t goal,
    int16_t measured,float dt,int16_t *duty){
    if(!pid||!c||!duty||goal>4095||measured<0||measured>4095||
       !isfinite(dt)||dt<=0||!isfinite(c->kp)||!isfinite(c->ki)||!isfinite(c->kd)||
       c->kp<0||c->ki<0||c->kd<0||!isfinite(c->integral_limit)||c->integral_limit<0||
       !isfinite(c->duty_limit)||c->duty_limit<=0||c->duty_limit>YPWM_DUTY_LIMIT||
       (c->polarity!=1&&c->polarity!=-1))return false;
    const bool reset=!pid->initialized;
    if(reset)position_pid_reset(pid,(float)measured);
    /* PWM backend follows the received absolute goal immediately. Speed/ACC
     * belong to native servo position packets; no software reference ramp. */
    pid->reference=(float)goal;
    /* Derivative on measurement, without a low-pass filter. Goal changes alone
     * do not create derivative kick. Actual fresh-feedback dt is preserved. */
    pid->velocity=((float)measured-pid->previous_position)/dt;
    pid->previous_position=(float)measured;
    const float error=pid->reference-(float)measured;
    if(fabsf(error)>(float)YPWM_FOLLOWING_ERROR_LIMIT)return false;
    const float integral_unclipped=pid->integral+c->ki*error*dt;
    const float candidate=clamp(integral_unclipped,-c->integral_limit,c->integral_limit);
    const float without_i=c->kp*error-c->kd*pid->velocity;
    const float unsaturated=without_i+candidate;
    /* Conditional integration prevents windup at the retained duty ceiling.
     * No output change-rate limit: successive PWM commands may reverse sign. */
    const float low=-c->duty_limit,high=c->duty_limit;
    if(!((unsaturated>high&&error>0)||(unsaturated<low&&error<0)))pid->integral=candidate;
    const float requested=clamp(without_i+pid->integral,low,high);
    pid->output=(float)c->polarity*requested;
    if(!isfinite(pid->output))return false;
    /* Trace arithmetic is observational: never feeds the controller. */
    pid->trace_error=error;pid->trace_p=c->kp*error;pid->trace_d=-c->kd*pid->velocity;
    const float requested_unclipped=without_i+pid->integral;
    pid->trace_flags=(uint8_t)(1U|(fabsf(requested_unclipped)>c->duty_limit?2U:0U)|
        (fabsf(integral_unclipped)>c->integral_limit?8U:0U)|
        ((unsaturated>high&&error>0)||(unsaturated<low&&error<0)?16U:0U)|
        (c->polarity<0?32U:0U)|(reset?64U:0U));
    *duty=(int16_t)lroundf(pid->output);return true;
}
