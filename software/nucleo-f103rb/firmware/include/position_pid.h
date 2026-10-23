#ifndef YERTLE_POSITION_PID_H
#define YERTLE_POSITION_PID_H
#include <stdbool.h>
#include <stdint.h>
/* Units: position encoder steps, time seconds, output duty raw /1000. These
 * gains are independent of servo EEPROM PID registers. No angle/raw remapping. */
typedef struct {
    float kp,ki,kd,integral_limit,duty_limit,slew_per_s,derivative_tau_s;
    int8_t polarity;
} position_pid_config_t;
typedef struct {
    float reference,reference_velocity,previous_position,velocity,integral,output;
    bool initialized;
} position_pid_t;
position_pid_config_t position_pid_default_config(void);
void position_pid_reset(position_pid_t *pid,float measured);
bool position_pid_step(position_pid_t *pid,const position_pid_config_t *config,
    uint16_t goal,uint16_t speed,uint8_t acceleration,int16_t measured,float dt_s,int16_t *duty);
#endif
