#ifndef YERTLE_POSITION_PID_H
#define YERTLE_POSITION_PID_H
#include <stdbool.h>
#include <stdint.h>
/* Units: position encoder steps, time seconds, output duty raw /1000. These
 * gains are independent of servo EEPROM PID registers. No angle/raw remapping. */
typedef struct {
    float kp,ki,kd,integral_limit,duty_limit;
    int8_t polarity;
} position_pid_config_t;
typedef struct {
    /* velocity is the unfiltered measured-position difference / actual dt. */
    float reference,reference_velocity,previous_position,velocity,integral,output;
    float trace_error,trace_p,trace_d;
    uint8_t trace_flags;
    bool initialized;
} position_pid_t;
position_pid_config_t position_pid_default_config(void);
/* Encoder-raw increase versus signed PWM, separate from simulation angle/raw
 * mapping. IDs 1,2,6,10,11,12 are reversed for the supported bench robot.
 * Invalid motor IDs return 0, rejected by position_pid_step (never guessed). */
int8_t position_pid_motor_polarity(uint8_t motor_id);
void position_pid_reset(position_pid_t *pid,float measured);
bool position_pid_step(position_pid_t *pid,const position_pid_config_t *config,
    uint16_t goal,uint16_t speed,uint8_t acceleration,int16_t measured,float dt_s,int16_t *duty);
#endif
