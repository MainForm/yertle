#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../../firmware/src/host_protocol.c"
static TickType_t tick=100;static unsigned critical,queues,on_count,native_writes;static int queue;
static int16_t duty[12];static uint16_t torque_mask;static uint8_t last_result,last_error;
void pwm_test_enter(void){++critical;}void pwm_test_exit(void){assert(critical);--critical;}
TickType_t xTaskGetTickCount(void){return tick;}void vTaskDelay(TickType_t t){tick+=t;}
void vTaskDelayUntil(TickType_t *p,TickType_t t){tick=*p+t;*p=tick;}
void *xQueueCreate(unsigned n,unsigned size){assert(n==1&&size);return &queue;}
BaseType_t xQueueOverwrite(void *q,const void *p){assert(q&&p);++queues;return pdPASS;}
BaseType_t xQueueReceive(void *q,void *p,TickType_t timeout){(void)q;(void)p;(void)timeout;return 0;}
BaseType_t xQueueReset(void *q){assert(q);return pdPASS;}
bool host_uart_write_bytes(const uint8_t *data,size_t n){uint8_t raw[YERTLE_PROTOCOL_MAX_RAW],type=0;uint16_t seq=0;size_t length=0;
 assert(cobs_decode(data+1,n-2,raw,&length)&&validate_frame(raw,length,&type,&seq));
 if(type==MESSAGE_ACK||type==MESSAGE_NACK){last_result=type;last_error=raw[10];}return true;
}
bool host_uart_try_read(uint8_t *b,bool *d){(void)b;*d=false;return false;}
bool sensor_take_position_offsets(motor_position_offsets_t *r){(void)r;return false;}
bool sensor_request_position_offsets(uint16_t s){(void)s;return false;}
bool sensor_take_motor_register(motor_register_result_t *r){(void)r;return false;}
bool sensor_request_motor_register(uint16_t s,uint8_t id,uint8_t a,uint8_t n){(void)s;(void)id;(void)a;(void)n;return false;}
bool sensor_take_motor_register_write(motor_register_write_result_t *r){(void)r;return false;}
bool sensor_request_motor_register_write(uint16_t s,const motor_register_write_request_t *r){(void)s;(void)r;return false;}
uint16_t motor_device_register_lock_uncertain_mask(void){return 0;}
void uart_diagnostics_send(void *c,uart_diag_sender_t send){(void)c;(void)send;}
bool sensor_get_feedback_snapshot(motor_feedback_set_t *f,imu_feedback_t *imu){assert(!critical);memset(f,0,sizeof *f);memset(imu,0,sizeof *imu);
 f->sequence=tick/20+1;f->uptime_ms=tick;f->valid_mask=YPWM_ALL_MASK;for(unsigned i=0;i<12;++i){f->motor[i].valid=true;f->motor[i].position=2048;f->motor[i].voltage=60;}return true;}
bool motor_device_validate_targets(const motor_target_set_t *g){for(unsigned i=0;i<12;++i)if(g->motor[i].position>4095||g->motor[i].speed>32767)return false;return true;}
motor_device_status_t motor_device_write_targets(const motor_target_set_t *g){(void)g;++native_writes;return MOTOR_DEVICE_OK;}
motor_device_status_t motor_device_set_torque(bool on){assert(!critical&&!on);return MOTOR_DEVICE_OK;}
motor_device_status_t motor_device_torque_mask(uint16_t mask,bool on){assert(!critical);if(on){++on_count;torque_mask=mask;}return MOTOR_DEVICE_OK;}
bool motor_device_verify_torque_off(uint16_t *unknown){assert(!critical);*unknown=0;return true;}
bool motor_device_verify_modes(uint8_t backend,uint16_t mask,uint16_t *verified){(void)backend;(void)mask;assert(!critical);*verified=YPWM_ALL_MASK;return true;}
motor_register_write_report_t motor_device_write_mode(uint8_t id,uint8_t mode,bool (*allowed)(void)){
 assert(!critical&&id>=1&&id<=12&&(mode==0||mode==2)&&allowed());motor_register_write_report_t r=motor_register_initial_report();r.status=MOTOR_WRITE_OK;return r;}
motor_device_status_t motor_device_write_pwm(uint16_t mask,const int16_t values[12]){assert(!critical&&mask);memcpy(duty,values,sizeof duty);return MOTOR_DEVICE_OK;}
static void receive(protocol_context_t *c,uint8_t type,const uint8_t *p,uint16_t n){
 uint8_t raw[YERTLE_PROTOCOL_MAX_RAW]={1,type,0,42,0};raw[5]=(uint8_t)n;if(n)memcpy(raw+7,p,n);
 ypwm_put16(raw+7+n,crc16_ccitt(raw,7+n));handle_frame(c,raw,9+n);
}
int main(void){protocol_context_t c={0};c.host_connected=true;c.state=HOST_FSM_DISARMED;c.last_host_heartbeat_tick=tick;published_state=HOST_FSM_DISARMED;
 assert(actuator_task_init());assert(pwm_control_service());assert(pwm_control_native_ready());
 uint8_t mode[8]={1,1};ypwm_put16(mode+2,YPWM_ALL_MASK);ypwm_put32(mode+4,123);
 receive(&c,MESSAGE_SET_CONTROL_MODE,mode,8);assert(last_result==MESSAGE_NACK&&!c.pwm_mode_pending);mode[0]=2;
 receive(&c,MESSAGE_SET_CONTROL_MODE,mode,8);assert(c.pwm_mode_pending);receive(&c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(&c);assert(c.state==HOST_FSM_DISARMED);
 assert(pwm_control_service());service_pwm(&c);assert(!c.pwm_mode_pending&&!c.lease_established);receive(&c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(&c);assert(c.state==HOST_FSM_ARMED);
 uint8_t targets[60]={0};for(unsigned i=0;i<12;++i){ypwm_put16(targets+5*i,2200);ypwm_put16(targets+5*i+2,200);}
 receive(&c,MESSAGE_SET_JOINT_TARGETS,targets,60);assert(last_result==MESSAGE_ACK&&last_error==0&&queues==0);
 assert(pwm_control_service());assert(on_count==1&&torque_mask==YPWM_ALL_MASK&&native_writes==0);for(unsigned i=0;i<12;++i)assert(duty[i]>0&&duty[i]<=100);
 uint8_t legacy[37]={0};receive(&c,MESSAGE_RELEASE_CONTROL,legacy,37);assert(last_result==MESSAGE_NACK);
 const uint8_t release[1]={2};receive(&c,MESSAGE_RELEASE_CONTROL,release,1);service_safety_state(&c);assert(c.state==HOST_FSM_DISARMED);
 assert(pwm_control_service());service_safety_state(&c);assert(!host_protocol_is_stop_requested());for(unsigned i=0;i<12;++i)assert(duty[i]==0);
 mode[1]=0;ypwm_put32(mode+4,124);receive(&c,MESSAGE_SET_CONTROL_MODE,mode,8);assert(c.pwm_mode_pending);assert(pwm_control_service());service_pwm(&c);
 assert(pwm_control_native_ready());receive(&c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(&c);receive(&c,MESSAGE_SET_JOINT_TARGETS,targets,60);
 assert(last_result==MESSAGE_ACK&&queues==1);puts("Actual host+actuator+PID: unchanged0x06 to PWM, no native writes, explicit release/restoration passed");return 0;}
