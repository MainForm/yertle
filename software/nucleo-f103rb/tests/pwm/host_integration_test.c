#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "calibration_store.h"
void calibration_memory_reset(void);
static void seed_calibration(void){calibration_memory_reset();calibration_cache_init();
 const uint8_t zero[2]={0};for(unsigned id=1;id<=12;++id)assert(calibration_cache_observe((uint8_t)id,31,zero,2));}
#include "../../firmware/src/host_protocol.c"
static TickType_t tick=100;static unsigned critical,on_count,native_writes;
static int16_t duty[12];static motor_target_set_t native_target;static uint16_t torque_mask;static uint8_t last_result,last_error;
static unsigned detail_frames,stats_frames;static bool boot_off_ok=true;
static uint8_t last_mode_status[YPWM_STATUS_LENGTH];
static uint8_t imu_tx_valid, imu_tx_status;
static bool imu_ready, imu_valid;
static unsigned tx_order,write_diag_order,result_order;static uint8_t mode_fail_id,servo_modes[12];

void pwm_test_enter(void){++critical;}void pwm_test_exit(void){assert(critical);--critical;}
TickType_t xTaskGetTickCount(void){return tick;}void vTaskDelay(TickType_t t){tick+=t;}
void vTaskDelayUntil(TickType_t *p,TickType_t t){tick=*p+t;*p=tick;}
bool host_uart_write_bytes(const uint8_t *data,size_t n){uint8_t raw[YERTLE_PROTOCOL_MAX_RAW],type=0;uint16_t seq=0;size_t length=0;
 assert(cobs_decode(data+1,n-2,raw,&length)&&validate_frame(raw,length,&type,&seq));
 ++tx_order;
 if(type==MESSAGE_EVENT&&raw[7]==7){assert(raw[5]==64);++detail_frames;}
 if(type==MESSAGE_EVENT&&raw[7]==8){assert(raw[5]==28);++stats_frames;}
 if(type==MESSAGE_EVENT&&raw[7]==YPWM_EVENT_CODE){assert(raw[5]==YPWM_STATUS_LENGTH);memcpy(last_mode_status,raw+7,sizeof last_mode_status);}
 if(type==REGISTER_WRITE_DIAGNOSTICS_TYPE){write_diag_order=tx_order;assert(raw[5]==64);}
 if(type==MESSAGE_IMU_FEEDBACK){assert(raw[5]==24);imu_tx_valid=raw[15];imu_tx_status=raw[16];}
 if(type==MESSAGE_ACK||type==MESSAGE_NACK){last_result=type;last_error=raw[10];result_order=tx_order;}return true;
}
bool host_uart_try_read(uint8_t *b,bool *d){(void)b;*d=false;return false;}
motor_device_status_t motor_device_read_position_offset(uint8_t id, motor_position_offset_t *offset)
{ (void)id; (void)offset; assert(0); return MOTOR_DEVICE_BUSY; }
motor_register_status_t motor_device_read_register(uint8_t id, uint8_t address, uint8_t length, motor_register_data_t *result)
{ assert(!critical&&address==0x21U&&length==1U&&id>=1U&&id<=12U);
  result->data[0]=servo_modes[id-1U];return MOTOR_REGISTER_OK; }
motor_register_write_report_t motor_device_write_register(const motor_register_write_request_t *request, bool (*allowed)(void))
{
    assert(!critical && allowed());
    motor_register_write_report_t report = motor_register_initial_report();
    if (request->address == 0x21U) {
        assert(!motor_control_can_arm());
        report.status = request->motor_id == mode_fail_id ? MOTOR_WRITE_VERIFY_MISMATCH : MOTOR_WRITE_OK;
        if (report.status == MOTOR_WRITE_OK) servo_modes[request->motor_id - 1U] = report.verified_value = request->data[0];
    }
    return report;
}
uint16_t motor_device_register_lock_uncertain_mask(void){return 0;}
void uart_diagnostics_send(void *c,uart_diag_sender_t send){(void)c;(void)send;}
void uart_diagnostics_send_motor_buses(void *c, uart_diag_sender_t s,uint16_t q){(void)c;(void)s;(void)q;}
void pmw3901_spi_send_diagnostics(void *c,pmw_spi_diag_sender_t s,uint16_t q){(void)c;(void)s;(void)q;}
bool pmw3901_spi_auto_payload(uint8_t p[64]){(void)p;return false;}
void pmw3901_spi_auto_result(bool sent){(void)sent;}
bool sensor_get_motor_feedback_snapshot(motor_feedback_set_t *f){assert(!critical);memset(f,0,sizeof *f);
 f->sequence=tick/20+1;f->uptime_ms=tick;f->valid_mask=YPWM_ALL_MASK;for(unsigned i=0;i<12;++i){f->motor[i].valid=true;f->motor[i].position=2048;f->motor[i].voltage=60;}return true;}
bool motor_device_validate_targets(const motor_target_set_t *g){for(unsigned i=0;i<12;++i)if(g->motor[i].position>4095||g->motor[i].speed>32767)return false;return true;}
motor_device_status_t motor_device_write_targets(const motor_target_set_t *g){assert(!critical);native_target=*g;++native_writes;return MOTOR_DEVICE_OK;}
motor_device_status_t motor_device_set_torque(bool on){assert(!critical);if(on){torque_mask=YPWM_ALL_MASK;return MOTOR_DEVICE_OK;}if(!boot_off_ok)return MOTOR_DEVICE_UART_ERROR;torque_mask=0;return MOTOR_DEVICE_OK;}
motor_device_status_t motor_device_torque_mask(uint16_t mask,bool on){assert(!critical);if(on){++on_count;torque_mask|=mask;}else torque_mask&=(uint16_t)~mask;return MOTOR_DEVICE_OK;}
bool motor_device_verify_torque_off(uint16_t *unknown){assert(!critical);*unknown=torque_mask;return !torque_mask;}
bool motor_device_verify_modes(uint8_t backend,uint16_t mask,uint16_t *verified){
 assert(!critical);*verified=0;for(unsigned i=0;i<12;++i)if((mask&(1U<<i))&&servo_modes[i]==ypwm_expected_servo_mode(backend))*verified|=(uint16_t)(1U<<i);
 return *verified==mask;
}
bool motor_device_verify_modes_detailed(uint8_t backend,uint16_t mask,uint16_t *verified,motor_verify_report_t *r){*r=motor_verify_initial_report();return motor_device_verify_modes(backend,mask,verified);}
bool motor_device_verify_torque_off_detailed(uint16_t *unknown,motor_verify_report_t *r){*r=motor_verify_initial_report();return motor_device_verify_torque_off(unknown);}
void motor_device_read_modes(uint16_t *position, uint16_t *pwm)
{
    assert(!critical);
    *position = *pwm = 0U;
    for(unsigned i=0;i<12;++i){
        if (servo_modes[i]==0U) *position|=(uint16_t)(1U<<i);
        if (servo_modes[i]==2U) *pwm|=(uint16_t)(1U<<i);
    }
}
motor_device_status_t motor_device_write_pwm(uint16_t mask,const int16_t values[12]){assert(!critical&&mask);memcpy(duty,values,sizeof duty);torque_mask|=mask;return MOTOR_DEVICE_OK;}
static void receive(protocol_context_t *c,uint8_t type,const uint8_t *p,uint16_t n){
 static uint16_t seq;uint8_t raw[YERTLE_PROTOCOL_MAX_RAW]={1,type};ypwm_put16(raw+3,++seq);raw[5]=(uint8_t)n;if(n)memcpy(raw+7,p,n);
 ypwm_put16(raw+7+n,crc16_ccitt(raw,7+n));handle_frame(c,raw,9+n);
}
static void write_mode(protocol_context_t *c, uint8_t id, uint8_t mode, uint8_t result)
{
    const uint8_t payload[]={id,0x21,1,1,mode};
    receive(c,MESSAGE_SET_MOTOR_REGISTER,payload,sizeof payload);
    assert(c->register_job_active);
    service_motor_register_write(c);
    assert(last_result==result && write_diag_order<result_order);
}

int main(void)
{
    seed_calibration();
    protocol_context_t c={0};
    published_state=c.state=HOST_FSM_DISARMED;
    motor_control_init(); motor_control_service();
    receive(&c,MESSAGE_HELLO,NULL,0);
    const uint8_t imu_status_request=0x01U, bad_sensor_flag=0x02U;
    receive(&c,MESSAGE_GET_SENSOR_FEEDBACK,&imu_status_request,1);
    assert(imu_tx_status==1U&&!imu_tx_valid);
    imu_ready=true;
    receive(&c,MESSAGE_GET_SENSOR_FEEDBACK,&imu_status_request,1);
    assert(imu_tx_status==2U&&!imu_tx_valid);
    imu_valid=true;
    receive(&c,MESSAGE_GET_SENSOR_FEEDBACK,&imu_status_request,1);
    assert(imu_tx_status==2U&&imu_tx_valid);
    receive(&c,MESSAGE_GET_SENSOR_FEEDBACK,NULL,0);
    assert(imu_tx_status==0U&&imu_tx_valid);
    receive(&c,MESSAGE_GET_SENSOR_FEEDBACK,&bad_sensor_flag,1);
    assert(last_result==MESSAGE_NACK&&last_error==ERROR_INVALID_FLAGS);
    assert(!c.lease_established);
    const uint8_t ordinary[]={1,0x15,1,1,40};
    receive(&c,MESSAGE_SET_MOTOR_REGISTER,ordinary,sizeof ordinary);
    service_motor_register_write(&c);
    assert(last_result==MESSAGE_ACK);
    write_mode(&c,1,2,MESSAGE_ACK);
    receive(&c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(&c);
    assert(c.state==HOST_FSM_DISARMED && !motor_control_accepts_targets());
    const uint8_t direct[24]={0};
    receive(&c,MESSAGE_SET_MOTOR_PWM,direct,sizeof direct);
    assert(last_result==MESSAGE_NACK && torque_mask==0 && on_count==0);
    mode_fail_id=2;
    write_mode(&c,2,2,MESSAGE_NACK);
    assert(servo_modes[0]==2 && servo_modes[1]==0);
    mode_fail_id=0;
    for(uint8_t id=2;id<=12;++id) write_mode(&c,id,2,MESSAGE_ACK);
    try_enter_control_ready(&c);
    assert(c.state==HOST_FSM_DISARMED);
    receive(&c,MESSAGE_HEARTBEAT,NULL,0);try_enter_control_ready(&c);
    assert(c.state==HOST_FSM_ARMED);
    uint8_t pwm[24]={0};ypwm_put16(pwm,200);
    receive(&c,MESSAGE_SET_MOTOR_PWM,pwm,sizeof pwm);
    motor_control_service(); service_pwm(&c);
    assert(last_result==MESSAGE_ACK && duty[0]==200 && torque_mask==YPWM_ALL_MASK);
    const uint8_t mode[]={1,0x21,1,1,0};
    receive(&c,MESSAGE_SET_MOTOR_REGISTER,mode,sizeof mode);
    assert(last_result==MESSAGE_NACK && servo_modes[0]==2);
    uint8_t targets[60]={0};
    for(unsigned id=0;id<12;++id){ypwm_put16(targets+5*id,2200);ypwm_put16(targets+5*id+2,200);}
    receive(&c,MESSAGE_SET_JOINT_TARGETS,targets,60);assert(last_result==MESSAGE_ACK);motor_control_service();
    assert(duty[0]!=200); /* existing position->PID route resumes */
    /* Host pressure delays diagnostics; never pop a partial set before ACKs. */
    unsigned before_detail=detail_frames;c.result_count=1;service_pid_detail(&c);assert(detail_frames==before_detail);c.result_count=0;
    c.host_connected=false;service_pid_detail(&c);assert(detail_frames==before_detail);c.host_connected=true;
    for(unsigned pair=0;pair<6;++pair){service_pid_detail(&c);assert(detail_frames==before_detail+pair+1);}
    /* Continuous diagnostics still emit 1Hz producer/loss counters. */
    tick+=1000;service_pid_detail(&c);assert(stats_frames>0);
    puts("host register writes serialize mode updates and block mixed-mode ARM");
    return 0;
}

bool sensor_get_extended_feedback_snapshot(motor_feedback_set_t *m, imu_feedback_t *i, pmw3901_feedback_t *f)
{memset(i,0,sizeof *i);i->ready=imu_ready;i->valid=imu_valid;memset(f,0,sizeof *f);return sensor_get_motor_feedback_snapshot(m);}
