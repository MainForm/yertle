#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_device.h"
#include "pwm_contract.h"
#include "motor_uart.h"
#include "uart_diagnostics.h"
#include "semphr.h"
static uint8_t memory[12][256],reply[48],last[160];
static size_t reply_length,last_length;
static unsigned depth,clock_ms,tx_count;
static uint8_t fail_write_id,override_read_id,servo_fault_id;
static unsigned transient_mode_failures,mode_target_writes;
static bool permission=true,apply_then_lose_ack,cancel_on_error,ignore_relock;
static bool allowed(void){return permission;}
SemaphoreHandle_t xSemaphoreCreateMutex(void){return (void*)1;}
int xSemaphoreTake(SemaphoreHandle_t m,TickType_t t){(void)t;assert(m&&!depth);++depth;return 1;}
int xSemaphoreGive(SemaphoreHandle_t m){assert(m&&depth==1);--depth;return 1;}
uint32_t xTaskGetTickCount(void){return clock_ms;}
void vTaskDelay(uint32_t t){clock_ms+=t;}
void motor_uart_init(void){}
void motor_uart_discard_rx(void){}
void motor_diagnostics_record(enum motor_diag_reason r){(void)r;}
void motor_transport_record(enum motor_transport_event e){(void)e;}
void motor_diagnostics_masks(uint16_t a,uint16_t b){(void)a;(void)b;}
bool motor_uart_write(const uint8_t *p,size_t n,uint32_t timeout){
 (void)timeout;assert(depth==1&&n<sizeof last);memcpy(last,p,n);last_length=n;++tx_count;
 unsigned sum=0;for(size_t k=2;k<n;++k)sum+=p[k];assert((sum&255U)==255U);
 if(p[4]==3&&p[5]==0x21){
  ++mode_target_writes;
  if(apply_then_lose_ack){memcpy(memory[p[2]-1]+p[5],p+6,n-7);apply_then_lose_ack=false;return false;}
  if(p[2]==fail_write_id||transient_mode_failures){
   if(transient_mode_failures)--transient_mode_failures;
   if(cancel_on_error)permission=false;
   return false;
  }
 }
 if(p[4]==0x83){assert(p[2]==254);unsigned width=p[6];
  for(size_t k=7;k<n-1;k+=width+1){assert(p[k]>=1&&p[k]<=12);memcpy(memory[p[k]-1]+p[5],p+k+1,width);}return true;}
 if(p[2]==254){assert(p[4]==3&&p[5]==0x28&&p[6]==0);for(unsigned i=0;i<12;++i)memory[i][0x28]=0;return true;}
 assert(p[2]>=1&&p[2]<=12);unsigned width=0;
 if(p[4]==2)width=p[6];else {assert(p[4]==3);
  if(!(ignore_relock&&p[5]==55&&p[6]==1))memcpy(memory[p[2]-1]+p[5],p+6,n-7);}
 reply[0]=reply[1]=255;reply[2]=p[2];reply[3]=(uint8_t)(width+2);reply[4]=0;
 if(p[4]==3&&p[5]==0x21&&p[2]==servo_fault_id)reply[4]=4;
 memcpy(reply+5,memory[p[2]-1]+p[5],width);reply_length=width+6;
 if(p[4]==2&&p[2]==override_read_id&&p[5]==0x21)reply[5]=1;
 sum=0;for(size_t k=2;k<reply_length-1;++k)sum+=reply[k];reply[reply_length-1]=(uint8_t)~sum;return true;
}
bool motor_uart_read_frame(uint8_t *p,size_t cap,size_t *n,uint32_t timeout){
 (void)timeout;assert(depth==1&&cap>=reply_length);memcpy(p,reply,reply_length);*n=reply_length;return true;}
int main(void){
 assert(motor_device_init());for(unsigned i=0;i<12;++i){memory[i][8]=1;memory[i][55]=1;}
 uint16_t verified,unknown;
 assert(motor_device_verify_modes(YPWM_POSITION,YPWM_ALL_MASK,&verified)&&verified==YPWM_ALL_MASK);
 memory[3][0x21]=2;assert(!motor_device_verify_modes(YPWM_POSITION,YPWM_ALL_MASK,&verified));
 assert(motor_device_verify_modes(YPWM_PID,1U<<3,&verified));
 for(unsigned i=0;i<12;++i)memory[i][0x21]=2;
 assert(motor_device_verify_modes(YPWM_PID,YPWM_ALL_MASK,&verified)&&verified==YPWM_ALL_MASK);
 assert(motor_device_verify_torque_off(&unknown)&&unknown==0);
 memory[4][0x28]=1;assert(!motor_device_verify_torque_off(&unknown)&&unknown==(1U<<4));memory[4][0x28]=0;
 int16_t pwm[12]={0};pwm[3]=-100;
 assert(motor_device_write_pwm(1U<<3,pwm)==MOTOR_DEVICE_OK);
 assert(last[2]==254&&last[4]==0x83&&last[5]==0x2c&&last[6]==2&&last[7]==4);
 assert(memory[3][0x2c]==100&&memory[3][0x2d]==4&&last_length==11);
 pwm[3]=1000;assert(motor_device_write_pwm(1U<<3,pwm)==MOTOR_DEVICE_OK);
 assert(memory[3][0x2c]==0xe8&&memory[3][0x2d]==0x03);
 pwm[3]=-1000;assert(motor_device_write_pwm(1U<<3,pwm)==MOTOR_DEVICE_OK);
 assert(memory[3][0x2c]==0xe8&&memory[3][0x2d]==0x07); /* magnitude1000 + BIT10 */
 const unsigned before=tx_count;pwm[3]=-1001;
 assert(motor_device_write_pwm(1U<<3,pwm)==MOTOR_DEVICE_INVALID_ARGUMENT&&tx_count==before);
 pwm[3]=1001;
 assert(motor_device_write_pwm(1U<<3,pwm)==MOTOR_DEVICE_INVALID_ARGUMENT&&tx_count==before);
 pwm[3]=0;pwm[0]=1;assert(motor_device_write_pwm(1U<<3,pwm)==MOTOR_DEVICE_INVALID_ARGUMENT&&tx_count==before);
 assert(motor_device_torque_mask(1U<<3,true)==MOTOR_DEVICE_OK&&memory[3][0x28]==1&&memory[0][0x28]==0);
 assert(motor_device_torque_mask(YPWM_ALL_MASK,false)==MOTOR_DEVICE_OK);
 assert(motor_device_verify_torque_off(&unknown));
 memory[1][0x21]=0;
 motor_register_write_report_t r=motor_device_write_mode(2,2,allowed);
 assert(r.status==MOTOR_WRITE_OK&&r.initial_value==0&&r.verified_value==2&&r.last_read_value==2);
 assert(r.last_read_status==MOTOR_REGISTER_OK&&r.last_read_address==0x21&&r.last_write_status==MOTOR_REGISTER_OK);
 assert(r.lock_restore_verified&&memory[1][55]==1);
 memory[2][0x21]=0;fail_write_id=3;mode_target_writes=0;r=motor_device_write_mode(3,2,allowed);fail_write_id=0;
 assert(r.mode_attempts==1&&r.mode_attempt_limit==1&&mode_target_writes==1&&r.io_detail==MOTOR_IO_TX);
 assert(r.status==MOTOR_WRITE_IO_ERROR&&r.primary_status==MOTOR_WRITE_IO_ERROR&&r.failure_stage==MOTOR_WRITE_STAGE_TARGET);
 assert(r.target_write_attempted&&r.last_write_status==MOTOR_REGISTER_COMMUNICATION_ERROR&&r.last_write_address==0x21);
 assert(r.initial_value==0&&r.verified_value==255&&r.lock_restore_verified&&memory[2][55]==1);
 memory[3][0x21]=0;override_read_id=4;r=motor_device_write_mode(4,2,allowed);override_read_id=0;
 assert(r.status==MOTOR_WRITE_VERIFY_MISMATCH&&r.failure_stage==MOTOR_WRITE_STAGE_VERIFY&&r.verified_value==1&&r.last_read_value==1);
 assert(r.lock_restore_verified&&memory[3][55]==1);
 motor_verify_report_t v;memory[4][0x21]=0;
 assert(!motor_device_verify_modes_detailed(YPWM_PID,YPWM_ALL_MASK,&verified,&v)&&v.motor_id==3&&v.address==0x21&&v.actual==0&&v.io_status==0);
 memory[4][0x28]=1;assert(!motor_device_verify_torque_off_detailed(&unknown,&v)&&v.motor_id==5&&v.address==0x28&&v.actual==1);memory[4][0x28]=0;
 motor_mode_snapshot_t original;assert(motor_device_snapshot_modes(&original,&v)&&original.valid_mask==0xfff);
 for(unsigned i=0;i<12;++i)memory[i][0x21]=original.mode[i]==0?2:0;
 uint16_t restored;uint8_t failed_id;
 assert(motor_device_restore_modes(&original,allowed,&restored,&failed_id,&r)&&restored==0xfff&&!failed_id);
 for(unsigned i=0;i<12;++i)assert(memory[i][0x21]==original.mode[i]&&memory[i][55]==1);
 memory[2][0x21]=original.mode[2]==0?2:0;fail_write_id=3;
 assert(!motor_device_restore_modes(&original,allowed,&restored,&failed_id,&r)&&failed_id==3&&restored==(0xfff&~4U));
 assert(r.status==MOTOR_WRITE_IO_ERROR&&r.lock_restore_verified);fail_write_id=0;
 override_read_id=4;assert(!motor_device_snapshot_modes(&original,&v)&&v.motor_id==4&&v.actual==1);override_read_id=0;
 /* The driver never retries. Five explicit caller requests are required. */
 memory[1][0x21]=0;transient_mode_failures=4;mode_target_writes=0;
 for(unsigned attempt=0;attempt<4;++attempt){r=motor_device_write_mode(2,2,allowed);
  assert(r.status==MOTOR_WRITE_IO_ERROR&&r.mode_attempts==1&&mode_target_writes==attempt+1);}
 r=motor_device_write_mode(2,2,allowed);
 assert(r.status==MOTOR_WRITE_OK&&r.mode_attempts==1&&mode_target_writes==5);
 assert(r.initial_value==0&&r.verified_value==2&&memory[1][55]==1);
 /* A lost ACK AFTER application must not consume EEPROM endurance again. */
 memory[1][0x21]=0;mode_target_writes=0;apply_then_lose_ack=true;r=motor_device_write_mode(2,2,allowed);
 assert(r.status==MOTOR_WRITE_IO_ERROR&&r.mode_attempts==1&&mode_target_writes==1);
 r=motor_device_write_mode(2,2,allowed);
 assert(r.status==MOTOR_WRITE_OK&&r.mode_attempts==1&&mode_target_writes==1&&memory[1][55]==1);
 /* Safety cancellation/servo faults/uncertain locks terminate before retry2. */
 memory[1][0x21]=0;mode_target_writes=0;fail_write_id=2;cancel_on_error=true;r=motor_device_write_mode(2,2,allowed);
 assert(r.mode_attempts==1&&mode_target_writes==1&&r.lock_restore_verified);
 permission=true;cancel_on_error=false;fail_write_id=0;
 memory[1][0x21]=0;servo_fault_id=2;r=motor_device_write_mode(2,2,allowed);servo_fault_id=0;
 assert(r.status==MOTOR_WRITE_SERVO_ERROR&&r.mode_attempts==1&&r.io_detail==MOTOR_IO_SERVO);
 memory[1][0x21]=0;ignore_relock=true;r=motor_device_write_mode(2,2,allowed);ignore_relock=false;
 assert(r.status==MOTOR_WRITE_LOCK_RESTORE_FAILED&&r.mode_attempts==1&&(motor_device_register_lock_uncertain_mask()&2));
 memory[1][55]=1;motor_register_data_t lock;assert(motor_device_read_register(2,55,1,&lock)==MOTOR_REGISTER_OK);
 assert(motor_device_register_lock_uncertain_mask()==0);
 puts("PASS actual PWM packet builders: BIT10, exact address/width, selected torque, range rejection, native/mixed readback");return 0;
}
