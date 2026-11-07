#define main heartbeat_regression_main
#include "heartbeat_test.c"
#undef main
static unsigned halves=2,attempts;
bool pmw3901_spi_auto_payload(uint8_t p[64]){
 if(!halves)return false;memset(p,0,64);p[0]=2;p[1]=(uint8_t)(6-halves);return true;
}
void pmw3901_spi_auto_result(bool sent){++attempts;if(sent)--halves;}
int main(void){
 protocol_context_t context=reset_context(false,false);tick=5000;
 assert(!service_optical_failure(&context)&&attempts==0); /* disconnected */
 context.host_connected=true;context.result_count=1;
 assert(!service_optical_failure(&context)&&attempts==0); /* ACK first */
 context.result_count=0;context.register_job_active=true;
 assert(!service_optical_failure(&context)&&attempts==0);
 context.register_job_active=false;context.pwm_mode_pending=true;
 assert(!service_optical_failure(&context)&&attempts==0);
 context.pwm_mode_pending=false;
 assert(service_optical_failure(&context)&&attempts==1&&halves==1);
 tick+=19;assert(!service_optical_failure(&context));
 ++tick;tx_success=false;assert(service_optical_failure(&context)&&attempts==2&&halves==1);
 tick+=99;assert(!service_optical_failure(&context));
 ++tick;tx_success=true;assert(service_optical_failure(&context)&&attempts==3&&halves==0);
 tick+=20;assert(!service_optical_failure(&context));
 assert(context.state==HOST_FSM_DISARMED&&!context.lease_established&&heartbeat_attempts==0);
 puts("PASS automatic host diagnostic priority, 20ms pacing, failed-TX retention, 100ms backoff and unchanged control state");return 0;
}
