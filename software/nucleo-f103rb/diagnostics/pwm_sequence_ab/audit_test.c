#undef NDEBUG
/* Reuse existing production-control stubs, replacing only the motor behavior.
 * This is a hypothetical motor model, not a real-device result. */
#define main existing_control_tests_main
#define motor_device_set_torque prior_set_torque
#define motor_device_write_pwm prior_write_pwm
#define motor_device_verify_torque_off prior_verify_off
#define motor_device_verify_torque_off_detailed prior_verify_off_detailed
#include "../../tests/pwm/control_test.c"
#undef main
#undef motor_device_set_torque
#undef motor_device_write_pwm
#undef motor_device_verify_torque_off
#undef motor_device_verify_torque_off_detailed
static bool model_auto_enable;
static uint16_t model_on;
motor_device_status_t motor_device_set_torque(bool on){
    assert(!on);model_on=0;return prior_set_torque(on);
}
motor_device_status_t motor_device_write_pwm(uint16_t mask,const int16_t values[12]){
    for(unsigned i=0;i<12;++i)assert(values[i]==0);
    if(model_auto_enable)model_on=mask;
    return prior_write_pwm(mask,values);
}
bool motor_device_verify_torque_off(uint16_t *unknown){*unknown=model_on;return !model_on;}
bool motor_device_verify_torque_off_detailed(uint16_t *unknown,motor_verify_report_t *r){
    *r=motor_verify_initial_report();*unknown=model_on;
    if(model_on){r->motor_id=1;r->address=0x28;r->expected=0;r->actual=1;r->io_status=0;}
    return !model_on;
}
motor_register_status_t motor_device_read_register(uint8_t id,uint8_t address,uint8_t length,motor_register_data_t *r){
    assert(!critical&&id>=1&&id<=12&&address==0x28&&length==1);
    memset(r,0,sizeof *r);r->data[0]=(model_on&(1U<<(id-1)))?1:0;return MOTOR_REGISTER_OK;
}
static void run_case(bool automatic){
    model_auto_enable=automatic;model_on=0;reset();stage(321,YPWM_ALL_MASK);
    assert(prepare(321,YPWM_ALL_MASK)==0&&pwm_control_service());
    uint8_t d[64],w[64];bool has;
    for(unsigned step=1;step<=3;++step){
        assert(pwm_control_mode_diagnostics(d,w,&has,false)&&!has&&d[0]==6&&d[1]==1&&d[2]==step);
        assert(ypwm_u16(d+18)==YPWM_ALL_MASK&&d[16]==MOTOR_DEVICE_OK);
        const uint16_t expected_on=automatic&&step==2?YPWM_ALL_MASK:0;
        assert(ypwm_u16(d+22)==expected_on&&ypwm_u16(d+20)==(uint16_t)(YPWM_ALL_MASK^expected_on));
    }
    assert(!model_on&&phase()==YPWM_READY&&!pwm_control_can_arm());
    motor_target_set_t g=goals(YPWM_ALL_MASK);assert(!pwm_control_submit_positions(&g));
    assert(on_count==0&&write_count==1);
    printf("PASS hypothetical auto_enable=%d: A=%s, B=OFF; ARM/position goals blocked\n",automatic,automatic?"ON":"OFF");
}
int main(void){run_case(false);run_case(true);return 0;}
