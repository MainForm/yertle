#undef NDEBUG
#include <assert.h>
#include "pwm_contract.h"

int main(void)
{
    assert(YPWM_REGISTER_MODE_CAPABILITY==0x2000U);
    assert(ypwm_expected_servo_mode(YPWM_POSITION)==0 && ypwm_expected_servo_mode(YPWM_PID)==2);
    assert(YPWM_DUTY_LIMIT==1000 && ypwm_servo_word(-1000)==0x07e8 && ypwm_servo_word(1000)==0x03e8);
    return 0;
}
