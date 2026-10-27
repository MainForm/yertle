"""Unit conversion only; never changes a motor/controller or calibrated mapping."""
import math

MOTOR_RESPONSE_GAIN = 1.0
ENCODER_STEPS_PER_REVOLUTION = 4096
MODEL_STALL_TORQUE_NM = 1.28
MODEL_FREE_SPEED_RAD_S = 2.3


def convert_joint_pd(kp_nm_per_rad=25.0, kd_nm_s_per_rad=0.5):
    """Scalar 1:1 axis, identical reference, unsaturated operation.

    This does not make a coupled-motor controller, a filtered reference or a
    sampled explicit PID dynamically identical to native implicit joint PD.
    """
    step_rad = math.tau / ENCODER_STEPS_PER_REVOLUTION
    nm_per_pwm_raw = MODEL_STALL_TORQUE_NM * MOTOR_RESPONSE_GAIN / 1000.0
    nm_per_step = kp_nm_per_rad * step_rad
    nm_s_per_step = kd_nm_s_per_rad * step_rad
    passive_damping = MODEL_STALL_TORQUE_NM * MOTOR_RESPONSE_GAIN / MODEL_FREE_SPEED_RAD_S
    return {
        'response_gain': MOTOR_RESPONSE_GAIN,
        'rad_per_raw_step': step_rad,
        'Nm_per_PWM_raw': nm_per_pwm_raw,
        'P_Nm_per_position_raw': nm_per_step,
        'I': 0.0,
        'D_Nm_per_velocity_raw_s': nm_s_per_step,
        'P_PWM_raw_per_position_raw': nm_per_step / nm_per_pwm_raw,
        'D_PWM_raw_per_velocity_raw_s_ideal': nm_s_per_step / nm_per_pwm_raw,
        'DC_passive_damping_Nm_s_per_rad': passive_damping,
        'D_PWM_raw_per_velocity_raw_s_DC_total_match':
            (kd_nm_s_per_rad-passive_damping)*step_rad/nm_per_pwm_raw,
    }


def convert_raw_pid(kp=5.0, ki=3.0, kd=0.10):
    """Raw/PWM controller -> encoder-motor-angle radians / commanded Nm.

    The angle is s*(calibrated encoder raw-2048); shin linkage stays in the
    existing position conversion. These are not twelve independent joint gains.
    """
    s = math.tau / ENCODER_STEPS_PER_REVOLUTION
    k = MODEL_STALL_TORQUE_NM * MOTOR_RESPONSE_GAIN / 1000.0
    factor = k/s
    return dict(P_Nm_per_rad=kp*factor, I_Nm_per_rad_s=ki*factor,
                D_Nm_s_per_rad=kd*factor, integral_limit_Nm=100*k,
                commanded_torque_limit_Nm=800*k, rad_per_raw=s, Nm_per_PWM_raw=k,
                motor_DC_passive_damping=MODEL_STALL_TORQUE_NM/MODEL_FREE_SPEED_RAD_S)


class AngleTorquePID:
    """Independent radians/Nm implementation with original integration limits.

    The original encoder reference is converted to radians internally. No ramp,
    filter, physical-motor parameter or calibrated mapping is changed here.
    """
    def __init__(self, raw_gains, measured_raw, integral_raw):
        import numpy as np
        self.np=np
        factors=[convert_raw_pid(*row) for row in raw_gains]
        self.s=factors[0]['rad_per_raw'];self.k=factors[0]['Nm_per_PWM_raw']
        self.p=np.array([r['P_Nm_per_rad'] for r in factors])[:,None]
        self.i=np.array([r['I_Nm_per_rad_s'] for r in factors])[:,None]
        self.d=np.array([r['D_Nm_s_per_rad'] for r in factors])[:,None]
        self.integral=np.array(integral_raw,dtype=float)*self.k
        self.previous=(np.array(measured_raw,dtype=float)-2048)*self.s
        self.icap=factors[0]['integral_limit_Nm'];self.limit=factors[0]['commanded_torque_limit_Nm']

    def step(self, reference_raw, measured_raw, dt=.02):
        np=self.np
        reference=(np.asarray(reference_raw)-2048)*self.s
        measured=(np.asarray(measured_raw)-2048)*self.s
        error=reference-measured;velocity=(measured-self.previous)/dt
        candidate=np.clip(self.integral+self.i*error*dt,-self.icap,self.icap)
        p=self.p*error;d=-self.d*velocity
        requested=p+d+candidate
        windup=((requested>self.limit)&(error>0))|((requested<-self.limit)&(error<0))
        self.integral=np.where(windup,self.integral,candidate)
        self.previous=measured
        self.terms=np.stack([error,p,self.integral,d],axis=-1)
        return np.clip(p+self.integral+d,-self.limit,self.limit)


if __name__ == '__main__':
    import json
    print(json.dumps(convert_joint_pd(), indent=2))
