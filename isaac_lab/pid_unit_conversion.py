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


if __name__ == '__main__':
    import json
    print(json.dumps(convert_joint_pd(), indent=2))
