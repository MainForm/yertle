#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

/* Real actuator loop and native-position service; mocked scheduling and I/O. */
#include "../../firmware/src/actuator_task.c"
#include "../../firmware/src/motor_control.c"

static jmp_buf finished;
static TickType_t tick;
static bool valid_targets;
static bool armed;
static bool inject_target;
static TickType_t inject_at;
static unsigned writes;
static unsigned torque_on_calls;
static motor_target_set_t targets;

TickType_t xTaskGetTickCount(void) { return tick; }
void vTaskDelayUntil(TickType_t *previous, TickType_t increment)
{
    tick = *previous + increment;
    *previous = tick;
    if (inject_target && tick == inject_at) {
        /* Equal positions still constitute a new host command. */
        const bool accepted = motor_control_submit_targets(&targets);
        assert(accepted == (armed && valid_targets));
        inject_target = false;
    }
    assert(writes < 200U); /* Prevent a broken watchdog hanging the test. */
}
host_safety_action_t host_protocol_get_pending_safety_action(void)
{ return HOST_SAFETY_ACTION_NONE; }
host_fsm_state_t host_protocol_get_state(void)
{ return armed ? HOST_FSM_ARMED : HOST_FSM_DISARMED; }
bool host_protocol_is_armed(void) { return armed; }
bool host_protocol_is_stop_requested(void) { return false; }
void host_protocol_complete_safe_stop(void) { assert(false); }
void host_protocol_latch_fault(host_fault_reason_t reason)
{ (void)reason; assert(false); }
void host_protocol_request_target_timeout(void) { longjmp(finished, 1); }
bool motor_device_validate_targets(const motor_target_set_t *value)
{ return value != NULL && valid_targets; }
motor_device_status_t motor_device_write_targets(const motor_target_set_t *value)
{
    assert(value->motor[0].position == 2200U);
    ++writes;
    return MOTOR_DEVICE_OK;
}
motor_device_status_t motor_device_set_torque(bool enabled)
{ if (enabled) ++torque_on_calls; return MOTOR_DEVICE_OK; }
bool sensor_get_feedback_snapshot(motor_feedback_set_t *motor, imu_feedback_t *imu)
{ (void)motor; (void)imu; return false; }

static void reset(void)
{
    tick = 100U;
    valid_targets = armed = true;
    inject_target = false;
    writes = torque_on_calls = 0U;
    memset(&targets, 0, sizeof(targets));
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i)
        targets.motor[i].position = 2200U;
    motor_control_init();
    assert(motor_control_submit_targets(&targets));
    motor_position_target_t saved_target;
    assert(motor_control_read_targets(&saved_target) && saved_target.accepted_ms == 100U);
}
static void run_until_timeout(TickType_t start, TickType_t expected)
{
    tick = start;
    if (setjmp(finished) == 0) actuator_task(NULL);
    assert(tick == expected);
}

int main(void)
{
    motor_position_target_t saved_target;
    reset();
    run_until_timeout(400U, 600U);
    assert(writes == 20U); /* Deadline stays at reception+500, not first service+500. */

    reset();
    run_until_timeout(700U, 700U);
    assert(writes == 0U && torque_on_calls == 0U); /* Stale accepted command. */

    reset();
    inject_target = true;
    inject_at = 590U;
    run_until_timeout(400U, 1090U);
    assert(motor_control_read_targets(&saved_target) && saved_target.accepted_ms == 590U);

    reset();
    tick = 350U;
    assert(motor_control_submit_targets(&targets)); /* Latest-target overwrite. */
    run_until_timeout(400U, 850U);

    reset();
    inject_target = true;
    inject_at = 590U;
    valid_targets = false;
    run_until_timeout(400U, 600U);
    assert(motor_control_read_targets(&saved_target) && saved_target.accepted_ms == 100U);

    reset();
    armed = false;
    tick = 590U;
    assert(!motor_control_submit_targets(&targets));
    assert(motor_control_read_targets(&saved_target) && saved_target.accepted_ms == 100U);

    reset();
    tick = UINT32_MAX - 199U;
    assert(motor_control_submit_targets(&targets));
    run_until_timeout(100U, 300U); /* Deadline crosses the tick-counter wrap. */
    puts("targets: reception timestamp, delayed service, expiry, overwrite, rejection and tick wrap passed");
    return 0;
}

bool sensor_get_extended_feedback_snapshot(motor_feedback_set_t *m, imu_feedback_t *i, pmw3901_feedback_t *f){memset(f,0,sizeof *f);return sensor_get_feedback_snapshot(m,i);}
