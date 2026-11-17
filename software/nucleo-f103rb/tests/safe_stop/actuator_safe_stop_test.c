#include <assert.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "actuator_task.h"
#include "host_protocol.h"
#include "sensor_task.h"
#include "motor_control.h"
#include "task.h"

static jmp_buf finished;
static TickType_t tick;
static host_safety_action_t safety_action;
static host_fsm_state_t state;
static bool stuck;
static bool fault_on_stop;
static bool inject_stop;
static bool fail_center_writes;
static bool missing_feedback;
static TickType_t center_after_tick;
static TickType_t stop_test_tick;
static unsigned torque_off_failures;
static unsigned center_writes;
static unsigned torque_on_calls;
static unsigned torque_off_calls;
static unsigned completed_stops;
static unsigned faults;
static TickType_t torque_off_tick;
static unsigned torque_off_queries, off_query_failures;
static bool verified_off;
static unsigned writes_at_first_successful_off;

TickType_t xTaskGetTickCount(void) { return tick; }

void vTaskDelayUntil(TickType_t *previous, TickType_t increment)
{
    tick = *previous + increment;
    *previous = tick;
    if (inject_stop && tick == 10U) {
        safety_action = HOST_SAFETY_ACTION_SAFE_STOP;
        state = fault_on_stop ? HOST_FSM_FAULT : HOST_FSM_DISARMED;
    }
    if (tick >= stop_test_tick) longjmp(finished, 1);
}

host_safety_action_t host_protocol_get_pending_safety_action(void) { return safety_action; }
host_fsm_state_t host_protocol_get_state(void) { return state; }
bool host_protocol_is_armed(void) { return state == HOST_FSM_ARMED; }
bool host_protocol_is_stop_requested(void)
{ return safety_action != HOST_SAFETY_ACTION_NONE || state == HOST_FSM_FAULT; }
void host_protocol_complete_safe_stop(void)
{
    assert(verified_off);
    ++completed_stops;
    safety_action = HOST_SAFETY_ACTION_NONE;
    if (state == HOST_FSM_FAULT) state = HOST_FSM_DISARMED;
}
void host_protocol_latch_fault(host_fault_reason_t reason)
{
    assert(reason == HOST_FAULT_SERVO);
    ++faults;
    state = HOST_FSM_FAULT;
}
void host_protocol_request_target_timeout(void) { assert(false); }

bool motor_device_validate_targets(const motor_target_set_t *targets)
{
    return targets != NULL;
}

motor_device_status_t motor_device_write_targets(const motor_target_set_t *targets)
{
    bool is_center = true;
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i)
        if (targets->motor[i].position != 2048U) is_center = false;
    if (is_center) {
        ++center_writes;
        if (fail_center_writes) return MOTOR_DEVICE_UART_ERROR;
    }
    return MOTOR_DEVICE_OK;
}

motor_device_status_t motor_device_set_torque(bool enabled)
{
    if (enabled) {
        assert(safety_action == HOST_SAFETY_ACTION_NONE);
        ++torque_on_calls;
    } else {
        ++torque_off_calls;
        torque_off_tick = tick;
        if (torque_off_calls <= torque_off_failures) return MOTOR_DEVICE_UART_ERROR;
        if (writes_at_first_successful_off == 0U)
            writes_at_first_successful_off = center_writes;
    }
    return MOTOR_DEVICE_OK;
}

bool motor_device_verify_torque_off(uint16_t *unknown)
{
    ++torque_off_queries;
    verified_off = torque_off_queries > off_query_failures;
    *unknown = verified_off ? 0U : 1U;
    return verified_off;
}

bool sensor_get_feedback_snapshot(motor_feedback_set_t *motor, imu_feedback_t *imu)
{
    if (missing_feedback) return false;
    memset(motor, 0, sizeof(*motor));
    memset(imu, 0, sizeof(*imu));
    motor->sequence = tick / 20U + 1U;
    motor->uptime_ms = tick;
    motor->valid_mask = YERTLE_ALL_MOTORS_MASK;
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i) {
        motor->motor[i].valid = true;
        motor->motor[i].position = stuck || tick < center_after_tick ? 2200 : 2048;
    }
    return true;
}

static void run_case(bool start_with_target, bool force_stall, bool stop_on_fault,
                     TickType_t centered_at, TickType_t run_until,
                     bool write_failure, bool no_feedback, unsigned off_failures, unsigned read_failures)
{
    tick = 0U;
    safety_action = start_with_target ? HOST_SAFETY_ACTION_NONE : HOST_SAFETY_ACTION_SAFE_STOP;
    state = start_with_target ? HOST_FSM_ARMED : HOST_FSM_DISARMED;
    stuck = force_stall;
    fault_on_stop = stop_on_fault;
    inject_stop = start_with_target;
    fail_center_writes = write_failure;
    missing_feedback = no_feedback;
    center_after_tick = centered_at;
    stop_test_tick = run_until;
    torque_off_failures = off_failures;
    center_writes = torque_on_calls = torque_off_calls = completed_stops = faults = 0U;
    torque_off_tick = 0U;
    off_query_failures = read_failures;
    torque_off_queries = 0U;
    verified_off = false;
    writes_at_first_successful_off = 0U;
    motor_control_init();
    if (start_with_target) {
        motor_target_set_t targets = { 0 };
        for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i)
            targets.motor[i].position = 2200U;
        assert(motor_control_submit_targets(&targets));
    }
    if (setjmp(finished) == 0) actuator_task(NULL);
}

int main(void)
{
    run_case(true, false, false, 70U, 250U, false, false, 0U, 0U);
    assert(torque_on_calls == 1U && center_writes > 0U);
    assert(torque_off_calls == 1U && torque_off_tick == 110U);
    assert(completed_stops == 1U && faults == 0U);

    /* A stall proceeds straight to OFF and never starts a second return. */
    run_case(true, true, false, 70U, 1750U, false, false, 0U, 0U);
    assert(torque_on_calls == 1U && center_writes > 0U);
    assert(torque_off_calls == 1U && faults == 0U && completed_stops == 1U);
    assert(state == HOST_FSM_DISARMED && center_writes == writes_at_first_successful_off);
    assert(torque_off_tick == 510U);

    run_case(false, false, false, 70U, 100U, false, false, 0U, 0U);
    assert(center_writes == 0U && torque_on_calls == 0U && torque_off_calls == 1U);
    assert(torque_off_tick == 0U && completed_stops == 1U);

    /* A stop already caused by FAULT follows the same return sequence. */
    run_case(true, false, true, 70U, 300U, false, false, 0U, 0U);
    assert(torque_on_calls == 1U && center_writes > 0U);
    assert(torque_off_calls == 1U && completed_stops == 1U && state == HOST_FSM_DISARMED);

    /* Three missed feedback opportunities cause OFF without new movement. */
    run_case(true, false, true, 70U, 200U, false, true, 0U, 0U);
    assert(torque_on_calls == 1U && center_writes == 0U);
    assert(torque_off_tick == 50U && completed_stops == 1U && state == HOST_FSM_DISARMED);

    /* The position backend keeps its three-attempt budget for return TX. */
    run_case(true, false, false, 70U, 300U, true, false, 0U, 0U);
    assert(center_writes == 3U && torque_on_calls == 1U && faults == 0U);
    assert(torque_off_calls == 1U && completed_stops == 1U);

    /* Failed OFF writes or readbacks must never re-enter the return stage. */
    run_case(true, false, true, 0U, 350U, false, false, 6U, 2U);
    assert(torque_on_calls == 1U && torque_off_calls == 9U && torque_off_queries == 3U);
    assert(completed_stops == 1U && center_writes == writes_at_first_successful_off);
    assert(state == HOST_FSM_DISARMED);

    run_case(true, true, true, 70U, 850U, false, false, 0U, 1000U);
    assert(torque_on_calls == 1U && torque_off_calls >= 10U && completed_stops == 0U);
    assert(state == HOST_FSM_FAULT && safety_action == HOST_SAFETY_ACTION_SAFE_STOP);
    return 0;
}

bool sensor_get_extended_feedback_snapshot(motor_feedback_set_t *m, imu_feedback_t *i, pmw3901_feedback_t *f){memset(f,0,sizeof *f);return sensor_get_feedback_snapshot(m,i);}
