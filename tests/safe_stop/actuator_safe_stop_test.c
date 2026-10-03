#include <assert.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "actuator_task.h"
#include "host_protocol.h"
#include "sensor_task.h"
#include "queue.h"
#include "task.h"

static jmp_buf finished;
static TickType_t tick;
static host_safety_action_t safety_action;
static host_fsm_state_t state;
static motor_target_set_t queued_targets;
static bool queued;
static bool stuck;
static bool fault_on_stop;
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
static TickType_t last_center_write_tick;
static unsigned writes_at_first_successful_off;

QueueHandle_t xQueueCreate(unsigned length, unsigned item_size)
{
    assert(length == 1U && item_size == sizeof(motor_target_set_t));
    return &queued_targets;
}

BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item)
{
    assert(queue == &queued_targets);
    queued_targets = *(const motor_target_set_t *)item;
    queued = true;
    return pdPASS;
}

BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t timeout)
{
    assert(queue == &queued_targets && timeout == 0U);
    if (!queued) return 0;
    *(motor_target_set_t *)item = queued_targets;
    queued = false;
    return pdTRUE;
}

BaseType_t xQueueReset(QueueHandle_t queue)
{
    assert(queue == &queued_targets);
    queued = false;
    return pdPASS;
}

TickType_t xTaskGetTickCount(void) { return tick; }

void vTaskDelayUntil(TickType_t *previous, TickType_t increment)
{
    tick = *previous + increment;
    *previous = tick;
    if (tick == 10U) {
        safety_action = HOST_SAFETY_ACTION_SAFE_STOP;
        if (fault_on_stop) state = HOST_FSM_FAULT;
    }
    if (tick >= stop_test_tick) longjmp(finished, 1);
}

host_safety_action_t host_protocol_get_pending_safety_action(void) { return safety_action; }
host_fsm_state_t host_protocol_get_state(void) { return state; }
bool host_protocol_is_armed(void) { return state == HOST_FSM_ARMED; }
void host_protocol_complete_safe_stop(void)
{
    ++completed_stops;
    safety_action = HOST_SAFETY_ACTION_NONE;
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
        last_center_write_tick = tick;
        if (fail_center_writes) return MOTOR_DEVICE_UART_ERROR;
    }
    return MOTOR_DEVICE_OK;
}

motor_device_status_t motor_device_set_torque(bool enabled)
{
    if (enabled) {
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
                     bool write_failure, bool no_feedback, unsigned off_failures)
{
    tick = 0U;
    safety_action = start_with_target ? HOST_SAFETY_ACTION_NONE : HOST_SAFETY_ACTION_SAFE_STOP;
    state = HOST_FSM_ARMED;
    queued = false;
    stuck = force_stall;
    fault_on_stop = stop_on_fault;
    fail_center_writes = write_failure;
    missing_feedback = no_feedback;
    center_after_tick = centered_at;
    stop_test_tick = run_until;
    torque_off_failures = off_failures;
    center_writes = torque_on_calls = torque_off_calls = completed_stops = faults = 0U;
    torque_off_tick = 0U;
    last_center_write_tick = 0U;
    writes_at_first_successful_off = 0U;
    assert(actuator_task_init());
    if (start_with_target) {
        motor_target_set_t targets = { 0 };
        for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i)
            targets.motor[i].position = 2200U;
        assert(actuator_submit_targets(&targets));
    }
    if (setjmp(finished) == 0) actuator_task(NULL);
}

int main(void)
{
    run_case(true, false, false, 70U, 250U, false, false, 0U);
    assert(torque_on_calls == 1U && center_writes > 0U);
    assert(torque_off_calls == 1U && torque_off_tick >= 110U);
    assert(completed_stops == 1U && faults == 0U);

    /* Initial stop fails, then both FAULT retries fail. Torque-off stays active. */
    run_case(true, true, false, 70U, 1750U, false, false, 0U);
    assert(torque_on_calls == 3U && center_writes > 0U);
    assert(torque_off_calls >= 10U && faults == 1U && completed_stops == 0U);
    assert(state == HOST_FSM_FAULT && center_writes == writes_at_first_successful_off);
    assert(last_center_write_tick < 1540U && torque_off_tick >= 1700U);

    run_case(false, false, false, 70U, 100U, false, false, 0U);
    assert(center_writes == 0U && torque_off_calls == 1U);
    assert(torque_off_tick == 0U && faults == 0U);

    /* FAULT recovery succeeds on the first try; no repeat after completion. */
    run_case(true, false, true, 70U, 300U, false, false, 0U);
    assert(torque_on_calls == 2U && center_writes > 0U);
    assert(torque_off_calls == 1U && completed_stops == 1U && state == HOST_FSM_FAULT);

    /* First FAULT try stalls, the second succeeds with fresh feedback. */
    run_case(true, false, true, 600U, 850U, false, false, 0U);
    assert(torque_on_calls == 3U && torque_off_calls == 1U && completed_stops == 1U);
    assert(torque_off_tick >= 650U && state == HOST_FSM_FAULT);

    /* No feedback cannot be mistaken for a centered pose. */
    run_case(true, false, true, 70U, 1250U, false, true, 0U);
    assert(torque_on_calls == 3U && torque_off_calls >= 10U && completed_stops == 0U);
    assert(center_writes == writes_at_first_successful_off);

    /* Three failed goal writes per attempt: initial + two retries, then off-only. */
    run_case(true, false, false, 70U, 300U, true, false, 0U);
    assert(center_writes == 9U && torque_on_calls == 1U && faults == 1U);
    assert(torque_off_calls >= 10U && completed_stops == 0U);

    /* A stop includes torque-off: six failed OFF writes exhaust both FAULT tries. */
    run_case(true, false, true, 0U, 350U, false, false, 6U);
    assert(torque_on_calls == 3U && torque_off_calls > 6U && completed_stops == 0U);
    assert(center_writes == writes_at_first_successful_off && state == HOST_FSM_FAULT);
    return 0;
}
