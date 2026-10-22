#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

/* Real queue item and actuator loop; mocked scheduling and device I/O. */
#include "../../firmware/src/actuator_task.c"

static jmp_buf finished;
static TickType_t tick;
static queued_target_t queue_item;
static bool queued;
static bool accept_queue;
static bool valid_targets;
static bool armed;
static bool inject_target;
static TickType_t inject_at;
static unsigned writes;
static unsigned torque_on_calls;
static motor_target_set_t targets;

QueueHandle_t xQueueCreate(unsigned length, unsigned item_size)
{
    assert(length == 1U && item_size == sizeof(queued_target_t));
    return &queue_item;
}
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item)
{
    assert(queue == &queue_item);
    if (!accept_queue) return 0;
    queue_item = *(const queued_target_t *)item;
    queued = true;
    return pdPASS;
}
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t timeout)
{
    assert(queue == &queue_item && timeout == 0U);
    if (!queued) return 0;
    *(queued_target_t *)item = queue_item;
    queued = false;
    return pdPASS;
}
BaseType_t xQueueReset(QueueHandle_t queue)
{ assert(queue == &queue_item); queued = false; return pdPASS; }
TickType_t xTaskGetTickCount(void) { return tick; }
void vTaskDelayUntil(TickType_t *previous, TickType_t increment)
{
    tick = *previous + increment;
    *previous = tick;
    if (inject_target && tick == inject_at) {
        /* Equal positions still constitute a new host command. */
        const bool accepted = actuator_submit_targets(&targets);
        assert(accepted == (armed && accept_queue && valid_targets));
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
    target_queue = NULL;
    tick = 100U;
    queued = false;
    accept_queue = valid_targets = armed = true;
    inject_target = false;
    writes = torque_on_calls = 0U;
    memset(&targets, 0, sizeof(targets));
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i)
        targets.motor[i].position = 2200U;
    assert(actuator_task_init());
    assert(actuator_submit_targets(&targets));
    assert(queue_item.accepted_tick == 100U);
}
static void run_until_timeout(TickType_t start, TickType_t expected)
{
    tick = start;
    if (setjmp(finished) == 0) actuator_task(NULL);
    assert(tick == expected);
}

int main(void)
{
    reset();
    run_until_timeout(400U, 600U);
    assert(writes == 20U); /* Deadline stays at enqueue+500, not dequeue+500. */

    reset();
    run_until_timeout(700U, 700U);
    assert(writes == 0U && torque_on_calls == 0U); /* Stale queued command. */

    reset();
    inject_target = true;
    inject_at = 590U;
    run_until_timeout(400U, 1090U);
    assert(queue_item.accepted_tick == 590U);

    reset();
    tick = 350U;
    assert(actuator_submit_targets(&targets)); /* Latest-only queue overwrite. */
    run_until_timeout(400U, 850U);

    reset();
    inject_target = true;
    inject_at = 590U;
    accept_queue = false;
    run_until_timeout(400U, 600U);
    assert(queue_item.accepted_tick == 100U);

    reset();
    inject_target = true;
    inject_at = 590U;
    valid_targets = false;
    run_until_timeout(400U, 600U);
    assert(queue_item.accepted_tick == 100U);

    reset();
    armed = false;
    tick = 590U;
    assert(!actuator_submit_targets(&targets));
    assert(queue_item.accepted_tick == 100U);

    reset();
    tick = UINT32_MAX - 199U;
    assert(actuator_submit_targets(&targets));
    run_until_timeout(100U, 300U); /* Deadline crosses the tick-counter wrap. */
    puts("targets: enqueue timestamp, delayed dequeue, expiry, overwrite, rejection and tick wrap passed");
    return 0;
}
