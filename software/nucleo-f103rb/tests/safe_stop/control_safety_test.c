#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

/* Exercise both production tasks and their shared safety gate. */
#include "../../firmware/src/host_protocol.c"
#include "../../firmware/src/actuator_task.c"

static jmp_buf finished;
static TickType_t tick, tx_elapsed, stop_at;
static queued_target_t queue_item;
static bool queued, flood, garbage, fault_on_read, expire_during_motion;
static unsigned enqueue_calls, torque_on_calls, torque_off_calls, center_writes;
static unsigned rx_reads, tx_calls, delay_calls;
static uint8_t rx_wire[80];
static size_t rx_length, rx_index;
static protocol_context_t context;
static unsigned stalled_id;
static bool changing_feedback, invalid_feedback;

QueueHandle_t xQueueCreate(unsigned length, unsigned size)
{ assert(length == 1U && size == sizeof(queued_target_t)); return &queue_item; }
BaseType_t xQueueOverwrite(QueueHandle_t queue, const void *item)
{ assert(queue == &queue_item); queue_item = *(const queued_target_t *)item; queued = true; ++enqueue_calls; return pdPASS; }
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t timeout)
{ assert(queue == &queue_item && timeout == 0U); if (!queued) return 0; *(queued_target_t *)item = queue_item; queued = false; return pdPASS; }
BaseType_t xQueueReset(QueueHandle_t queue)
{ assert(queue == &queue_item); queued = false; return pdPASS; }
TickType_t xTaskGetTickCount(void) { return tick; }
void vTaskDelay(TickType_t ticks)
{ tick += ticks; ++delay_calls; longjmp(finished, 1); }
void vTaskDelayUntil(TickType_t *previous, TickType_t increment)
{
    tick = *previous + increment; *previous = tick;
    if (expire_during_motion && tick == 10U) {
        context.last_host_heartbeat_tick = tick - pdMS_TO_TICKS(1000U);
        expire_host_link(&context);
    }
    if (expire_during_motion) service_safety_state(&context);
    if (tick >= stop_at) longjmp(finished, 1);
}
bool host_uart_write_bytes(const uint8_t *data, size_t length)
{
    uint8_t raw[YERTLE_PROTOCOL_MAX_RAW], type;
    uint16_t sequence;
    size_t raw_length;
    assert(data[0] == 0U && data[length - 1U] == 0U);
    assert(cobs_decode(data + 1U, length - 2U, raw, &raw_length));
    assert(validate_frame(raw, raw_length, &type, &sequence));
    if (type == MESSAGE_EVENT && raw[7] == EVENT_STATE_CHANGED &&
        (raw[9] == HOST_FSM_DISARMED || raw[9] == HOST_FSM_FAULT) &&
        raw[10] != STATE_CAUSE_SAFE_STOP_COMPLETE) {
        /* Represents observing the shared gate while UART TX is preempted. */
        assert(pending_safety_action == HOST_SAFETY_ACTION_SAFE_STOP);
        assert(host_protocol_is_stop_requested());
    }
    ++tx_calls; tick += tx_elapsed;
    return true;
}
bool host_uart_try_read(uint8_t *byte, bool *discontinuity)
{
    *discontinuity = false;
    if (!flood) return false;
    if (fault_on_read) { fault_on_read = false; host_protocol_latch_fault(HOST_FAULT_SERVO); }
    ++rx_reads;
    assert(rx_reads <= 256U); /* The old unbounded RX loop fails here. */
    if (garbage) *byte = 1U;
    else { *byte = rx_wire[rx_index++]; if (rx_index == rx_length) rx_index = 0U; }
    return true;
}
bool sensor_take_position_offsets(motor_position_offsets_t *result)
{ (void)result; return false; }
bool sensor_request_position_offsets(uint16_t sequence)
{ (void)sequence; return false; }
bool sensor_request_motor_register(uint16_t sequence, uint8_t id, uint8_t address, uint8_t length)
{ (void)sequence; (void)id; (void)address; (void)length; return false; }
bool sensor_take_motor_register(motor_register_result_t *result)
{ (void)result; return false; }
bool sensor_request_motor_register_write(uint16_t sequence, const motor_register_write_request_t *request)
{ (void)sequence; (void)request; return false; }
bool sensor_take_motor_register_write(motor_register_write_result_t *result)
{ (void)result; return false; }
uint16_t motor_device_register_lock_uncertain_mask(void) { return 0U; }
void uart_diagnostics_send(void *opaque, uart_diag_sender_t send)
{ (void)opaque; (void)send; }
bool motor_device_validate_targets(const motor_target_set_t *targets)
{ return targets != NULL; }
motor_device_status_t motor_device_write_targets(const motor_target_set_t *targets)
{ if (targets->motor[0].position == 2048U) ++center_writes; return MOTOR_DEVICE_OK; }
motor_device_status_t motor_device_set_torque(bool enabled)
{ if (enabled) ++torque_on_calls; else { assert(center_writes > 0U); ++torque_off_calls; } return MOTOR_DEVICE_OK; }
bool sensor_get_feedback_snapshot(motor_feedback_set_t *motor, imu_feedback_t *imu)
{
    memset(motor, 0, sizeof(*motor)); memset(imu, 0, sizeof(*imu));
    motor->valid_mask = YERTLE_ALL_MOTORS_MASK;
    motor->sequence = tick / 20U + 1U; motor->uptime_ms = tick;
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i) {
        motor->motor[i].valid = true;
        motor->motor[i].position = (int16_t)(changing_feedback ? 2200U - tick / 100U * 12U : 2048U);
    }
    if (stalled_id) motor->motor[stalled_id - 1U].position = 2200;
    if (invalid_feedback) motor->motor[11].servo_error = 4U;
    return true;
}
static size_t make_request(uint8_t *raw, uint8_t type, uint16_t payload)
{
    const size_t size = YERTLE_PROTOCOL_HEADER_SIZE + payload + 2U;
    memset(raw, 0, size); raw[0] = 1U; raw[1] = type; raw[3] = 1U; raw[5] = (uint8_t)payload;
    const uint16_t crc = crc16_ccitt(raw, size - 2U);
    raw[size - 2U] = (uint8_t)crc; raw[size - 1U] = (uint8_t)(crc >> 8U);
    return size;
}
static void reset(void)
{
    memset(&context, 0, sizeof(context)); context.state = HOST_FSM_ARMED;
    context.host_connected = context.lease_established = true;
    published_state = HOST_FSM_ARMED;
    requested_fault_reason = HOST_FAULT_NONE; requested_target_timeout = false;
    requested_unsafe_reason = HOST_UNSAFE_NONE; safe_stop_completed = false;
    pending_safety_action = HOST_SAFETY_ACTION_NONE;
    tick = tx_elapsed = 0U; queued = flood = garbage = fault_on_read = expire_during_motion = false;
    enqueue_calls = torque_on_calls = torque_off_calls = center_writes = 0U;
    rx_reads = tx_calls = delay_calls = 0U;
    rx_index = 0U; stalled_id = 0U; changing_feedback = invalid_feedback = false;
    target_queue = NULL; assert(actuator_task_init());
    uint8_t raw[YERTLE_PROTOCOL_MAX_RAW];
    const size_t size = make_request(raw, MESSAGE_GET_FSM_STATE, 0U);
    rx_wire[0] = 0U; rx_length = cobs_encode(raw, size, rx_wire + 1U) + 2U;
    rx_wire[rx_length - 1U] = 0U;
}
int main(void)
{
    uint8_t raw[YERTLE_PROTOCOL_MAX_RAW];
    reset(); tick = 1500U;
    size_t size = make_request(raw, MESSAGE_SET_JOINT_TARGETS, YERTLE_MOTOR_COUNT * 5U);
    handle_frame(&context, raw, size);
    assert(enqueue_calls == 0U && !context.host_connected && context.state == HOST_FSM_DISARMED);

    reset(); host_protocol_request_target_timeout();
    motor_target_set_t targets = { 0 }; targets.motor[0].position = 2200U;
    assert(!actuator_submit_targets(&targets));
    handle_frame(&context, raw, size);
    assert(enqueue_calls == 0U && context.state == HOST_FSM_DISARMED && host_protocol_is_stop_requested());
    reset(); host_protocol_latch_fault(HOST_FAULT_SERVO);
    assert(!actuator_submit_targets(&targets));
    handle_frame(&context, raw, size);
    assert(enqueue_calls == 0U && context.state == HOST_FSM_FAULT);

    /* Actual actuator keeps torque on through center return on heartbeat expiry. */
    reset(); assert(actuator_submit_targets(&targets));
    expire_during_motion = true; stop_at = 200U;
    if (setjmp(finished) == 0) actuator_task(NULL);
    assert(torque_on_calls == 1U && center_writes > 0U && torque_off_calls == 1U);

    /* Continuous valid queries and delimiter-free input cannot starve FSM. */
    for (unsigned mode = 0U; mode < 2U; ++mode) {
        reset(); flood = fault_on_read = true; garbage = mode != 0U;
        if (setjmp(finished) == 0) host_protocol_task(NULL);
        assert(delay_calls == 1U && rx_reads <= YERTLE_RX_BYTES_PER_CYCLE);
        assert(published_state == HOST_FSM_FAULT && host_protocol_is_stop_requested());
    }
    reset(); flood = true; process_host_rx(&context);
    assert(tx_calls == YERTLE_RX_FRAMES_PER_CYCLE);
    reset(); flood = true; tx_elapsed = 9U; process_host_rx(&context);
    assert(tx_calls == 1U); /* Time budget checked between complete replies. */

    /* Every individual motor can independently stop an otherwise moving return. */
    for (stalled_id = 1U; stalled_id <= YERTLE_MOTOR_COUNT; ++stalled_id) {
        safe_stop_context_t stop = { 0 }; changing_feedback = true;
        begin_stop_attempt(&stop, 0U, true, true);
        for (tick = 0U; tick <= 400U; tick += 100U) {
            assert(!center_reached(&stop, tick)); assert(!stop_has_stalled(&stop, tick));
        }
        assert(stop_has_stalled(&stop, 500U));
        assert(stop.last_progress_tick[stalled_id - 1U] == 0U);
    }
    /* Already-centered motors remain healthy while another is progressing. */
    reset(); safe_stop_context_t stop = { 0 };
    begin_stop_attempt(&stop, 0U, true, true);
    for (tick = 0U; tick <= 1000U; tick += 100U) {
        center_reached(&stop, tick); assert(!stop_has_stalled(&stop, tick));
    }
    reset(); begin_stop_attempt(&stop, 0U, true, true); invalid_feedback = true; tick = 100U;
    assert(!center_reached(&stop, tick));
    for (unsigned i = 0U; i < YERTLE_MOTOR_COUNT; ++i) assert(stop.last_progress_tick[i] == 0U);
    puts("control safety: ordered stop, stale/pending target rejection, RX budgets and per-axis stall passed");
    return 0;
}
