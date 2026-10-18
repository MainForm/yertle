#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../../firmware/src/host_protocol.c"

static TickType_t tick, tx_elapsed, accepted_tick;
static bool tx_success;
static unsigned tx_calls, submitted, rx_reads;
static uint8_t last_wire[80];
static size_t last_length;
uint32_t xTaskGetTickCount(void) { return tick; }
void vTaskDelay(uint32_t ticks) { tick += ticks; }
bool host_uart_write_bytes(const uint8_t *data, size_t size)
{
    assert(size <= sizeof(last_wire));
    memcpy(last_wire, data, size); last_length = size;
    ++tx_calls; tick += tx_elapsed;
    return tx_success;
}
bool host_uart_try_read(uint8_t *byte, bool *discontinuity)
{ (void)byte; *discontinuity = false; ++rx_reads; return false; }
bool sensor_take_position_offsets(motor_position_offsets_t *value)
{ (void)value; return false; }
bool sensor_request_position_offsets(uint16_t sequence)
{ (void)sequence; return false; }
bool sensor_get_feedback_snapshot(motor_feedback_set_t *motor, imu_feedback_t *imu)
{ (void)motor; (void)imu; return false; }
bool motor_device_validate_targets(const motor_target_set_t *targets)
{ (void)targets; return true; }
bool actuator_submit_targets(const motor_target_set_t *targets)
{ (void)targets; ++submitted; accepted_tick = tick; return true; }
void uart_diagnostics_send(void *context, uart_diag_sender_t send)
{ (void)context; (void)send; }

static protocol_context_t reset(void)
{
    protocol_context_t context = { 0 };
    context.state = HOST_FSM_ARMED;
    context.host_connected = context.lease_established = true;
    published_state = HOST_FSM_ARMED;
    requested_fault_reason = HOST_FAULT_NONE; requested_target_timeout = false;
    pending_safety_action = HOST_SAFETY_ACTION_NONE; safe_stop_completed = false;
    tick = tx_elapsed = accepted_tick = 0U; tx_success = true;
    tx_calls = submitted = rx_reads = 0U; last_length = 0U;
    return context;
}
static void target(protocol_context_t *context, uint16_t sequence)
{
    uint8_t raw[YERTLE_PROTOCOL_HEADER_SIZE + YERTLE_MOTOR_COUNT * 5U + 2U] = { 0 };
    raw[0] = 1U; raw[1] = MESSAGE_SET_JOINT_TARGETS;
    raw[3] = (uint8_t)sequence; raw[4] = (uint8_t)(sequence >> 8U);
    raw[5] = YERTLE_MOTOR_COUNT * 5U;
    const uint16_t crc = crc16_ccitt(raw, sizeof(raw) - 2U);
    raw[sizeof(raw) - 2U] = (uint8_t)crc; raw[sizeof(raw) - 1U] = (uint8_t)(crc >> 8U);
    handle_frame(context, raw, sizeof(raw));
}
static void check_reply(uint8_t expected_type, uint16_t request, uint8_t code)
{
    uint8_t raw[YERTLE_PROTOCOL_MAX_RAW], type;
    uint16_t sequence; size_t size;
    assert(last_wire[0] == 0U && last_wire[last_length - 1U] == 0U);
    assert(cobs_decode(last_wire + 1U, last_length - 2U, raw, &size));
    assert(validate_frame(raw, size, &type, &sequence));
    assert(type == expected_type && size == 13U);
    assert(raw[7] == MESSAGE_SET_JOINT_TARGETS && raw[8] == (uint8_t)request &&
           raw[9] == (uint8_t)(request >> 8U) && raw[10] == code);
}
int main(void)
{
    protocol_context_t context = reset();
    target(&context, 87U);
    assert(submitted == 1U && tx_calls == 1U && context.result_count == 0U);
    check_reply(MESSAGE_ACK, 87U, 0U);

    context = reset(); tx_success = false; tx_elapsed = 20U;
    target(&context, 0x1234U);
    assert(submitted == 1U && accepted_tick == 0U && context.result_count == 1U);
    uint8_t original[80]; const size_t length = last_length;
    memcpy(original, last_wire, length);
    tick = 29U; service_result_tx(&context); assert(tx_calls == 1U);
    tick = 30U; tx_success = true; service_result_tx(&context);
    assert(tx_calls == 2U && context.result_count == 0U);
    assert(last_length == length && memcmp(last_wire, original, length) == 0);
    assert(submitted == 1U && accepted_tick == 0U && context.last_host_heartbeat_tick == 0U);
    assert(context.result_tx_failures == 1U && context.result_retry_attempts == 1U &&
           context.result_retry_successes == 1U && context.result_dropped == 0U);

    /* NACK also retries unchanged, without accepting a rejected target. */
    context = reset(); context.state = HOST_FSM_DISARMED; tx_success = false;
    target(&context, 9U); check_reply(MESSAGE_NACK, 9U, ERROR_INVALID_STATE);
    tick = 10U; tx_success = true; service_result_tx(&context);
    check_reply(MESSAGE_NACK, 9U, ERROR_INVALID_STATE);
    assert(submitted == 0U && context.result_count == 0U);

    /* Permanent failure is bounded, not a spin loop or a new watchdog lease. */
    context = reset(); tx_success = false;
    target(&context, 1U);
    for (unsigned attempt = 1U; attempt < YERTLE_RESULT_MAX_ATTEMPTS; ++attempt) {
        tick = attempt * YERTLE_RESULT_RETRY_MS; service_result_tx(&context);
    }
    service_result_tx(&context);
    assert(tx_calls == YERTLE_RESULT_MAX_ATTEMPTS && context.result_count == 0U);
    assert(context.result_dropped == 1U && submitted == 1U);

    context = reset(); tx_success = false; target(&context, 1U);
    tick = YERTLE_RESULT_MAX_AGE_MS; tx_success = true; service_result_tx(&context);
    assert(tx_calls == 1U && context.result_count == 0U && context.result_dropped == 1U);

    /* FIFO: new responses do not overtake or replace the failed result. */
    context = reset(); tx_success = false; target(&context, 1U); target(&context, 2U);
    assert(context.result_count == 2U && tx_calls == 1U && submitted == 2U);
    tick = 10U; tx_success = true; service_result_tx(&context);
    check_reply(MESSAGE_ACK, 1U, 0U); service_result_tx(&context);
    check_reply(MESSAGE_ACK, 2U, 0U); assert(context.result_count == 0U);

    context = reset(); tx_success = false;
    for (unsigned i = 0U; i < YERTLE_RESULT_QUEUE_CAPACITY; ++i) target(&context, (uint16_t)i);
    assert(context.result_count == YERTLE_RESULT_QUEUE_CAPACITY);
    process_host_rx(&context); assert(rx_reads == 0U);
    host_protocol_latch_fault(HOST_FAULT_SERVO); service_safety_state(&context);
    assert(context.state == HOST_FSM_FAULT && host_protocol_is_stop_requested());
    tick = 1000U; expire_host_link(&context); assert(!context.host_connected);
    service_result_tx(&context); process_host_rx(&context); assert(rx_reads == 1U);

    /* Both retry interval and expiry are wrap-safe. */
    context = reset(); tick = UINT32_MAX - 5U; tx_success = false;
    send_result(&context, MESSAGE_ACK, MESSAGE_SET_JOINT_TARGETS, 77U, 0U);
    tick = 3U; service_result_tx(&context); assert(tx_calls == 1U);
    tick = 4U; tx_success = true; service_result_tx(&context); assert(tx_calls == 2U);
    context = reset(); tick = UINT32_MAX - 5U; tx_success = false;
    send_result(&context, MESSAGE_ACK, MESSAGE_SET_JOINT_TARGETS, 77U, 0U);
    tick = 244U; service_result_tx(&context); assert(context.result_dropped == 1U);
    puts("result TX: ACK/NACK retries, exact frame, no request replay, bounded FIFO and safety passed");
    return 0;
}
