#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Exercise the production frame handler and link service with a mock UART. */
#include "../../firmware/src/host_protocol.c"

static TickType_t tick;
static bool tx_success;
static TickType_t tx_elapsed;
static unsigned heartbeat_attempts;

uint32_t xTaskGetTickCount(void) { return tick; }
void vTaskDelay(uint32_t ticks) { tick += ticks; }

bool host_uart_write_bytes(const uint8_t *data, size_t length)
{
    uint8_t raw[YERTLE_PROTOCOL_MAX_RAW];
    size_t raw_length = 0U;
    uint8_t type;
    uint16_t sequence;
    assert(length >= 3U && data[0] == 0U && data[length - 1U] == 0U);
    assert(cobs_decode(data + 1U, length - 2U, raw, &raw_length));
    assert(validate_frame(raw, raw_length, &type, &sequence));
    if (type == MESSAGE_F103_HEARTBEAT) ++heartbeat_attempts;
    tick += tx_elapsed;
    return tx_success;
}

bool host_uart_try_read(uint8_t *byte, bool *discontinuity)
{ (void)byte; *discontinuity = false; return false; }
bool sensor_take_position_offsets(motor_position_offsets_t *result)
{ (void)result; return false; }
bool sensor_request_position_offsets(uint16_t sequence)
{ (void)sequence; return false; }
bool sensor_request_motor_register(uint16_t sequence, uint8_t id, uint8_t address, uint8_t length)
{ (void)sequence; (void)id; (void)address; (void)length; return false; }
bool sensor_take_motor_register(motor_register_result_t *result)
{ (void)result; return false; }
bool sensor_get_feedback_snapshot(motor_feedback_set_t *motor, imu_feedback_t *imu)
{ (void)motor; (void)imu; return false; }
bool motor_device_validate_targets(const motor_target_set_t *targets)
{ (void)targets; return true; }
bool actuator_submit_targets(const motor_target_set_t *targets)
{ (void)targets; return true; }
void uart_diagnostics_send(void *context, uart_diag_sender_t send)
{ (void)context; (void)send; }

static protocol_context_t reset_context(bool connected, bool armed)
{
    protocol_context_t context = { 0 };
    tick = 0U;
    tx_success = true;
    tx_elapsed = 0U;
    heartbeat_attempts = 0U;
    published_state = HOST_FSM_DISARMED;
    requested_fault_reason = HOST_FAULT_NONE;
    requested_unsafe_reason = HOST_UNSAFE_NONE;
    pending_safety_action = HOST_SAFETY_ACTION_NONE;
    safe_stop_completed = false;
    requested_target_timeout = false;
    context.host_connected = connected;
    context.lease_established = armed;
    context.state = armed ? HOST_FSM_ARMED : HOST_FSM_DISARMED;
    return context;
}

static void receive_request(protocol_context_t *context, uint8_t type, bool valid_crc)
{
    uint8_t raw[YERTLE_PROTOCOL_HEADER_SIZE + YERTLE_PROTOCOL_CRC_SIZE] = {
        YERTLE_PROTOCOL_VERSION, type, 0U, 1U, 0U, 0U, 0U, 0U, 0U
    };
    const uint16_t crc = crc16_ccitt(raw, sizeof(raw) - 2U);
    raw[7] = (uint8_t)crc;
    raw[8] = (uint8_t)(crc >> 8U);
    if (!valid_crc) raw[7] ^= 1U;
    handle_frame(context, raw, sizeof(raw));
}

int main(void)
{
    /* Telemetry is sent every second, even while disconnected. */
    protocol_context_t context = reset_context(false, false);
    tick = 999U;
    service_link_heartbeat(&context);
    assert(heartbeat_attempts == 0U && context.last_host_heartbeat_tick == 0U);
    tick = 1000U;
    tx_elapsed = 3U;
    service_link_heartbeat(&context);
    assert(heartbeat_attempts == 1U && context.last_host_heartbeat_tick == 0U);
    assert(!context.host_connected && !context.lease_established);
    tick = 1999U;
    service_link_heartbeat(&context);
    assert(heartbeat_attempts == 1U);
    tick = 2000U;
    service_link_heartbeat(&context);
    assert(heartbeat_attempts == 2U && context.last_host_heartbeat_tick == 0U);

    /* Successful local TX cannot prevent the PC heartbeat timeout. */
    context = reset_context(true, true);
    tick = 999U;
    service_link_heartbeat(&context);
    assert(context.host_connected);
    tick = 1000U;
    service_link_heartbeat(&context);
    assert(heartbeat_attempts == 1U && context.last_host_heartbeat_tick == 0U);
    assert(!context.host_connected && !context.lease_established);
    assert(context.state == HOST_FSM_DISARMED);
    assert(pending_safety_action == HOST_SAFETY_ACTION_SAFE_STOP);

    /* PC RX keeps the link alive even when all local TX fails. */
    context = reset_context(true, true);
    tx_success = false;
    tick = 900U;
    receive_request(&context, MESSAGE_HEARTBEAT, true);
    assert(context.last_host_heartbeat_tick == 900U);
    tick = 1899U;
    service_link_heartbeat(&context);
    assert(context.host_connected);
    assert(context.last_host_heartbeat_tick == 900U);
    tick = 1900U;
    service_link_heartbeat(&context);
    assert(!context.host_connected);

    context = reset_context(true, true);
    for (tick = 250U; tick <= 3000U; tick += 250U) {
        receive_request(&context, MESSAGE_HEARTBEAT, true);
        service_link_heartbeat(&context);
        assert(context.host_connected && context.state == HOST_FSM_ARMED);
        assert(context.last_host_heartbeat_tick == tick);
    }
    assert(heartbeat_attempts == 3U);

    context = reset_context(true, false);
    tick = 900U;
    receive_request(&context, MESSAGE_HEARTBEAT, false);
    assert(context.last_host_heartbeat_tick == 0U && !context.lease_established);
    receive_request(&context, MESSAGE_GET_FSM_STATE, true);
    assert(context.last_host_heartbeat_tick == 0U && !context.lease_established);
    receive_request(&context, MESSAGE_HELLO, true);
    assert(context.last_host_heartbeat_tick == 0U && !context.lease_established);
    tick = 1000U;
    service_link_heartbeat(&context);
    assert(!context.host_connected);

    /* Initial HELLO allows one second to supply the first real heartbeat. */
    context = reset_context(false, false);
    tick = 250U;
    receive_request(&context, MESSAGE_HEARTBEAT, true);
    assert(!context.host_connected && !context.lease_established);
    receive_request(&context, MESSAGE_HELLO, true);
    assert(context.host_connected && !context.lease_established);
    assert(context.last_host_heartbeat_tick == 250U);
    tick = 1249U;
    service_link_heartbeat(&context);
    assert(context.host_connected && !context.lease_established);
    receive_request(&context, MESSAGE_HEARTBEAT, true);
    try_enter_control_ready(&context);
    assert(context.state == HOST_FSM_ARMED);

    context = reset_context(true, true);
    context.last_host_heartbeat_tick = UINT32_MAX - 100U;
    context.last_f103_heartbeat_tick = UINT32_MAX - 100U;
    tick = 149U;
    receive_request(&context, MESSAGE_HEARTBEAT, true);
    tick = 899U; /* 1000 ticks across wrap since previous telemetry */
    service_link_heartbeat(&context);
    assert(heartbeat_attempts == 1U);
    assert(context.host_connected && context.last_host_heartbeat_tick == 149U);
    tick = 1149U;
    service_link_heartbeat(&context);
    assert(!context.host_connected);

    puts("heartbeat: 1000ms telemetry, PC-only timeout, TX independence, lease isolation and tick wrap passed");
    return 0;
}
