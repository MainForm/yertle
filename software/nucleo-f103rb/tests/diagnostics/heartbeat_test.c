#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

/* Exercise the production frame handler and link service with a mock UART. */
#include "../../firmware/src/host_protocol.c"

static TickType_t tick;
static bool tx_success;
static TickType_t tx_elapsed;
static unsigned heartbeat_attempts;
static uint8_t heartbeat_state;
static bool armed_event_sent, armed_event_before_heartbeat;

static jmp_buf task_finished;
static bool run_task;
static unsigned task_passes;
static uint8_t task_rx[2U * (YERTLE_PROTOCOL_MAX_ENCODED + 2U)];
static size_t task_rx_length, task_rx_offset;

uint32_t xTaskGetTickCount(void) { return tick; }

void vTaskDelay(uint32_t ticks)
{
    tick += ticks;
    if (run_task) {
        if (++task_passes == 2U) longjmp(task_finished, 1);
        /* Start the next pass with both requests and a due F103 heartbeat. */
        tick = 1000U;
        host_protocol_complete_safe_stop();
    }
}

bool host_uart_write_bytes(const uint8_t *data, size_t length)
{
    uint8_t raw[YERTLE_PROTOCOL_MAX_RAW];
    size_t raw_length = 0U;
    uint8_t type;
    uint16_t sequence;
    assert(length >= 3U && data[0] == 0U && data[length - 1U] == 0U);
    assert(cobs_decode(data + 1U, length - 2U, raw, &raw_length));
    assert(validate_frame(raw, raw_length, &type, &sequence));
    const uint8_t *payload = raw + YERTLE_PROTOCOL_HEADER_SIZE;
    if (type == MESSAGE_F103_HEARTBEAT) {
        ++heartbeat_attempts;
        heartbeat_state = payload[4];
        armed_event_before_heartbeat = armed_event_sent;
    } else if (type == MESSAGE_EVENT && payload[0] == EVENT_STATE_CHANGED &&
               payload[2] == HOST_FSM_ARMED) {
        armed_event_sent = true;
    }
    tick += tx_elapsed;
    return tx_success;
}

bool host_uart_try_read(uint8_t *byte, bool *discontinuity)
{
    *discontinuity = false;
    if (!run_task || task_passes == 0U || task_rx_offset == task_rx_length) return false;
    *byte = task_rx[task_rx_offset++];
    return true;
}
motor_device_status_t motor_device_read_position_offset(uint8_t id, motor_position_offset_t *offset)
{ (void)id; (void)offset; assert(0); return MOTOR_DEVICE_BUSY; }
motor_register_status_t motor_device_read_register(uint8_t id, uint8_t address, uint8_t length, motor_register_data_t *result)
{ (void)id; (void)address; (void)length; (void)result; assert(0); return MOTOR_REGISTER_BUSY; }
motor_register_write_report_t motor_device_write_register(const motor_register_write_request_t *request, bool (*allowed)(void))
{ (void)request; (void)allowed; assert(0); return motor_register_initial_report(); }
uint16_t motor_device_register_lock_uncertain_mask(void) { return 0U; }
bool sensor_get_feedback_snapshot(motor_feedback_set_t *motor, imu_feedback_t *imu)
{ (void)motor; (void)imu; return false; }
bool motor_device_validate_targets(const motor_target_set_t *targets)
{ (void)targets; return true; }
bool motor_control_submit_targets(const motor_target_set_t *targets)
{ (void)targets; return true; }
void uart_diagnostics_send(void *context, uart_diag_sender_t send)
{ (void)context; (void)send; }
void uart_diagnostics_send_motor_buses(void *c, uart_diag_sender_t s,uint16_t q){(void)c;(void)s;(void)q;}

static protocol_context_t reset_context(bool connected, bool armed)
{
    protocol_context_t context = { 0 };
    tick = 0U;
    tx_success = true;
    tx_elapsed = 0U;
    heartbeat_attempts = 0U;
    heartbeat_state = HOST_FSM_BOOT;
    armed_event_sent = false;
    armed_event_before_heartbeat = false;
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

static void append_task_request(uint8_t type, uint8_t sequence)
{
    uint8_t raw[YERTLE_PROTOCOL_HEADER_SIZE + YERTLE_PROTOCOL_CRC_SIZE] = {
        YERTLE_PROTOCOL_VERSION, type, 0U, sequence, 0U, 0U, 0U, 0U, 0U
    };
    const uint16_t crc = crc16_ccitt(raw, YERTLE_PROTOCOL_HEADER_SIZE);
    raw[7] = (uint8_t)crc;
    raw[8] = (uint8_t)(crc >> 8U);
    task_rx[task_rx_length++] = 0U;
    task_rx_length += cobs_encode(raw, sizeof(raw), task_rx + task_rx_length);
    task_rx[task_rx_length++] = 0U;
}

static void test_arm_before_status_report(void)
{
    (void)reset_context(false, false);
    published_state = HOST_FSM_BOOT;
    task_passes = 0U;
    task_rx_length = 0U;
    task_rx_offset = 0U;
    append_task_request(MESSAGE_HELLO, 1U);
    append_task_request(MESSAGE_HEARTBEAT, 2U);

    run_task = true;
    if (setjmp(task_finished) == 0) host_protocol_task(NULL);
    run_task = false;
    assert(task_rx_offset == task_rx_length);
    assert(published_state == HOST_FSM_ARMED);
    assert(heartbeat_attempts == 1U && heartbeat_state == HOST_FSM_ARMED);
    assert(armed_event_before_heartbeat);
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
    assert(context.host_connected && !context.lease_established && context.state==HOST_FSM_DISARMED);
    assert(context.last_host_heartbeat_tick==0U && context.last_idle_monitor_tick==900U);
    tick = 1900U;
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

    /* Expiry must block ARM even before the link state has been updated. */
    context = reset_context(true, false);
    receive_request(&context, MESSAGE_HEARTBEAT, true);
    tick = 999U;
    assert(can_arm(&context));
    request_safe_stop();
    assert(!can_arm(&context));
    host_protocol_complete_safe_stop();
    service_safety_state(&context);
    assert(can_arm(&context));
    tick = 1000U;
    assert(context.host_connected && context.lease_established);
    try_enter_control_ready(&context);
    assert(context.state == HOST_FSM_DISARMED);

    expire_host_link(&context);
    assert(!context.host_connected && !context.lease_established);
    receive_request(&context, MESSAGE_HEARTBEAT, true);
    assert(!context.lease_established && !can_arm(&context));
    receive_request(&context, MESSAGE_HELLO, true);
    assert(context.host_connected && !can_arm(&context));
    receive_request(&context, MESSAGE_HEARTBEAT, true);
    assert(can_arm(&context));

    test_arm_before_status_report();
    puts("heartbeat: link expiry, ARM freshness, tick wrap and state-report order passed");
    return 0;
}

bool sensor_get_extended_feedback_snapshot(motor_feedback_set_t *m, imu_feedback_t *i, pmw3901_feedback_t *f){memset(f,0,sizeof *f);return sensor_get_feedback_snapshot(m,i);}
