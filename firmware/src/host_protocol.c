#include "host_protocol.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "host_uart.h"
#include "actuator_task.h"
#include "sensor_task.h"
#include "motor_uart.h"
#include "uart_diagnostics.h"

#define YERTLE_PROTOCOL_VERSION              1U
#define YERTLE_PROTOCOL_MAX_PAYLOAD          64U
#define YERTLE_PROTOCOL_HEADER_SIZE           7U
#define YERTLE_PROTOCOL_CRC_SIZE              2U
#define YERTLE_PROTOCOL_MAX_RAW (YERTLE_PROTOCOL_HEADER_SIZE + YERTLE_PROTOCOL_MAX_PAYLOAD + YERTLE_PROTOCOL_CRC_SIZE)
#define YERTLE_PROTOCOL_MAX_ENCODED (YERTLE_PROTOCOL_MAX_RAW + 2U)
#define YERTLE_HEARTBEAT_PERIOD_MS         1000U
#define YERTLE_HOST_TIMEOUT_MS             1000U

enum message_type {
    MESSAGE_HELLO = 0x01U,
    MESSAGE_HEARTBEAT = 0x02U,
    MESSAGE_SET_JOINT_TARGETS = 0x06U,
    MESSAGE_GET_UART_STATS = 0x08U,
    MESSAGE_GET_SENSOR_FEEDBACK = 0x09U,
    MESSAGE_GET_FSM_STATE = 0x0AU,
    MESSAGE_ACK = 0x80U,
    MESSAGE_NACK = 0x81U,
    MESSAGE_HELLO_ACK = 0x82U,
    MESSAGE_EVENT = 0x83U,
    MESSAGE_F103_HEARTBEAT = 0x84U,
    MESSAGE_UART_STATS = 0x85U,
    MESSAGE_UART_ISR_STATS = 0x86U,
    MESSAGE_MOTOR_DIAGNOSTICS = 0x87U,
    MESSAGE_MOTOR_TRACE = 0x88U,
    MESSAGE_UART_RX_ERRORS = 0x89U,
    MESSAGE_MOTOR_FEEDBACK = 0x8BU,
    MESSAGE_IMU_FEEDBACK = 0x8CU,
    MESSAGE_FSM_STATE = 0x8DU
};

enum error_code {
    ERROR_UNKNOWN_MESSAGE = 1U,
    ERROR_INVALID_LENGTH = 2U,
    ERROR_NOT_CONNECTED = 3U,
    ERROR_UNSUPPORTED_VERSION = 4U,
    ERROR_INVALID_STATE = 5U,
    ERROR_INVALID_FLAGS = 7U,
    ERROR_INVALID_TARGET = 8U,
    ERROR_TARGET_BUSY = 9U,
    ERROR_FEEDBACK_UNAVAILABLE = 10U
};

enum event_code {
    EVENT_STATE_CHANGED = 1U,
    EVENT_HOST_LINK_CHANGED = 2U,
    EVENT_UNSAFE_REPORTED = 3U
};

enum state_cause {
    STATE_CAUSE_BOOT_COMPLETE = 1U,
    STATE_CAUSE_HEARTBEAT_TIMEOUT = 5U,
    STATE_CAUSE_LEASE_READY = 6U,
    STATE_CAUSE_TARGET_TIMEOUT = 7U,
    STATE_CAUSE_FAULT = 8U,
    STATE_CAUSE_SAFE_STOP_COMPLETE = 9U
};

typedef struct {
    uint8_t encoded[YERTLE_PROTOCOL_MAX_ENCODED];
    size_t encoded_length;
    uint8_t decoded[YERTLE_PROTOCOL_MAX_RAW];
    bool host_connected;
    bool lease_established;
    host_fsm_state_t state;
    TickType_t last_host_tick;
    TickType_t last_f103_heartbeat_tick;
    host_unsafe_reason_t last_reported_unsafe_reason;
    uint16_t tx_sequence;
} protocol_context_t;

static volatile host_fsm_state_t published_state = HOST_FSM_BOOT;
static volatile bool published_host_connected = false;
static volatile host_fault_reason_t requested_fault_reason = HOST_FAULT_NONE;
static volatile host_unsafe_reason_t requested_unsafe_reason = HOST_UNSAFE_NONE;
static volatile host_safety_action_t pending_safety_action = HOST_SAFETY_ACTION_NONE;
static volatile bool safe_stop_completed = false;
static volatile bool requested_target_timeout = false;

static uint16_t crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;

    for (size_t index = 0U; index < length; ++index) {
        crc ^= (uint16_t) data[index] << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                ? (uint16_t) ((crc << 1U) ^ 0x1021U)
                : (uint16_t) (crc << 1U);
        }
    }
    return crc;
}

static bool cobs_decode(const uint8_t *input, size_t input_length,
                        uint8_t *output, size_t *output_length)
{
    size_t read_index = 0U;
    size_t write_index = 0U;

    while (read_index < input_length) {
        const uint8_t code = input[read_index++];
        if ((code == 0U) || ((size_t) (code - 1U) > (input_length - read_index))) {
            return false;
        }
        for (uint8_t copy = 1U; copy < code; ++copy) {
            if (write_index >= YERTLE_PROTOCOL_MAX_RAW) {
                return false;
            }
            output[write_index++] = input[read_index++];
        }
        if ((code != 0xFFU) && (read_index < input_length)) {
            if (write_index >= YERTLE_PROTOCOL_MAX_RAW) {
                return false;
            }
            output[write_index++] = 0U;
        }
    }

    *output_length = write_index;
    return true;
}

static size_t cobs_encode(const uint8_t *input, size_t input_length, uint8_t *output)
{
    size_t read_index = 0U;
    size_t write_index = 1U;
    size_t code_index = 0U;
    uint8_t code = 1U;

    while (read_index < input_length) {
        if (input[read_index] == 0U) {
            output[code_index] = code;
            code = 1U;
            code_index = write_index++;
            ++read_index;
        } else {
            output[write_index++] = input[read_index++];
            ++code;
            if (code == 0xFFU) {
                output[code_index] = code;
                code = 1U;
                code_index = write_index++;
            }
        }
    }
    output[code_index] = code;
    return write_index;
}

static void send_frame(protocol_context_t *context, uint8_t type,
                       const uint8_t *payload, uint16_t payload_length)
{
    uint8_t raw[YERTLE_PROTOCOL_MAX_RAW];
    uint8_t encoded[YERTLE_PROTOCOL_MAX_ENCODED];
    const size_t raw_length = YERTLE_PROTOCOL_HEADER_SIZE + payload_length + YERTLE_PROTOCOL_CRC_SIZE;
    const uint16_t sequence = context->tx_sequence++;

    if (payload_length > YERTLE_PROTOCOL_MAX_PAYLOAD) {
        return;
    }

    raw[0] = YERTLE_PROTOCOL_VERSION;
    raw[1] = type;
    raw[2] = 0U;
    raw[3] = (uint8_t) sequence;
    raw[4] = (uint8_t) (sequence >> 8U);
    raw[5] = (uint8_t) payload_length;
    raw[6] = (uint8_t) (payload_length >> 8U);
    for (uint16_t index = 0U; index < payload_length; ++index) {
        raw[YERTLE_PROTOCOL_HEADER_SIZE + index] = payload[index];
    }

    const uint16_t crc = crc16_ccitt(raw, raw_length - YERTLE_PROTOCOL_CRC_SIZE);
    raw[raw_length - 2U] = (uint8_t) crc;
    raw[raw_length - 1U] = (uint8_t) (crc >> 8U);

    const size_t encoded_length = cobs_encode(raw, raw_length, encoded);
    const uint8_t delimiter = 0U;
    host_uart_write_bytes(&delimiter, 1U);
    host_uart_write_bytes(encoded, encoded_length);
    host_uart_write_bytes(&delimiter, 1U);
}

static void send_result(protocol_context_t *context, uint8_t result_type,
                        uint8_t request_type, uint16_t request_sequence, uint8_t code)
{
    const uint8_t payload[4] = {
        request_type, (uint8_t) request_sequence, (uint8_t) (request_sequence >> 8U), code
    };
    send_frame(context, result_type, payload, sizeof(payload));
}

static void send_diagnostic_frame(void *context, uint8_t type,
                                  const uint8_t *payload, uint16_t length)
{
    send_frame((protocol_context_t *) context, type, payload, length);
}

static void send_hello_ack(protocol_context_t *context)
{
    const uint8_t payload[] = {
        1U, 0U, 0U,                         /* firmware semantic version */
        YERTLE_PROTOCOL_VERSION,
        0x01U, 0U, 0U, 0U                  /* phase-1 FSM capability */
    };
    send_frame(context, MESSAGE_HELLO_ACK, payload, sizeof(payload));
}

static void send_f103_heartbeat(protocol_context_t *context)
{
    const uint32_t uptime_ms = (uint32_t) xTaskGetTickCount();
    const uint8_t payload[] = {
        (uint8_t) uptime_ms, (uint8_t) (uptime_ms >> 8U),
        (uint8_t) (uptime_ms >> 16U), (uint8_t) (uptime_ms >> 24U),
        (uint8_t) context->state,
        context->host_connected ? 1U : 0U,
        (uint8_t) pending_safety_action,
        (uint8_t) requested_unsafe_reason,
        (uint8_t) requested_fault_reason
    };
    send_frame(context, MESSAGE_F103_HEARTBEAT, payload, sizeof(payload));
}

static void send_fsm_state(protocol_context_t *context, uint16_t request_sequence)
{
    const uint8_t payload[] = {
        (uint8_t)request_sequence,
        (uint8_t)(request_sequence >> 8U),
        (uint8_t)context->state
    };
    send_frame(context, MESSAGE_FSM_STATE, payload, sizeof(payload));
}

static void send_motor_feedback(protocol_context_t *context, const motor_feedback_set_t *feedback)
{
    for (uint8_t first = 0U; first < YERTLE_MOTOR_COUNT; first += 4U) {
        uint8_t payload[56] = { 0 };
        const uint32_t sequence = feedback->sequence, uptime = feedback->uptime_ms;
        for (uint8_t b = 0U; b < 4U; ++b) {
            payload[b] = (uint8_t)(sequence >> (8U * b));
            payload[4U + b] = (uint8_t)(uptime >> (8U * b));
        }
        payload[8] = (uint8_t)feedback->valid_mask;
        payload[9] = (uint8_t)(feedback->valid_mask >> 8U);
        payload[10] = first + 1U;
        payload[11] = 4U;
        for (uint8_t j = 0U; j < 4U; ++j) {
            const motor_feedback_t *m = &feedback->motor[first + j];
            const size_t p = 12U + (size_t)j * 11U;
            payload[p] = (uint8_t)m->position;
            payload[p+1U] = (uint8_t)((uint16_t)m->position >> 8U);
            payload[p+2U] = (uint8_t)m->speed;
            payload[p+3U] = (uint8_t)((uint16_t)m->speed >> 8U);
            payload[p+4U] = (uint8_t)m->load;
            payload[p+5U] = (uint8_t)((uint16_t)m->load >> 8U);
            payload[p+6U] = (uint8_t)m->current;
            payload[p+7U] = (uint8_t)((uint16_t)m->current >> 8U);
            payload[p+8U] = m->voltage;
            payload[p+9U] = m->temperature;
            payload[p+10U] = m->servo_error;
        }
        send_frame(context, MESSAGE_MOTOR_FEEDBACK, payload, sizeof(payload));
    }
}

static void send_imu_feedback(protocol_context_t *context, const imu_feedback_t *feedback)
{
    uint8_t payload[24] = { 0 };
    const int16_t values[7] = {
        feedback->raw.accel_x, feedback->raw.accel_y, feedback->raw.accel_z,
        feedback->raw.temperature, feedback->raw.gyro_x, feedback->raw.gyro_y,
        feedback->raw.gyro_z
    };
    for (uint8_t b = 0U; b < 4U; ++b) {
        payload[b] = (uint8_t)(feedback->sequence >> (8U * b));
        payload[4U + b] = (uint8_t)(feedback->uptime_ms >> (8U * b));
    }
    payload[8] = feedback->valid ? 1U : 0U;
    /* payload[9] is reserved for future status codes. */
    if (feedback->valid) {
        for (uint8_t i = 0U; i < 7U; ++i) {
            payload[10U + i * 2U] = (uint8_t)values[i];
            payload[11U + i * 2U] = (uint8_t)((uint16_t)values[i] >> 8U);
        }
    }
    send_frame(context, MESSAGE_IMU_FEEDBACK, payload, sizeof(payload));
}

static void transition_state(protocol_context_t *context, host_fsm_state_t next,
                             uint8_t cause, bool announce)
{
    const host_fsm_state_t previous = context->state;

    if (previous == next) {
        return;
    }
    context->state = next;
    published_state = next;
    if (announce) {
        const uint8_t payload[] = {
            EVENT_STATE_CHANGED, (uint8_t) previous, (uint8_t) next, cause
        };
        send_frame(context, MESSAGE_EVENT, payload, sizeof(payload));
    }
}

static void request_safe_stop(void)
{
    safe_stop_completed = false;
    pending_safety_action = HOST_SAFETY_ACTION_SAFE_STOP;
}

static void set_host_connected(protocol_context_t *context, bool connected)
{
    if (context->host_connected == connected) {
        return;
    }
    context->host_connected = connected;
    if (!connected) context->lease_established = false;
    published_host_connected = connected;
    const uint8_t payload[] = { EVENT_HOST_LINK_CHANGED, connected ? 1U : 0U };
    send_frame(context, MESSAGE_EVENT, payload, sizeof(payload));
}

static bool can_arm(const protocol_context_t *context)
{
    return context->host_connected && context->lease_established &&
           (context->state == HOST_FSM_DISARMED) &&
           (pending_safety_action == HOST_SAFETY_ACTION_NONE) &&
           (requested_fault_reason == HOST_FAULT_NONE);
}

static void try_enter_control_ready(protocol_context_t *context)
{
    if (can_arm(context)) {
        transition_state(context, HOST_FSM_ARMED, STATE_CAUSE_LEASE_READY, true);
    }
}

static bool validate_frame(const uint8_t *raw, size_t raw_length,
                           uint8_t *type, uint16_t *sequence)
{
    if (raw_length < (YERTLE_PROTOCOL_HEADER_SIZE + YERTLE_PROTOCOL_CRC_SIZE)) {
        return false;
    }

    const uint16_t payload_length = (uint16_t) raw[5] | ((uint16_t) raw[6] << 8U);
    const size_t expected_length = YERTLE_PROTOCOL_HEADER_SIZE + payload_length + YERTLE_PROTOCOL_CRC_SIZE;
    if ((payload_length > YERTLE_PROTOCOL_MAX_PAYLOAD) || (expected_length != raw_length)) {
        return false;
    }
    if (crc16_ccitt(raw, raw_length - YERTLE_PROTOCOL_CRC_SIZE) !=
        ((uint16_t) raw[raw_length - 2U] | ((uint16_t) raw[raw_length - 1U] << 8U))) {
        return false;
    }

    *type = raw[1];
    *sequence = (uint16_t) raw[3] | ((uint16_t) raw[4] << 8U);
    return true;
}

static void handle_frame(protocol_context_t *context, const uint8_t *raw, size_t raw_length)
{
    uint8_t type;
    uint16_t sequence;
    if (!validate_frame(raw, raw_length, &type, &sequence)) {
        return;
    }
    if (raw[0] != YERTLE_PROTOCOL_VERSION) {
        send_result(context, MESSAGE_NACK, type, sequence, ERROR_UNSUPPORTED_VERSION);
        return;
    }
    if (raw[2] != 0U) {
        send_result(context, MESSAGE_NACK, type, sequence, ERROR_INVALID_FLAGS);
        return;
    }
    if (type != MESSAGE_SET_JOINT_TARGETS && ((raw[5] != 0U) || (raw[6] != 0U))) {
        send_result(context, MESSAGE_NACK, type, sequence, ERROR_INVALID_LENGTH);
        return;
    }

    switch (type) {
    case MESSAGE_GET_FSM_STATE:
        /* Read-only: never refreshes the host lease or changes the FSM. */
        send_fsm_state(context, sequence);
        break;
    case MESSAGE_GET_SENSOR_FEEDBACK: {
        motor_feedback_set_t feedback;
        imu_feedback_t imu;
        if (!sensor_get_feedback_snapshot(&feedback, &imu) || feedback.sequence == 0U) {
            send_result(context, MESSAGE_NACK, type, sequence, ERROR_FEEDBACK_UNAVAILABLE);
        } else {
            send_motor_feedback(context, &feedback);
            send_imu_feedback(context, &imu);
        }
        break;
    }
    case MESSAGE_SET_JOINT_TARGETS: {
        if (raw[5] != YERTLE_MOTOR_COUNT * 5U || raw[6] != 0U) {
            send_result(context, MESSAGE_NACK, type, sequence, ERROR_INVALID_LENGTH);
            break;
        }
        if (!context->host_connected || context->state != HOST_FSM_ARMED ||
            pending_safety_action != HOST_SAFETY_ACTION_NONE) {
            send_result(context, MESSAGE_NACK, type, sequence, ERROR_INVALID_STATE);
            break;
        }
        motor_target_set_t targets = { 0 };
        targets.sequence = sequence;
        for (uint8_t i = 0U; i < YERTLE_MOTOR_COUNT; ++i) {
            const size_t offset = YERTLE_PROTOCOL_HEADER_SIZE + (size_t)i * 5U;
            targets.motor[i].position = (uint16_t)raw[offset] | ((uint16_t)raw[offset + 1U] << 8U);
            targets.motor[i].speed = (uint16_t)raw[offset + 2U] | ((uint16_t)raw[offset + 3U] << 8U);
            targets.motor[i].acceleration = raw[offset + 4U];
        }
        if (!motor_device_validate_targets(&targets)) {
            send_result(context, MESSAGE_NACK, type, sequence, ERROR_INVALID_TARGET);
        } else if (!actuator_submit_targets(&targets)) {
            send_result(context, MESSAGE_NACK, type, sequence, ERROR_TARGET_BUSY);
        } else {
            send_result(context, MESSAGE_ACK, type, sequence, 0U);
        }
        break;
    }
    case MESSAGE_GET_UART_STATS:
        /* Read-only diagnostic: does not refresh the control lease or ARM. */
        uart_diagnostics_send(context, send_diagnostic_frame);
        break;
    case MESSAGE_HELLO:
        context->last_host_tick = xTaskGetTickCount();
        set_host_connected(context, true);
        send_hello_ack(context);
        send_result(context, MESSAGE_ACK, type, sequence, 0U);
        break;
    case MESSAGE_HEARTBEAT:
        if (!context->host_connected) {
            send_result(context, MESSAGE_NACK, type, sequence, ERROR_NOT_CONNECTED);
            break;
        }
        context->last_host_tick = xTaskGetTickCount();
        context->lease_established = true;
        send_result(context, MESSAGE_ACK, type, sequence, 0U);
        break;
    default:
        send_result(context, MESSAGE_NACK, type, sequence, ERROR_UNKNOWN_MESSAGE);
        break;
    }
}

void host_protocol_task(void *argument)
{
    protocol_context_t context = { 0 };
    uint8_t byte;

    (void) argument;
    context.last_host_tick = xTaskGetTickCount();
    context.last_f103_heartbeat_tick = context.last_host_tick;
    transition_state(&context, HOST_FSM_DISARMED, STATE_CAUSE_BOOT_COMPLETE, false);
    request_safe_stop();

    for (;;) {
        for (;;) {
            bool discontinuity = false;
            const bool available = host_uart_try_read(&byte, &discontinuity);
            if (discontinuity) {
                context.encoded_length = 0U;
            }
            if (!available) {
                break;
            }
            if (byte == 0U) {
                if (context.encoded_length > 0U) {
                    size_t decoded_length = 0U;
                    if (cobs_decode(context.encoded, context.encoded_length,
                                    context.decoded, &decoded_length)) {
                        handle_frame(&context, context.decoded, decoded_length);
                    }
                }
                context.encoded_length = 0U;
            } else if (context.encoded_length < sizeof(context.encoded)) {
                context.encoded[context.encoded_length++] = byte;
            } else {
                context.encoded_length = 0U;
            }
        }

        const TickType_t now = xTaskGetTickCount();
        if (safe_stop_completed) {
            safe_stop_completed = false;
            pending_safety_action = HOST_SAFETY_ACTION_NONE;
            if (context.state == HOST_FSM_FAULT) {
                requested_fault_reason = HOST_FAULT_NONE;
                /* Stay DISARMED until a heartbeat received after recovery. */
                context.lease_established = false;
                requested_target_timeout = false;
                transition_state(&context, HOST_FSM_DISARMED,
                                 STATE_CAUSE_SAFE_STOP_COMPLETE, true);
            }
        }

        if ((requested_fault_reason != HOST_FAULT_NONE) && (context.state != HOST_FSM_FAULT)) {
            transition_state(&context, HOST_FSM_FAULT, STATE_CAUSE_FAULT, true);
            request_safe_stop();
        }

        if (requested_target_timeout) {
            requested_target_timeout = false;
            if (context.state == HOST_FSM_ARMED) {
                /* Require a fresh host lease before accepting another target. */
                context.lease_established = false;
                transition_state(&context, HOST_FSM_DISARMED, STATE_CAUSE_TARGET_TIMEOUT, true);
                request_safe_stop();
            }
        }

        if (context.last_reported_unsafe_reason != requested_unsafe_reason) {
            context.last_reported_unsafe_reason = requested_unsafe_reason;
            const uint8_t payload[] = {
                EVENT_UNSAFE_REPORTED, (uint8_t) requested_unsafe_reason
            };
            send_frame(&context, MESSAGE_EVENT, payload, sizeof(payload));
        }

        if (context.host_connected &&
            ((now - context.last_host_tick) >= pdMS_TO_TICKS(YERTLE_HOST_TIMEOUT_MS))) {
            set_host_connected(&context, false);
            if (context.state == HOST_FSM_ARMED) {
                transition_state(&context, HOST_FSM_DISARMED, STATE_CAUSE_HEARTBEAT_TIMEOUT, true);
                request_safe_stop();
            }
        }

        /* The FSM owns the transition; heartbeat only provides a healthy lease. */
        try_enter_control_ready(&context);

        if ((now - context.last_f103_heartbeat_tick) >= pdMS_TO_TICKS(YERTLE_HEARTBEAT_PERIOD_MS)) {
            context.last_f103_heartbeat_tick = now;
            send_f103_heartbeat(&context);
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

host_safety_action_t host_protocol_get_pending_safety_action(void)
{
    return pending_safety_action;
}

void host_protocol_complete_safe_stop(void)
{
    safe_stop_completed = true;
}

void host_protocol_latch_unsafe(host_unsafe_reason_t reason)
{
    if (reason != HOST_UNSAFE_NONE) {
        requested_unsafe_reason = reason;
    }
}

void host_protocol_clear_unsafe(void)
{
    requested_unsafe_reason = HOST_UNSAFE_NONE;
}

void host_protocol_latch_fault(host_fault_reason_t reason)
{
    if (reason != HOST_FAULT_NONE) {
        requested_fault_reason = reason;
    }
}

void host_protocol_request_target_timeout(void)
{
    requested_target_timeout = true;
}

host_fsm_state_t host_protocol_get_state(void)
{
    return published_state;
}

bool host_protocol_is_armed(void)
{
    return published_state == HOST_FSM_ARMED;
}

bool host_protocol_is_host_connected(void)
{
    return published_host_connected;
}
