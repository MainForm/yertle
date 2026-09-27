#include "motor_device.h"

#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "motor_uart.h"
#include "sts3215_protocol.h"
#include "sts3215_regs.h"

#define MOTOR_BUS_LOCK_TIMEOUT_MS 20U
#define MOTOR_UART_TIMEOUT_MS      5U
#define MOTOR_POSITION_MAX      4095U
#define MOTOR_SPEED_MAX         3400U

static SemaphoreHandle_t bus_mutex;
static uint8_t tx_frame[STS3215_TX_BUF_SIZE];
static uint8_t rx_frame[STS3215_RX_BUF_SIZE];
static const uint8_t motor_ids[YERTLE_MOTOR_COUNT] = { 1U, 2U, 3U, 4U, 5U, 6U };

static int motor_index_from_id(uint8_t id)
{
    if ((id < 1U) || (id > YERTLE_MOTOR_COUNT)) {
        return -1;
    }
    return (int) id - 1;
}

static bool take_bus(void)
{
    return (bus_mutex != NULL) &&
           (xSemaphoreTake(bus_mutex, pdMS_TO_TICKS(MOTOR_BUS_LOCK_TIMEOUT_MS)) == pdTRUE);
}

static motor_device_status_t send_broadcast(int16_t frame_length)
{
    if (frame_length <= 0) {
        return MOTOR_DEVICE_PROTOCOL_ERROR;
    }
    motor_uart_discard_rx();
    return motor_uart_write(tx_frame, (size_t) frame_length, MOTOR_UART_TIMEOUT_MS)
        ? MOTOR_DEVICE_OK : MOTOR_DEVICE_UART_ERROR;
}

static uint8_t read_sync_block(uint8_t reg, uint8_t data_length,
                               motor_feedback_set_t *feedback, bool current_block)
{
    const int16_t frame_length = STS3215_BuildSyncRead(
        tx_frame, sizeof(tx_frame), reg, data_length, motor_ids, YERTLE_MOTOR_COUNT);
    if (frame_length <= 0) {
        return 0U;
    }

    motor_uart_discard_rx();
    if (!motor_uart_write(tx_frame, (size_t) frame_length, MOTOR_UART_TIMEOUT_MS)) {
        return 0U;
    }

    uint8_t received_mask = 0U;
    for (uint8_t reply_index = 0U; reply_index < YERTLE_MOTOR_COUNT; ++reply_index) {
        size_t received_length = 0U;
        if (!motor_uart_read_frame(rx_frame, sizeof(rx_frame), &received_length,
                                   MOTOR_UART_TIMEOUT_MS)) {
            break;
        }

        STS3215_Reply_t reply = { 0 };
        const STS3215_Status_t parse_status =
            STS3215_ParseReply(rx_frame, (uint16_t) received_length, &reply);
        if ((parse_status != STS3215_OK) &&
            (parse_status != STS3215_ERR_SERVO_FAULT)) {
            continue;
        }

        const int index = motor_index_from_id(reply.id);
        if ((index < 0) || (reply.data_len < data_length)) {
            continue;
        }

        motor_feedback_t *motor = &feedback->motor[index];
        motor->id = reply.id;
        motor->servo_error = reply.error;
        if (current_block) {
            motor->current = STS3215_UnpackS16LE(&reply.data[0]);
        } else {
            motor->position = STS3215_UnpackS16LE(&reply.data[0]);
            motor->speed = STS3215_UnpackS16LE(&reply.data[2]);
            motor->load = STS3215_UnpackS16LE(&reply.data[4]);
            motor->voltage = reply.data[6];
            motor->temperature = reply.data[7];
        }
        received_mask |= (uint8_t) (1U << (uint8_t) index);
    }
    return received_mask;
}

bool motor_device_init(void)
{
    if (bus_mutex == NULL) {
        bus_mutex = xSemaphoreCreateMutex();
    }
    if (bus_mutex == NULL) {
        return false;
    }
    motor_uart_init();
    return true;
}

bool motor_device_validate_targets(const motor_target_set_t *targets)
{
    if (targets == NULL) {
        return false;
    }
    for (uint8_t index = 0U; index < YERTLE_MOTOR_COUNT; ++index) {
        if ((targets->motor[index].position > MOTOR_POSITION_MAX) ||
            (targets->motor[index].speed > MOTOR_SPEED_MAX)) {
            return false;
        }
    }
    return true;
}

motor_device_status_t motor_device_write_targets(const motor_target_set_t *targets)
{
    if (!motor_device_validate_targets(targets)) {
        return MOTOR_DEVICE_INVALID_ARGUMENT;
    }
    if (!take_bus()) {
        return bus_mutex == NULL ? MOTOR_DEVICE_NOT_INITIALIZED : MOTOR_DEVICE_BUSY;
    }

    STS3215_SyncEntry_t entries[YERTLE_MOTOR_COUNT] = { 0 };
    for (uint8_t index = 0U; index < YERTLE_MOTOR_COUNT; ++index) {
        const motor_target_t *target = &targets->motor[index];
        entries[index].id = motor_ids[index];
        entries[index].data[0] = target->acceleration;
        entries[index].data[1] = (uint8_t) target->position;
        entries[index].data[2] = (uint8_t) (target->position >> 8U);
        entries[index].data[3] = 0U;
        entries[index].data[4] = 0U;
        entries[index].data[5] = (uint8_t) target->speed;
        entries[index].data[6] = (uint8_t) (target->speed >> 8U);
    }

    const int16_t frame_length = STS3215_BuildSyncWrite(
        tx_frame, sizeof(tx_frame), STS3215_REG_ACCELERATION, 7U,
        entries, YERTLE_MOTOR_COUNT);
    const motor_device_status_t result = send_broadcast(frame_length);
    (void) xSemaphoreGive(bus_mutex);
    return result;
}

motor_device_status_t motor_device_set_torque(bool enabled)
{
    if (!take_bus()) {
        return bus_mutex == NULL ? MOTOR_DEVICE_NOT_INITIALIZED : MOTOR_DEVICE_BUSY;
    }
    const int16_t frame_length = enabled
        ? STS3215_BuildEnableTorque(tx_frame, sizeof(tx_frame), STS3215_BROADCAST_ID)
        : STS3215_BuildDisableTorque(tx_frame, sizeof(tx_frame), STS3215_BROADCAST_ID);
    const motor_device_status_t result = send_broadcast(frame_length);
    (void) xSemaphoreGive(bus_mutex);
    return result;
}

motor_device_status_t motor_device_read_feedback(motor_feedback_set_t *feedback)
{
    if (feedback == NULL) {
        return MOTOR_DEVICE_INVALID_ARGUMENT;
    }
    if (!take_bus()) {
        return bus_mutex == NULL ? MOTOR_DEVICE_NOT_INITIALIZED : MOTOR_DEVICE_BUSY;
    }

    memset(feedback, 0, sizeof(*feedback));
    for (uint8_t index = 0U; index < YERTLE_MOTOR_COUNT; ++index) {
        feedback->motor[index].id = motor_ids[index];
    }

    const uint8_t motion_mask = read_sync_block(
        STS3215_REG_CURRENT_POS, 8U, feedback, false);
    const uint8_t current_mask = read_sync_block(
        STS3215_REG_CURRENT_CURRENT, 2U, feedback, true);
    feedback->valid_mask = motion_mask & current_mask;

    for (uint8_t index = 0U; index < YERTLE_MOTOR_COUNT; ++index) {
        feedback->motor[index].valid =
            (feedback->valid_mask & (uint8_t) (1U << index)) != 0U;
    }

    (void) xSemaphoreGive(bus_mutex);
    return feedback->valid_mask == 0x3FU
        ? MOTOR_DEVICE_OK : MOTOR_DEVICE_PARTIAL_FEEDBACK;
}
