#ifndef MOTOR_DEVICE_H
#define MOTOR_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

#define YERTLE_MOTOR_COUNT 6U

typedef struct {
    uint16_t position;
    uint16_t speed;
    uint8_t acceleration;
} motor_target_t;

typedef struct {
    uint32_t sequence;
    motor_target_t motor[YERTLE_MOTOR_COUNT];
} motor_target_set_t;

typedef struct {
    uint8_t id;
    bool valid;
    int16_t position;
    int16_t speed;
    int16_t load;
    int16_t current;
    uint8_t voltage;
    uint8_t temperature;
    uint8_t servo_error;
} motor_feedback_t;

typedef struct {
    uint32_t sequence;
    uint8_t valid_mask;
    motor_feedback_t motor[YERTLE_MOTOR_COUNT];
} motor_feedback_set_t;

typedef enum {
    MOTOR_DEVICE_OK = 0,
    MOTOR_DEVICE_INVALID_ARGUMENT,
    MOTOR_DEVICE_NOT_INITIALIZED,
    MOTOR_DEVICE_BUSY,
    MOTOR_DEVICE_PROTOCOL_ERROR,
    MOTOR_DEVICE_UART_ERROR,
    MOTOR_DEVICE_PARTIAL_FEEDBACK
} motor_device_status_t;

bool motor_device_init(void);
bool motor_device_validate_targets(const motor_target_set_t *targets);
motor_device_status_t motor_device_write_targets(const motor_target_set_t *targets);
motor_device_status_t motor_device_set_torque(bool enabled);
motor_device_status_t motor_device_read_feedback(motor_feedback_set_t *feedback);

#endif /* MOTOR_DEVICE_H */
