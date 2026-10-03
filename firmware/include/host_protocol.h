#ifndef HOST_PROTOCOL_H
#define HOST_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Host-owned FSM. FAULT stays at value 5 in the v1 host wire format.
 * Values 3 and 4 are intentionally unused after removing the two transient
 * states that previously represented timeout and unsafe detection.
 */
typedef enum {
    HOST_FSM_BOOT = 0U,
    HOST_FSM_DISARMED = 1U,
    HOST_FSM_ARMED = 2U,
    HOST_FSM_FAULT = 5U
} host_fsm_state_t;

typedef enum {
    HOST_SAFETY_ACTION_NONE = 0U,
    HOST_SAFETY_ACTION_SAFE_STOP = 1U
} host_safety_action_t;

typedef enum {
    HOST_FAULT_NONE = 0U,
    HOST_FAULT_DRIVER = 1U,
    HOST_FAULT_COMMUNICATION = 2U,
    HOST_FAULT_SERVO = 3U
} host_fault_reason_t;

/* A diagnostic input for the later safety-monitor task, not an FSM state. */
typedef enum {
    HOST_UNSAFE_NONE = 0U,
    HOST_UNSAFE_CURRENT_LIMIT = 1U,
    HOST_UNSAFE_LOAD_LIMIT = 2U,
    HOST_UNSAFE_VOLTAGE_LIMIT = 3U,
    HOST_UNSAFE_SERVO_LIMIT = 4U
} host_unsafe_reason_t;

void host_protocol_task(void *argument);

/* Task-context interface shared with actuator_task. Never call from ISR. */
host_safety_action_t host_protocol_get_pending_safety_action(void);
/* Completion also releases FAULT into DISARMED, requiring a fresh heartbeat. */
void host_protocol_complete_safe_stop(void);
void host_protocol_latch_unsafe(host_unsafe_reason_t reason);
void host_protocol_clear_unsafe(void);
void host_protocol_latch_fault(host_fault_reason_t reason);
void host_protocol_request_target_timeout(void);
host_fsm_state_t host_protocol_get_state(void);
bool host_protocol_is_armed(void);

#endif /* HOST_PROTOCOL_H */
