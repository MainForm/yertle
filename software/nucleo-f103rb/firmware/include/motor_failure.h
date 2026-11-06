#ifndef MOTOR_FAILURE_H
#define MOTOR_FAILURE_H
#include <stdint.h>
#include <stdbool.h>
/* GET_UART_STATS / 0x91 sections 5,6=first; 7,8=latest.
 * 13 LE words per half; snapshot ID in both halves detects mismatched pairs.
 * Only feedback SyncRead failures are captured. Normal traffic never clears it.
 * RAM only: reset clears the records. No Flash writes or automatic TX. */
typedef struct { uint32_t meta[13], hw[13]; } motor_failure_snapshot_t;
void motor_failure_record(unsigned bus_index, motor_failure_snapshot_t *value);
bool motor_failure_get(unsigned bus_index, bool first, motor_failure_snapshot_t *out);
#endif
