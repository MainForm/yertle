#ifndef YERTLE_MODE_STORE_H
#define YERTLE_MODE_STORE_H
#include <stdbool.h>
#include <stdint.h>
/* Single writer: actuator task, DISARMED and all torque independently OFF.
 * Never persists arming, goals, controller integrators or session epochs. */
typedef struct { uint8_t backend; uint16_t mask; } mode_setting_t;
typedef enum { MODE_STORE_ABSENT,MODE_STORE_VALID,MODE_STORE_READ_ERROR } mode_store_result_t;
mode_store_result_t mode_store_read(mode_setting_t *setting);
bool mode_store_load(mode_setting_t *setting);
bool mode_store_save(const mode_setting_t *setting);
#endif
