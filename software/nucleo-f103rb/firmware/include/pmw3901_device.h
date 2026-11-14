#ifndef YERTLE_PMW3901_DEVICE_H
#define YERTLE_PMW3901_DEVICE_H
#include <stdbool.h>
#include <stdint.h>
/* Numeric values are also used in host feedback and SPI diagnostics. */
enum { PMW_START = 0, PMW_ID = 4, PMW_RESET, PMW_CLEAR, PMW_CONFIG,
       PMW_DISCARD, PMW_READY, PMW_RETRY };
enum { PMW_ERROR_NONE, PMW_ERROR_SPI, PMW_ERROR_ID };
typedef struct {
    uint32_t sequence, uptime_ms, interval_ms, errors;
    uint16_t epoch;
    uint8_t state, last_error;
    bool valid, ready, interval_valid;
    uint8_t raw[12]; /* exact motion burst: motion,observation,dx LE,dy LE,
                        squal,sum,max,min,shutter HIGH then LOW */
} pmw3901_feedback_t;
typedef struct {
    pmw3901_feedback_t feedback;
    uint32_t last_health_ms;
    bool needs_runtime_baseline;
} pmw3901_device_t;
/* Zero-initialize the device before first use. Call before scheduling or from a task. */
bool pmw3901_device_init(pmw3901_device_t *device);
/* Reads motion or checks the ID. False requires a new init call. */
bool pmw3901_device_service(pmw3901_device_t *device);
#endif
