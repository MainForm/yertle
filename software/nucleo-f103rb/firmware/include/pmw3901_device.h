#ifndef YERTLE_PMW3901_DEVICE_H
#define YERTLE_PMW3901_DEVICE_H
#include <stdbool.h>
#include <stdint.h>
enum { PMW_START, PMW_RELEASE_RESET, PMW_CS_LOW, PMW_CS_HIGH, PMW_ID,
       PMW_RESET, PMW_CLEAR, PMW_CONFIG, PMW_DISCARD, PMW_READY, PMW_RETRY };
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
    uint32_t deadline, previous_read_ms, last_health_ms;
    uint16_t config_index;
    uint8_t state, clear_address;
    bool previous_read;
} pmw3901_device_t;
void pmw3901_device_init(pmw3901_device_t *device);
/* One bounded SPI operation per service call, no multi-tick sleeps. */
void pmw3901_device_service(pmw3901_device_t *device,uint32_t now_ms);
#endif
