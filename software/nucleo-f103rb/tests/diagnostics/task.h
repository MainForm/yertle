#include <stdint.h>
#ifdef PMW_DEVICE_TEST
#define taskSCHEDULER_NOT_STARTED 1
#define taskSCHEDULER_RUNNING 2
int xTaskGetSchedulerState(void);
#endif
#if defined(DMA_DRIVER_TEST) || defined(SENSOR_TEST) || defined(HOST_PROTOCOL_TEST) || defined(PMW_DEVICE_TEST)
uint32_t xTaskGetTickCount(void);
void vTaskDelay(uint32_t ticks);
void vTaskDelayUntil(uint32_t *previous, uint32_t increment);
#else
static inline uint32_t xTaskGetTickCount(void) { return 1000U; }
#endif
