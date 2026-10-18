#include <stdint.h>
#if defined(DMA_DRIVER_TEST) || defined(SENSOR_TEST) || defined(HOST_PROTOCOL_TEST)
uint32_t xTaskGetTickCount(void);
void vTaskDelay(uint32_t ticks);
void vTaskDelayUntil(uint32_t *previous, uint32_t increment);
#else
static inline uint32_t xTaskGetTickCount(void) { return 1000U; }
#endif
