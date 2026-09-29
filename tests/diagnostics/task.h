#include <stdint.h>
#ifdef DMA_DRIVER_TEST
uint32_t xTaskGetTickCount(void);
void vTaskDelay(uint32_t ticks);
#else
static inline uint32_t xTaskGetTickCount(void) { return 1000U; }
#endif
