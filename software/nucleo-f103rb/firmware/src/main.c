#include "FreeRTOS.h"
#include "task.h"

#include "host_uart.h"
#include "host_protocol.h"
#include "actuator_task.h"
#include "board_clock.h"
#include "motor_device.h"
#include "sensor_task.h"

/* Required by newlib-nano when the vendor startup invokes __libc_init_array. */
void _init(void)
{
}

void _fini(void)
{
}

/*
 * Application startup. Hardware drivers and shared RTOS objects are created
 * before the scheduler so no task can observe a partially initialized device.
 */
int main(void)
{
    BaseType_t created;

    board_clock_init();
    host_uart_init();
    configASSERT(motor_device_init());
    configASSERT(actuator_task_init());
    configASSERT(sensor_task_init());

    created = xTaskCreate(host_protocol_task,
                          "host-proto",
                          /* Result FIFO + nested diagnostic serialization. */
                          configMINIMAL_STACK_SIZE * 4U,
                          NULL,
                          tskIDLE_PRIORITY + 3U,
                          NULL);
    configASSERT(created == pdPASS);

    created = xTaskCreate(actuator_task,
                          "actuator",
                          configMINIMAL_STACK_SIZE * 3U,
                          NULL,
                          tskIDLE_PRIORITY + 4U,
                          NULL);
    configASSERT(created == pdPASS);

    created = xTaskCreate(sensor_task,
                          "sensors",
                          configMINIMAL_STACK_SIZE * 2U,
                          NULL,
                          tskIDLE_PRIORITY + 2U,
                          NULL);
    configASSERT(created == pdPASS);

    vTaskStartScheduler();

    for (;;) {
        /* Scheduler starts only if heap allocation and port setup succeed. */
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
    (void) task;
    (void) task_name;

    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}
