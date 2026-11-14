#include "FreeRTOS.h"
#include "task.h"

#include "host_uart.h"
#include "host_protocol.h"
#include "actuator_task.h"
#include "motor_control.h"
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
    static pmw3901_device_t optical_flow;
    BaseType_t created;

    board_clock_init();
    (void) pmw3901_device_init(&optical_flow);
    host_uart_init();
    configASSERT(motor_device_init());
    motor_control_init();
    configASSERT(sensor_task_init(&optical_flow.feedback));

    created = xTaskCreate(host_protocol_task,
                          "host-proto",
                          /* Result FIFO, diagnostic serialization and register driver calls. */
                          configMINIMAL_STACK_SIZE * 4U,
                          NULL,
                          tskIDLE_PRIORITY + 3U,
                          NULL);
    configASSERT(created == pdPASS);

    created = xTaskCreate(actuator_task,
                          "actuator",
                          /* ARM .su: actuator376 + service440 + PID344/raw256;
                           * reserve nested motor driver/RTOS calls + margin. */
                          configMINIMAL_STACK_SIZE * 5U,
                          NULL,
                          tskIDLE_PRIORITY + 4U,
                          NULL);
    configASSERT(created == pdPASS);

    created = xTaskCreate(sensor_task,
                          "sensors",
                          /* Motor, IMU and optical acquisition, diagnostics and RTOS headroom. */
                          configMINIMAL_STACK_SIZE * 3U,
                          &optical_flow,
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
