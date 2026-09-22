#include "FreeRTOS.h"
#include "task.h"

#include "host_uart.h"
#include "host_protocol.h"

/* Required by newlib-nano when the vendor startup invokes __libc_init_array. */
void _init(void)
{
}

void _fini(void)
{
}

/*
 * First target-image smoke test.  It proves startup, the STM32F103RB vector
 * table, linker script and scheduler can link together.  Hardware drivers and
 * the actual servo tasks are added after this base image is flashed/verified.
 */
int main(void)
{
    host_uart_init();

    (void) xTaskCreate(host_protocol_task,
                       "host-proto",
                       configMINIMAL_STACK_SIZE * 2U,
                       NULL,
                       tskIDLE_PRIORITY + 1U,
                       NULL);

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
