#include "rtos_shell.h"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "debug_uart.h"
#include "stm32f1xx.h"

#define SHELL_LINE_MAX 64U

static void shell_prompt(void)
{
    debug_uart_write("f103> ");
}

static void shell_status(void)
{
    char line[80];
    const unsigned long uptime_ms = (unsigned long) xTaskGetTickCount();
    const unsigned long free_heap = (unsigned long) xPortGetFreeHeapSize();
    const unsigned long task_count = (unsigned long) uxTaskGetNumberOfTasks();

    (void) snprintf(line, sizeof(line), "uptime=%lums tasks=%lu heap_free=%luB",
                    uptime_ms, task_count, free_heap);
    debug_uart_write_line(line);
}

static void shell_execute(const char *line)
{
    if (strcmp(line, "help") == 0) {
        debug_uart_write_line("help | status | heap | reboot");
    } else if ((strcmp(line, "status") == 0) || (strcmp(line, "heap") == 0)) {
        shell_status();
    } else if (strcmp(line, "reboot") == 0) {
        debug_uart_write_line("rebooting...");
        vTaskDelay(pdMS_TO_TICKS(10));
        NVIC_SystemReset();
    } else if (*line != '\0') {
        debug_uart_write_line("unknown command; type help");
    }
}

void rtos_shell_task(void *argument)
{
    char line[SHELL_LINE_MAX];
    size_t length = 0U;
    uint8_t byte;

    (void) argument;
    debug_uart_write_line("Yertle NUCLEO-F103RB FreeRTOS shell");
    debug_uart_write_line("type help");
    shell_prompt();

    for (;;) {
        if (!debug_uart_try_read(&byte)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        if ((byte == '\r') || (byte == '\n')) {
            debug_uart_write("\r\n");
            line[length] = '\0';
            shell_execute(line);
            length = 0U;
            shell_prompt();
        } else if ((byte == '\b') || (byte == 0x7FU)) {
            if (length > 0U) {
                --length;
                debug_uart_write("\b \b");
            }
        } else if ((byte >= 0x20U) && (byte <= 0x7EU) && (length < (SHELL_LINE_MAX - 1U))) {
            line[length++] = (char) byte;
            char echo[2] = { (char) byte, '\0' };
            debug_uart_write(echo);
        }
    }
}
