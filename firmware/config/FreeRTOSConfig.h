#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

/*
 * The startup template leaves the MCU on its reset HSI clock (8 MHz).
 * Change this together with a real board clock-initialisation routine when
 * moving to the planned 72 MHz configuration.
 */
#define configCPU_CLOCK_HZ                      ( ( uint32_t ) 8000000 )
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                     8
#define configMINIMAL_STACK_SIZE                 128
#define configTOTAL_HEAP_SIZE                    ( ( size_t ) ( 16 * 1024 ) )
#define configMAX_TASK_NAME_LEN                  16
#define configUSE_16_BIT_TICKS                   0
#define configUSE_PREEMPTION                     1
#define configUSE_TIME_SLICING                   1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0

#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_MUTEXES                        1
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_QUEUE_SETS                     0
#define configUSE_TASK_NOTIFICATIONS             1
#define configUSE_TRACE_FACILITY                 0
#define configUSE_STATS_FORMATTING_FUNCTIONS     0
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configQUEUE_REGISTRY_SIZE                0

#define configPRIO_BITS                          4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY          ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

#define vPortSVCHandler                          SVC_Handler
#define xPortPendSVHandler                       PendSV_Handler
#define xPortSysTickHandler                      SysTick_Handler

#define INCLUDE_vTaskDelay                       1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_xTaskGetSchedulerState           1

#define configASSERT( x )                        if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ) {} }

#endif /* FREERTOS_CONFIG_H */
