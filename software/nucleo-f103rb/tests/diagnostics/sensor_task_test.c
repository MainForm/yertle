#include "sensor_task.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <setjmp.h>

static TickType_t tick;
static unsigned cycles, initializations, services;
static bool cache_locked;
static jmp_buf finished;

TickType_t xTaskGetTickCount(void)
{
    return tick;
}

SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    return &cache_locked;
}

int xSemaphoreTake(SemaphoreHandle_t handle, TickType_t timeout)
{
    (void)timeout;
    assert(handle == &cache_locked && !cache_locked);
    cache_locked = true;
    return pdTRUE;
}

int xSemaphoreGive(SemaphoreHandle_t handle)
{
    assert(handle == &cache_locked && cache_locked);
    cache_locked = false;
    return pdTRUE;
}

motor_device_status_t motor_device_read_feedback(motor_feedback_set_t *sample)
{
    assert(!cache_locked);
    ++cycles;
    sample->motor[0].valid = true;
    return MOTOR_DEVICE_OK;
}

bool imu_device_init(void)
{
    return true;
}

bool imu_device_read(imu_raw_sample_t *sample)
{
    (void)sample;
    assert(!cache_locked);
    return true;
}

bool pmw3901_device_init(pmw3901_device_t *device)
{
    motor_feedback_set_t motor;
    imu_feedback_t imu;
    assert(sensor_get_feedback_snapshot(&motor, &imu));
    assert(motor.sequence == cycles && motor.motor[0].valid);
    assert(imu.sequence == cycles && imu.valid);
    ++initializations;
    assert((initializations == 1U && tick == 0U) ||
           (initializations == 2U && tick == 1120U && cycles == 53U));
    tick += 85U;  /* Synchronous initialization crosses four sensor periods. */
    ++device->feedback.epoch;
    device->feedback.state = PMW_READY;
    device->feedback.ready = true;
    return true;
}

bool pmw3901_device_service(pmw3901_device_t *device)
{
    assert(!cache_locked);
    ++services;
    if (services == 1U) {
        assert(tick == 100U);
        tick += 2U;
        device->feedback.ready = false;
        device->feedback.state = PMW_RETRY;
        return false;
    }
    assert(services == 2U && tick == 1220U);
    device->feedback.valid = true;
    return true;
}

void vTaskDelayUntil(TickType_t *previous, TickType_t increment)
{
    motor_feedback_set_t motor;
    imu_feedback_t imu;
    pmw3901_feedback_t flow;
    assert(increment == 20U && *previous + increment > tick);
    assert(sensor_get_extended_feedback_snapshot(&motor, &imu, &flow));
    assert(motor.sequence == cycles && imu.sequence == cycles);
    if (services == 1U && initializations == 1U) {
        assert(!flow.ready && flow.state == PMW_RETRY);
    }
    if (services == 2U) {
        assert(flow.ready && flow.valid && flow.epoch == 2U);
        longjmp(finished, 1);
    }
    *previous += increment;
    tick = *previous;
}

int main(void)
{
    assert(sensor_task_init());
    if (setjmp(finished) == 0) {
        sensor_task(NULL);
        assert(0);
    }
    assert(cycles == 54U);
    return 0;
}
