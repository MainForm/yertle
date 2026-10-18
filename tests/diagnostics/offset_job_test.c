#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "sensor_task.h"

static jmp_buf end_iteration;
static unsigned mutex_depth, reads, feedback_reads, iterations, stop_after;
static uint32_t tick;
static unsigned calls[YERTLE_MOTOR_COUNT], failures[YERTLE_MOTOR_COUNT];
static motor_device_status_t failure_status[YERTLE_MOTOR_COUNT];
static uint8_t expected_id = 1U;
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void *)1; }
int xSemaphoreTake(SemaphoreHandle_t handle, TickType_t timeout) {
    (void)timeout; assert(handle && mutex_depth==0U); ++mutex_depth; return 1;
}
int xSemaphoreGive(SemaphoreHandle_t handle) {
    assert(handle && mutex_depth==1U); --mutex_depth; return 1;
}
uint32_t xTaskGetTickCount(void) { return tick; }
void vTaskDelayUntil(uint32_t *previous, uint32_t increment) {
    assert(mutex_depth==0U); *previous+=increment; tick=*previous;
    if (++iterations==stop_after) longjmp(end_iteration,1);
}
bool imu_device_init(void) { assert(mutex_depth==0U); return false; }
bool imu_device_read(imu_raw_sample_t *sample) { (void)sample; return false; }
motor_device_status_t motor_device_read_feedback(motor_feedback_set_t *sample) {
    assert(mutex_depth==0U); ++feedback_reads;
    memset(sample,0,sizeof(*sample)); return MOTOR_DEVICE_OK;
}
motor_device_status_t motor_device_read_position_offset(uint8_t id, motor_position_offset_t *offset) {
    assert(mutex_depth==0U && id==expected_id); ++reads;
    const unsigned attempt=++calls[id-1U];
    const bool failed=attempt<=failures[id-1U];
    if (!failed || attempt==SENSOR_POSITION_OFFSET_MAX_ATTEMPTS) ++expected_id;
    if (failed) {
        /* The noncanonical raw must never turn into a measured zero offset. */
        if (failure_status[id-1U]==MOTOR_DEVICE_PROTOCOL_ERROR) offset->raw=0x1800U;
        return failure_status[id-1U];
    }
    offset->raw=id; offset->offset_steps=id; return MOTOR_DEVICE_OK;
}
static void prepare_job(void) {
    memset(calls,0,sizeof calls);
    memset(failures,0,sizeof failures);
    memset(failure_status,0,sizeof failure_status);
    expected_id=1U;
}
static void run(unsigned count) {
    iterations=0U; stop_after=count;
    if (setjmp(end_iteration)==0) sensor_task(NULL);
}
int main(void) {
    motor_position_offsets_t result;
    assert(!sensor_request_position_offsets(1U));
    assert(sensor_task_init());
    assert(!sensor_take_position_offsets(&result));
    assert(sensor_request_position_offsets(42U));
    assert(!sensor_request_position_offsets(43U));
    run(11U);
    assert(reads==11U && feedback_reads==11U && !sensor_take_position_offsets(&result));
    run(1U);
    assert(!sensor_request_position_offsets(43U)); /* completed result not overwritten */
    assert(sensor_take_position_offsets(&result));
    assert(result.request_sequence==42U && result.valid_mask==0xfffU);
    assert(result.motor[11].offset_steps==12 && !sensor_take_position_offsets(&result));
    assert(!sensor_take_position_offsets(NULL));
    prepare_job(); failures[5]=5U; failure_status[5]=MOTOR_DEVICE_UART_ERROR;
    assert(sensor_request_position_offsets(43U)); run(16U);
    assert(sensor_take_position_offsets(&result));
    assert(result.request_sequence==43U && result.valid_mask==(0xfffU & ~(1U<<5U)));
    assert(result.status[5]==MOTOR_DEVICE_UART_ERROR && result.motor[5].raw==0U);
    assert(calls[5]==5U && calls[0]==1U && calls[11]==1U);
    assert(reads==28U && feedback_reads==28U);

    /* Transient protocol errors: use the first valid reply on attempt 3. */
    prepare_job(); failures[4]=2U; failure_status[4]=MOTOR_DEVICE_PROTOCOL_ERROR;
    assert(sensor_request_position_offsets(44U)); run(6U);
    assert(calls[4]==2U && calls[5]==0U && !sensor_take_position_offsets(&result));
    assert(!sensor_request_position_offsets(45U));
    run(8U);
    assert(sensor_take_position_offsets(&result) && result.request_sequence==44U);
    assert(result.valid_mask==0xfffU && calls[4]==3U && calls[5]==1U);
    assert(result.motor[4].raw==5U && result.status[4]==MOTOR_DEVICE_OK);

    /* Success on the fifth attempt must still be accepted. */
    prepare_job(); failures[0]=4U; failure_status[0]=MOTOR_DEVICE_BUSY;
    assert(sensor_request_position_offsets(45U)); run(16U);
    assert(sensor_take_position_offsets(&result) && result.valid_mask==0xfffU && calls[0]==5U);

    /* All 12 fail: exactly 60 attempts, never an endless loop or stale success. */
    prepare_job();
    for (unsigned i=0U;i<YERTLE_MOTOR_COUNT;++i) {
        failures[i]=5U; failure_status[i]=MOTOR_DEVICE_PROTOCOL_ERROR;
    }
    assert(sensor_request_position_offsets(46U)); run(59U);
    assert(!sensor_take_position_offsets(&result)); run(1U);
    assert(sensor_take_position_offsets(&result) && result.request_sequence==46U);
    assert(result.valid_mask==0U);
    for (unsigned i=0U;i<YERTLE_MOTOR_COUNT;++i) {
        assert(calls[i]==5U && result.status[i]==MOTOR_DEVICE_PROTOCOL_ERROR);
        assert(result.motor[i].raw==0x1800U);
    }
    assert(reads==feedback_reads && reads==118U);

    /* A new request starts with fresh retry counters and no stale failed data. */
    prepare_job(); assert(sensor_request_position_offsets(47U)); run(12U);
    assert(sensor_take_position_offsets(&result) && result.valid_mask==0xfffU);
    for (unsigned i=0U;i<YERTLE_MOTOR_COUNT;++i) assert(calls[i]==1U);
    puts("Offset job tests passed: first-valid selection, five-attempt limit, retry reset, partial/all failures, feedback maintained");
    return 0;
}
