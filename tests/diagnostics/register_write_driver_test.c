#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "motor_device.h"
#include "motor_uart.h"
#include "uart_diagnostics.h"
#include "semphr.h"
#include "sts3215_protocol.h"

/* Exercise real driver + vendor builders/parser with a fake servo register map.
 * All UART operations require the one bus mutex (never nested). */
static uint8_t memory[256], packet[40];
static size_t packet_length;
static unsigned mutex_depth, operations, target_writes, unlock_writes, restore_writes;
static int fault_at=-1, cancel_at=-1;
enum fault_kind { NONE, RX_TIMEOUT, TX_TIMEOUT, CHECKSUM, WRONG_ID, WRONG_LENGTH, SERVO, MISMATCH };
static enum fault_kind fault;
static bool lock_available=true, safe=true, ignore_restore;
static uint8_t target_address;
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void *)1; }
int xSemaphoreTake(SemaphoreHandle_t handle, TickType_t timeout)
{ (void)timeout; assert(handle && mutex_depth==0U); if (!lock_available) return 0; ++mutex_depth; return 1; }
int xSemaphoreGive(SemaphoreHandle_t handle)
{ assert(handle && mutex_depth==1U); --mutex_depth; return 1; }
uint32_t xTaskGetTickCount(void) { return 100U; }
void vTaskDelay(uint32_t ticks) { assert(mutex_depth==1U && ticks==10U); }
void motor_uart_init(void) {}
void motor_uart_discard_rx(void) {}
static bool allowed(void) { return safe; }
void motor_diagnostics_record(enum motor_diag_reason reason) { (void)reason; }
void motor_transport_record(enum motor_transport_event event) { (void)event; }
void motor_diagnostics_masks(uint16_t motion, uint16_t current) { (void)motion; (void)current; }
bool motor_uart_write(const uint8_t *data, size_t length, uint32_t timeout_ms)
{
    assert(mutex_depth==1U && timeout_ms==5U && length>=8U && data[2]>=1U && data[2]<=12U);
    unsigned sum=0U; for (size_t i=2U;i<length;++i) sum+=data[i]; assert((sum & 255U)==255U);
    const unsigned operation=operations++;
    const bool fail=(int)operation==fault_at;
    const uint8_t address=data[5]; uint8_t n=0U;
    if (data[4]==2U) {
        n=data[6]; assert(length==8U);
    } else {
        assert(data[4]==3U); /* No REG_WRITE, SYNC_WRITE, ACTION or broadcast. */
        n=(uint8_t)(length-7U);
        if (address==target_address) ++target_writes;
        else { assert(address==55U && n==1U); if (data[6]==0U) ++unlock_writes; else ++restore_writes; }
        if (!(ignore_restore && address==55U && data[6]==1U) && !(fail && fault==MISMATCH))
            memcpy(memory+address,data+6U,n);
        n=0U;
    }
    if ((int)operation==cancel_at) safe=false;
    if (fail && fault==TX_TIMEOUT) return false; /* May already have applied. */
    if (fail && fault==RX_TIMEOUT) { packet_length=0U; return true; }
    packet[0]=packet[1]=255U; packet[2]=data[2]; packet[3]=(uint8_t)(n+2U); packet[4]=0U;
    if (n!=0U) memcpy(packet+5U,memory+address,n);
    packet_length=n+6U;
    if (fail && fault==WRONG_ID) ++packet[2];
    if (fail && fault==WRONG_LENGTH) {
        const uint8_t wrong=n==0U ? 1U : 0U;
        packet[3]=(uint8_t)(wrong+2U); packet[5]=0U; packet_length=wrong+6U;
    }
    if (fail && fault==SERVO) packet[4]=4U;
    sum=0U; for (size_t i=2U;i<packet_length-1U;++i) sum+=packet[i];
    packet[packet_length-1U]=(uint8_t)~sum;
    if (fail && fault==CHECKSUM) packet[packet_length-1U]^=1U;
    return true;
}
bool motor_uart_read_frame(uint8_t *frame, size_t capacity, size_t *length, uint32_t timeout_ms)
{
    assert(mutex_depth==1U && timeout_ms==5U);
    if (packet_length==0U) return false;
    assert(capacity>=packet_length); memcpy(frame,packet,packet_length); *length=packet_length; return true;
}
static motor_register_write_request_t setup(uint8_t address, uint16_t value, bool locked)
{
    assert(mutex_depth==0U && motor_device_register_lock_uncertain_mask()==0U);
    memset(memory,0,sizeof(memory)); memory[8]=1U; memory[55]=locked ? 1U : 0U;
    operations=target_writes=unlock_writes=restore_writes=0U;
    safe=lock_available=true; ignore_restore=false; fault_at=cancel_at=-1; fault=NONE;
    target_address=address;
    motor_register_write_request_t request={.motor_id=1U,.address=address,
        .length=(address==16U || address==48U) ? 2U : 1U,.options=1U};
    request.data[0]=(uint8_t)value; request.data[1]=(uint8_t)(value>>8U);
    return request;
}
static void clear_lock_latch(void)
{
    /* External tool restored the lock; a fresh GET confirms the saved value. */
    memory[55]=1U; fault_at=-1; ignore_restore=false;
    motor_register_data_t data;
    assert(motor_device_read_register(1U,55U,1U,&data)==MOTOR_REGISTER_OK);
    assert(motor_device_register_lock_uncertain_mask()==0U);
}
int main(void)
{
    assert(motor_device_init());
    motor_register_write_request_t request=setup(21U,32U,true);
    motor_register_write_report_t result=motor_device_write_register(&request,allowed);
    assert(result.status==0U && result.primary_status==0U && result.target_write_attempted);
    assert(memory[21]==32U && memory[55]==1U && target_writes==1U && unlock_writes==1U && restore_writes==1U);
    assert(result.lock_restore_verified && result.lock_restore_attempted && operations==11U);
    motor_register_write_diagnostics_t diagnostics;
    assert(motor_device_get_register_write_diagnostics(&diagnostics));
    assert(diagnostics.motor_id==1U && diagnostics.address==21U && diagnostics.report.status==0U);
    assert(!motor_device_get_register_write_diagnostics(NULL));

    /* Exact-width allow-list and boundaries, unrelated writable registers denied. */
    const uint8_t addresses[]={16U,21U,22U,23U,24U,48U};
    for (unsigned i=0U;i<sizeof(addresses);++i) {
        const uint16_t maximum=(addresses[i]==16U || addresses[i]==48U) ? 1000U : 254U;
        request=setup(addresses[i],maximum,true);
        assert(motor_register_validate_write(&request)==MOTOR_WRITE_OK);
        request.data[0]=(uint8_t)(maximum+1U); request.data[1]=(uint8_t)((maximum+1U)>>8U);
        assert(motor_register_validate_write(&request)==MOTOR_WRITE_INVALID_REGISTER);
        request.data[0]=0U; request.data[1]=0U; ++request.length;
        assert(motor_register_validate_write(&request)==MOTOR_WRITE_INVALID_REGISTER);
    }
    request=setup(21U,8U,true); request.options=0U;
    assert(motor_device_write_register(&request,allowed).status==MOTOR_WRITE_FORBIDDEN && operations==0U);
    request.options=2U; assert(motor_register_validate_write(&request)==MOTOR_WRITE_INVALID_REGISTER);
    request.options=1U;
    for (unsigned address=0U;address<256U;++address) {
        request.address=(uint8_t)address;
        if (address!=16U && address!=21U && address!=22U && address!=23U && address!=24U && address!=48U)
            assert(motor_register_validate_write(&request)==MOTOR_WRITE_FORBIDDEN);
    }
    request=setup(48U,500U,false); request.options=0U;
    result=motor_device_write_register(&request,allowed);
    assert(result.status==0U && target_writes==1U && unlock_writes==0U && restore_writes==0U);
    assert(memory[48]==0xf4U && memory[49]==1U);
    request=setup(24U,254U,true); memory[25]=91U;
    result=motor_device_write_register(&request,allowed);
    assert(result.status==0U && target_writes==1U && memory[24]==254U && memory[25]==91U);
    assert(memory[55]==1U && result.lock_restore_verified);
    request=setup(24U,16U,true); request.options=0U;
    assert(motor_device_write_register(&request,allowed).status==13U && operations==0U);
    request.options=1U; request.length=2U;
    assert(motor_device_write_register(&request,allowed).status==12U && operations==0U);
    request.length=1U; request.data[0]=255U;
    assert(motor_device_write_register(&request,allowed).status==12U && operations==0U);
    request=setup(23U,8U,false); memory[8]=0U; /* No WRITE response, READ still mandatory. */
    result=motor_device_write_register(&request,allowed);
    assert(result.status==0U && memory[23]==8U && memory[55]==0U && target_writes==1U && unlock_writes==0U);
    request=setup(21U,32U,true); memory[21]=32U;
    assert(motor_device_write_register(&request,allowed).status==0U && target_writes==0U && unlock_writes==0U);
    request=setup(21U,32U,true); memory[40]=1U;
    assert(motor_device_write_register(&request,allowed).status==5U && target_writes==0U && unlock_writes==0U);
    request=setup(21U,32U,true); lock_available=false;
    assert(motor_device_write_register(&request,allowed).status==11U && operations==0U); lock_available=true;
    safe=false; assert(motor_device_write_register(&request,allowed).status==5U && operations==0U);

    /* Fault injection at every normal transaction step, including cleanup.
     * No outcome may repeat the target write or silently leave the lock open. */
    for (int kind=RX_TIMEOUT;kind<=SERVO;++kind) for (int op=0;op<11;++op) {
        request=setup(21U,32U,true); fault=(enum fault_kind)kind; fault_at=op;
        result=motor_device_write_register(&request,allowed);
        assert(target_writes<=1U && mutex_depth==0U);
        assert(memory[55]==1U || motor_device_register_lock_uncertain_mask()==1U);
        if (result.status==0U) assert(memory[21]==32U && memory[55]==1U);
        if (motor_device_register_lock_uncertain_mask()!=0U) clear_lock_latch();
    }
    request=setup(21U,32U,true); fault=MISMATCH; fault_at=6;
    result=motor_device_write_register(&request,allowed);
    assert(result.status==15U && memory[55]==1U && target_writes==1U);
    request=setup(21U,32U,true); fault=SERVO; fault_at=6;
    result=motor_device_write_register(&request,allowed);
    assert(result.status==16U && result.servo_error==4U && memory[55]==1U);
    request=setup(21U,32U,true); cancel_at=4; /* Stop after unlock, before target. */
    result=motor_device_write_register(&request,allowed);
    assert(result.status==5U && target_writes==0U && memory[55]==1U && restore_writes==1U);
    request=setup(21U,32U,true); ignore_restore=true;
    result=motor_device_write_register(&request,allowed);
    assert(result.status==17U && result.primary_status==0U && memory[55]==0U);
    assert(motor_device_register_lock_uncertain_mask()==1U);
    /* A new explicit request recovers the ORIGINAL lock, not the observed zero. */
    ignore_restore=false; request.data[0]=33U;
    result=motor_device_write_register(&request,allowed);
    assert(result.status==0U && memory[55]==1U && memory[21]==33U && motor_device_register_lock_uncertain_mask()==0U);
    puts("register write driver: policy, torque, WRITE/read-back, lock cleanup, all-stage I/O faults and no target retry passed");
    return 0;
}
