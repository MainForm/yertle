#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Production parser, FSM and host maintenance dispatch, with fake motor I/O. */
#include "../../firmware/src/host_protocol.c"

static TickType_t tick;
static unsigned read_calls, tx_calls, write_calls;
static motor_register_write_request_t last_write;
static bool tx_success = true;
static motor_register_status_t read_status;
static uint8_t response_type, response[64];
static size_t response_length;
static uint16_t uncertain_mask;
static motor_register_write_status_t write_status;
static uint8_t diagnostic[64];static unsigned diagnostic_count;
uint16_t motor_device_register_lock_uncertain_mask(void) { return uncertain_mask; }
motor_register_write_report_t motor_device_write_register(const motor_register_write_request_t *request,
    bool (*allowed)(void)) {
    assert(allowed() && motor_register_validate_write(request)==MOTOR_WRITE_OK);
    last_write = *request;
    ++write_calls;
    motor_register_write_report_t r=motor_register_initial_report();r.status=write_status;
    r.lock_expected=1U;r.lock_actual=write_status==17U ? 0U : 1U;
    r.lock_read_status=0U;r.lock_attempts=2U;r.lock_elapsed_ms=17U;
    r.first_lock_failure=3U;r.first_lock_actual=0U;
    return r;
}

uint32_t xTaskGetTickCount(void) { return tick; }
void vTaskDelay(uint32_t ticks) { tick+=ticks; }
void vTaskDelayUntil(uint32_t *previous, uint32_t increment)
{ *previous+=increment; tick=*previous; }
bool host_uart_try_read(uint8_t *byte, bool *discontinuity)
{ (void)byte; *discontinuity=false; return false; }
bool host_uart_write_bytes(const uint8_t *data, size_t length)
{
    uint8_t raw[YERTLE_PROTOCOL_MAX_RAW]; size_t size; uint16_t sequence;
    assert(data[0]==0U && data[length-1U]==0U);
    assert(cobs_decode(data+1U,length-2U,raw,&size));
    assert(validate_frame(raw,size,&response_type,&sequence));
    response_length=size-YERTLE_PROTOCOL_HEADER_SIZE-YERTLE_PROTOCOL_CRC_SIZE;
    memcpy(response,raw+YERTLE_PROTOCOL_HEADER_SIZE,response_length);
    if(response_type==0x90U){assert(response_length==64U);memcpy(diagnostic,response,64U);++diagnostic_count;}
    ++tx_calls; return tx_success;
}
bool sensor_get_extended_feedback_snapshot(motor_feedback_set_t *motor, imu_feedback_t *imu, pmw3901_feedback_t *flow)
{ (void)motor; (void)imu; (void)flow; return false; }

static unsigned offset_calls[YERTLE_MOTOR_COUNT];
static unsigned offset_failures[YERTLE_MOTOR_COUNT];
motor_device_status_t motor_device_read_position_offset(uint8_t id, motor_position_offset_t *offset)
{
    assert(register_access_allowed() && id >= 1U && id <= YERTLE_MOTOR_COUNT);
    if (++offset_calls[id - 1U] <= offset_failures[id - 1U]) {
        offset->raw = 0x1800U;
        return MOTOR_DEVICE_PROTOCOL_ERROR;
    }
    offset->raw = id;
    offset->offset_steps = id;
    return MOTOR_DEVICE_OK;
}
motor_register_status_t motor_device_read_register(uint8_t id, uint8_t address,
    uint8_t length, motor_register_data_t *result)
{
    assert(host_protocol_get_state()==HOST_FSM_DISARMED);
    assert(!host_protocol_is_stop_requested() && motor_register_request_valid(id,address,length));
    ++read_calls;
    result->servo_error=read_status==MOTOR_REGISTER_SERVO_ERROR ? 4U : 0U;
    for (unsigned i=0U;i<length;++i) result->data[i]=(uint8_t)(0x80U+i);
    return read_status;
}
bool motor_device_validate_targets(const motor_target_set_t *targets)
{ (void)targets; return true; }
bool motor_control_submit_targets(const motor_target_set_t *targets)
{ (void)targets; assert(0); return false; }
void uart_diagnostics_send(void *context, uart_diag_sender_t send)
{ (void)context; (void)send; }
void uart_diagnostics_send_motor_buses(void *c, uart_diag_sender_t s,uint16_t q){(void)c;(void)s;(void)q;}

static protocol_context_t reset(void)
{
    memset(offset_calls, 0, sizeof offset_calls);
    memset(offset_failures, 0, sizeof offset_failures);
    last_write = (motor_register_write_request_t){0};
    published_state=HOST_FSM_DISARMED;
    pending_safety_action=HOST_SAFETY_ACTION_NONE;
    requested_fault_reason=HOST_FAULT_NONE;
    requested_target_timeout=safe_stop_completed=false;
    tick=100U; read_calls=tx_calls=write_calls=0U; response_length=0U;
    diagnostic_count=0U;
    uncertain_mask=0U; write_status=MOTOR_WRITE_OK;
    read_status=MOTOR_REGISTER_OK; tx_success=true;
    protocol_context_t context={0}; context.state=HOST_FSM_DISARMED;
    return context;
}
static void request(protocol_context_t *context, uint16_t sequence,
    uint8_t id, uint8_t address, uint8_t length, uint8_t payload_length)
{
    uint8_t raw[16]={1U,MESSAGE_GET_MOTOR_REGISTER,0U};
    raw[3]=(uint8_t)sequence; raw[4]=(uint8_t)(sequence>>8U);
    raw[5]=payload_length; raw[7]=id; raw[8]=address; raw[9]=length;
    const size_t size=YERTLE_PROTOCOL_HEADER_SIZE+payload_length+YERTLE_PROTOCOL_CRC_SIZE;
    const uint16_t crc=crc16_ccitt(raw,size-2U);
    raw[size-2U]=(uint8_t)crc; raw[size-1U]=(uint8_t)(crc>>8U);
    handle_frame(context,raw,size);
}
static void check_nack(uint8_t code)
{
    assert(tx_calls==1U && response_type==MESSAGE_NACK && response_length==4U);
    assert(response[0]==MESSAGE_GET_MOTOR_REGISTER && response[3]==code);
    assert(read_calls==0U);
}
static void check_reply(uint16_t sequence, uint8_t id, uint8_t address,
    uint8_t length, motor_register_status_t status)
{
    assert(response_type==MESSAGE_MOTOR_REGISTER);
    assert(response[0]==(uint8_t)sequence && response[1]==(uint8_t)(sequence>>8U));
    assert(response[2]==id && response[3]==address && response[4]==length && response[5]==status);
    assert(response_length==7U+(status==MOTOR_REGISTER_OK ? length : 0U));
    assert(response[6]==(status==MOTOR_REGISTER_SERVO_ERROR ? 4U : 0U));
    if (status==MOTOR_REGISTER_OK)
        for (unsigned i=0U;i<length;++i) assert(response[7U+i]==0x80U+i);
}

static void set_payload(protocol_context_t *context, uint16_t sequence, const uint8_t *payload, uint8_t length)
{
    uint8_t raw[YERTLE_PROTOCOL_MAX_RAW]={1U,MESSAGE_SET_MOTOR_REGISTER,0U};
    raw[3]=(uint8_t)sequence; raw[4]=(uint8_t)(sequence>>8U); raw[5]=length;
    memcpy(raw+7U,payload,length);
    const size_t size=9U+length;
    const uint16_t crc=crc16_ccitt(raw,size-2U);
    raw[size-2U]=(uint8_t)crc; raw[size-1U]=(uint8_t)(crc>>8U);
    uint8_t encoded[YERTLE_PROTOCOL_MAX_ENCODED], decoded[YERTLE_PROTOCOL_MAX_RAW];
    size_t decoded_length;
    const size_t encoded_length=cobs_encode(raw,size,encoded);
    assert(cobs_decode(encoded,encoded_length,decoded,&decoded_length));
    assert(decoded_length==size && memcmp(raw,decoded,size)==0);
    handle_frame(context,decoded,decoded_length);
}
static void check_set_result(uint16_t sequence, uint8_t code)
{
    assert(response_type==(code==0U ? MESSAGE_ACK : MESSAGE_NACK) && response_length==4U);
    assert(response[0]==0x0dU && response[1]==(uint8_t)sequence && response[2]==(uint8_t)(sequence>>8U));
    assert(response[3]==code);
}
static void check_position_offsets(void)
{
    protocol_context_t context = reset();
    uint8_t raw[] = { 1U, MESSAGE_GET_POSITION_OFFSETS, 0U, 47U, 0U, 0U, 0U, 0U, 0U };
    ypwm_put16(raw + 7U, crc16_ccitt(raw, 7U));
    handle_frame(&context, raw, sizeof raw);
    assert(response_type == MESSAGE_ACK && context.offsets_job_active && offset_calls[0] == 0U);
    context.host_connected = context.lease_established = true;
    assert(!can_arm(&context));
    context.host_connected = context.lease_established = false;

    offset_failures[4] = 2U;
    offset_failures[5] = YERTLE_OFFSET_MAX_ATTEMPTS;
    for (unsigned i = 0; i < 18U; ++i) {
        service_position_offsets(&context);
        service_position_offsets(&context); /* Same tick cannot start another read. */
        tick += YERTLE_OFFSET_ATTEMPT_PERIOD_MS;
        assert(context.offsets_job_active == (i != 17U));
    }
    assert(response_type == MESSAGE_POSITION_OFFSETS && response_length == 56U);
    assert(ypwm_u16(response) == 47U && ypwm_u16(response + 6U) == (YPWM_ALL_MASK & ~(1U << 5U)));
    assert(offset_calls[4] == 3U && ypwm_u16(response + 8U + 4U * 4U) == 5U);
    assert(offset_calls[5] == 5U && ypwm_u16(response + 8U + 5U * 4U) == 0x1800U);
    assert(response[8U + 5U * 4U + 3U] == MOTOR_DEVICE_PROTOCOL_ERROR);
    assert(offset_calls[11] == 1U);

    context = reset();
    assert(start_position_offsets(&context, 48U));
    service_position_offsets(&context);
    host_protocol_latch_fault(HOST_FAULT_DRIVER);
    service_position_offsets(&context);
    assert(!context.offsets_job_active && offset_calls[1] == 0U);
    assert(response_type == MESSAGE_POSITION_OFFSETS && ypwm_u16(response) == 48U);
    assert(ypwm_u16(response + 6U) == 1U && response[15U] == MOTOR_DEVICE_BUSY);

    context = reset();
    assert(start_position_offsets(&context, 49U));
    for (unsigned i = 0; i < YERTLE_MOTOR_COUNT; ++i)
        offset_failures[i] = YERTLE_OFFSET_MAX_ATTEMPTS;
    for (unsigned i = 0; i < YERTLE_MOTOR_COUNT * YERTLE_OFFSET_MAX_ATTEMPTS; ++i) {
        service_position_offsets(&context);
        tick += YERTLE_OFFSET_ATTEMPT_PERIOD_MS;
    }
    assert(!context.offsets_job_active && ypwm_u16(response + 6U) == 0U);
    for (unsigned i = 0; i < YERTLE_MOTOR_COUNT; ++i)
        assert(offset_calls[i] == YERTLE_OFFSET_MAX_ATTEMPTS);

    /* A second job in the same context starts from ID 1 and accepts the
     * fifth attempt as valid. Previous failures do not leak into its reply. */
    memset(offset_calls, 0, sizeof offset_calls);
    memset(offset_failures, 0, sizeof offset_failures);
    offset_failures[0] = YERTLE_OFFSET_MAX_ATTEMPTS - 1U;
    assert(start_position_offsets(&context, 50U));
    for (unsigned i = 0; i < YERTLE_MOTOR_COUNT + YERTLE_OFFSET_MAX_ATTEMPTS - 1U; ++i) {
        service_position_offsets(&context);
        tick += YERTLE_OFFSET_ATTEMPT_PERIOD_MS;
    }
    assert(!context.offsets_job_active && ypwm_u16(response) == 50U);
    assert(ypwm_u16(response + 6U) == YPWM_ALL_MASK && offset_calls[0] == YERTLE_OFFSET_MAX_ATTEMPTS);
    assert(offset_calls[11] == 1U && ypwm_u16(response + 52U) == 12U);
}

int main(void)
{
    protocol_context_t context=reset();
    /* Full 32-byte raw response, no ACK, no UART work in host handler. */
    request(&context,0x1234U,12U,224U,32U,3U);
    assert(tx_calls==0U && read_calls==0U && context.register_job_active);
    assert(!context.host_connected && !context.lease_established && context.last_host_heartbeat_tick==0U);
    assert(!start_position_offsets(&context, 2U));
    service_motor_register(&context);
    assert(tx_calls==1U && !context.register_job_active);
    check_reply(0x1234U,12U,224U,32U,MOTOR_REGISTER_OK);
    assert(published_state==HOST_FSM_DISARMED && pending_safety_action==HOST_SAFETY_ACTION_NONE);

    for (unsigned status=0U;status<=4U;++status) {
        context=reset(); read_status=(motor_register_status_t)status;
        request(&context,42U,1U,255U,1U,3U);
        service_motor_register(&context);
        assert(read_calls==1U && tx_calls==1U);
        check_reply(42U,1U,255U,1U,read_status);
    }
    const uint8_t invalid[][3]={{0,48,2},{13,48,2},{255,48,2},{1,48,0},
        {1,48,33},{1,0,255},{1,225,32},{1,255,2}};
    for (unsigned i=0U;i<sizeof(invalid)/sizeof(invalid[0]);++i) {
        context=reset(); request(&context,7U,invalid[i][0],invalid[i][1],invalid[i][2],3U);
        check_nack(ERROR_INVALID_REGISTER);
    }
    const uint8_t wrong_lengths[]={0U,2U,4U};
    for (unsigned i=0U;i<3U;++i) {
        context=reset(); request(&context,7U,1U,48U,2U,wrong_lengths[i]);
        check_nack(ERROR_INVALID_LENGTH);
    }
    const host_fsm_state_t rejected[]={HOST_FSM_BOOT,HOST_FSM_ARMED,HOST_FSM_FAULT};
    for (unsigned i=0U;i<3U;++i) {
        context=reset(); context.state=published_state=rejected[i];
        request(&context,7U,1U,48U,2U,3U); check_nack(ERROR_INVALID_STATE);
    }
    context=reset(); pending_safety_action=HOST_SAFETY_ACTION_SAFE_STOP;
    request(&context,7U,1U,48U,2U,3U); check_nack(ERROR_INVALID_STATE);

    /* BUSY response never overwrites the original pending job. */
    context=reset(); request(&context,1U,1U,48U,2U,3U);
    request(&context,2U,12U,255U,1U,3U);
    check_reply(2U,12U,255U,1U,MOTOR_REGISTER_BUSY);
    service_motor_register(&context);
    check_reply(1U,1U,48U,2U,MOTOR_REGISTER_OK);
    assert(read_calls==1U && tx_calls==2U);
    context=reset(); assert(start_position_offsets(&context, 9U));
    request(&context,3U,1U,48U,2U,3U);
    check_reply(3U,1U,48U,2U,MOTOR_REGISTER_BUSY);
    assert(!context.register_job_active && context.offsets_job_active);

    /* A new lease cannot ARM until the read and result delivery finish. */
    context=reset(); context.host_connected=context.lease_established=true;
    request(&context,5U,1U,48U,2U,3U);
    try_enter_control_ready(&context); assert(context.state==HOST_FSM_DISARMED);
    service_motor_register(&context); assert(can_arm(&context));
    assert(context.last_host_heartbeat_tick==0U);

    /* A stop/fault after acceptance cancels motor I/O, not the response. */
    context=reset(); request(&context,6U,1U,48U,2U,3U);
    host_protocol_latch_fault(HOST_FAULT_DRIVER);
    service_motor_register(&context);
    assert(read_calls==0U); check_reply(6U,1U,48U,2U,MOTOR_REGISTER_BUSY);
    context=reset(); request(&context,6U,1U,48U,2U,3U);
    published_state=HOST_FSM_ARMED;
    service_motor_register(&context);
    assert(read_calls==0U); check_reply(6U,1U,48U,2U,MOTOR_REGISTER_BUSY);

    /* Lost host response is bounded; only PC timeout causes a new query. */
    context=reset(); request(&context,8U,1U,48U,2U,3U);
    tx_success=false; service_motor_register(&context);
    service_motor_register(&context);
    assert(read_calls==1U && tx_calls==1U && !context.register_job_active);
    /* Document's payload golden vector; completion ACK, never acceptance ACK. */
    const uint8_t golden[]={1U,0x15U,1U,1U,0x20U};
    context=reset(); set_payload(&context,0x1234U,golden,sizeof(golden));
    assert(context.register_job_active && write_calls==0U && tx_calls==0U);
    assert(!start_position_offsets(&context, 9U));
    assert(!context.host_connected && !context.lease_established && context.last_host_heartbeat_tick==0U);
    set_payload(&context,0x1234U,golden,sizeof(golden)); /* In-flight duplicate. */
    assert(tx_calls==0U && write_calls==0U);
    service_motor_register_write(&context);
    check_set_result(0x1234U,0U);
    assert(diagnostic_count==1U && diagnostic[0]==2U && diagnostic[2]==0x34U && diagnostic[3]==0x12U);
    assert(diagnostic[4]==1U && diagnostic[5]==21U && diagnostic[16]==2U && diagnostic[24]==17U);
    assert(!context.register_job_active && pending_safety_action==HOST_SAFETY_ACTION_NONE);
    set_payload(&context,0x1234U,golden,sizeof(golden)); check_set_result(0x1234U,0U);
    assert(diagnostic_count==2U); /* Replay is correlated, no second motor job. */
    assert(write_calls==1U && !context.register_job_active);
    uint8_t altered[5]; memcpy(altered,golden,5U); ++altered[4];
    set_payload(&context,0x1234U,altered,5U); check_set_result(0x1234U,12U);
    assert(write_calls==1U);

    const uint8_t startup_torque[]={1U,0x18U,1U,1U,16U};
    context=reset(); set_payload(&context,7U,startup_torque,sizeof(startup_torque));
    service_motor_register_write(&context);
    assert(last_write.address == 0x18U && last_write.length == 1U && last_write.data[0] == 16U);
    check_set_result(7U,0U); assert(write_calls==1U);

    const uint8_t integral_limit[]={1U,0x19U,1U,1U,16U};
    context=reset(); set_payload(&context,7U,integral_limit,sizeof(integral_limit));
    service_motor_register_write(&context);
    assert(last_write.address == 0x19U && last_write.length == 1U && last_write.data[0] == 16U);
    check_set_result(7U,0U); assert(write_calls==1U);
    uint8_t invalid_integral[][6]={
        {1U,0x19U,1U,0U,16U,0U}, {1U,0x19U,2U,1U,16U,0U}, {1U,0x19U,1U,1U,255U,0U}
    };
    const uint8_t integral_errors[]={13U,12U,12U};
    for (unsigned i=0U;i<3U;++i) {
        context=reset();
        set_payload(&context,7U,invalid_integral[i],(uint8_t)(4U+invalid_integral[i][2]));
        check_set_result(7U,integral_errors[i]); assert(write_calls==0U && !context.register_job_active);
    }

    const uint8_t outcomes[]={0U,5U,11U,12U,13U,14U,15U,16U,17U};
    for (unsigned i=0U;i<sizeof(outcomes);++i) {
        context=reset(); write_status=(motor_register_write_status_t)outcomes[i];
        set_payload(&context,8U,golden,5U);
        service_motor_register_write(&context); check_set_result(8U,outcomes[i]);
        set_payload(&context,8U,golden,5U); check_set_result(8U,outcomes[i]);
        assert(write_calls==1U);
    }
    uint8_t invalid_set[][6]={
        {0U,21U,1U,1U,32U,0U}, {13U,21U,1U,1U,32U,0U}, {254U,21U,1U,1U,32U,0U},
        {1U,21U,1U,2U,32U,0U}, {1U,21U,1U,1U,255U,0U}, {1U,255U,2U,1U,0U,0U},
        {1U,16U,2U,1U,0xe9U,3U}
    };
    for (unsigned i=0U;i<sizeof(invalid_set)/sizeof(invalid_set[0]);++i) {
        context=reset(); set_payload(&context,9U,invalid_set[i],(uint8_t)(4U+invalid_set[i][2]));
        check_set_result(9U,12U); assert(write_calls==0U && !context.register_job_active);
    }
    const uint8_t calibration[]={10U,0x28U,1U,1U,128U};
    context=reset();context.state=published_state=HOST_FSM_ARMED;
    set_payload(&context,9U,calibration,sizeof(calibration));check_set_result(9U,5U);
    assert(!context.register_job_active);
    context=reset();
    set_payload(&context,9U,calibration,sizeof(calibration));
    service_motor_register_write(&context); check_set_result(9U,0U);
    assert(last_write.motor_id == 10U && last_write.address == 0x28U);
    set_payload(&context,9U,calibration,sizeof(calibration));check_set_result(9U,0U);
    assert(write_calls==1U);

    context=reset();uint8_t forbidden[]={1U,40U,1U,0U,1U};
    set_payload(&context,9U,forbidden,5U); check_set_result(9U,12U);
    forbidden[1]=21U; set_payload(&context,9U,forbidden,5U); check_set_result(9U,13U);
    context=reset(); set_payload(&context,9U,golden,4U); check_set_result(9U,2U);
    const uint8_t zero_length[]={1U,21U,0U,1U};
    set_payload(&context,9U,zero_length,4U); check_set_result(9U,12U);
    for (unsigned i=0U;i<3U;++i) {
        context=reset(); context.state=published_state=rejected[i];
        set_payload(&context,9U,golden,5U); check_set_result(9U,5U); assert(write_calls==0U);
    }
    context=reset(); pending_safety_action=HOST_SAFETY_ACTION_SAFE_STOP;
    set_payload(&context,9U,golden,5U); check_set_result(9U,5U);
    context=reset(); assert(start_position_offsets(&context, 9U));
    set_payload(&context,10U,golden,5U); check_set_result(10U,11U);
    context=reset(); request(&context,9U,1U,48U,2U,3U);
    set_payload(&context,10U,golden,5U); check_set_result(10U,11U);
    context=reset(); set_payload(&context,9U,golden,5U);
    request(&context,10U,1U,48U,2U,3U); check_reply(10U,1U,48U,2U,MOTOR_REGISTER_BUSY);
    context=reset(); set_payload(&context,9U,golden,5U);
    host_protocol_latch_fault(HOST_FAULT_DRIVER);
    service_motor_register_write(&context);
    check_set_result(9U,5U); assert(write_calls==0U);

    /* A full result queue defers motor I/O; TX failure never reruns a write. */
    context=reset(); set_payload(&context,9U,golden,5U);
    context.result_count=YERTLE_RESULT_QUEUE_CAPACITY;
    service_motor_register_write(&context); assert(write_calls==0U && context.register_job_active);
    context.result_count=0U; tx_success=false; service_motor_register_write(&context);
    assert(context.result_count==1U && write_calls==1U);
    tick+=10U; tx_success=true; service_result_tx(&context);
    check_set_result(9U,0U); assert(write_calls==1U && context.result_count==0U);

    context=reset(); context.host_connected=context.lease_established=true;
    set_payload(&context,9U,golden,5U); assert(!can_arm(&context));
    uncertain_mask=1U; service_motor_register_write(&context); assert(!can_arm(&context));
    uncertain_mask=0U; assert(can_arm(&context));

    /* Fixed cache never evicts an unexpired entry to allow a duplicate write.
     * Past the documented retention window, sequence reuse is a NEW request. */
    context=reset();
    for (unsigned i=0U;i<YERTLE_WRITE_CACHE_CAPACITY;++i) {
        set_payload(&context,(uint16_t)i,golden,5U);
        service_motor_register_write(&context);
    }
    set_payload(&context,99U,golden,5U); check_set_result(99U,11U);
    assert(write_calls==YERTLE_WRITE_CACHE_CAPACITY);
    tick+=YERTLE_WRITE_CACHE_LIFETIME_MS;
    set_payload(&context,99U,golden,5U); assert(context.register_job_active);
    check_position_offsets();
    puts("register GET/SET and offsets: host dispatch, validation, replay, pacing and safety passed");
    return 0;
}
