#include "pmw3901_device.h"
#include "pmw3901_spi.h"
#include "pmw3901_spi_diagnostic.h"
#include "pmw3901_init_sequence.h"

#include "FreeRTOS.h"
#include "task.h"

#define PMW_SPI_TRANSFER_BUDGET_US 20000U
#define PMW_HEALTH_PERIOD_MS 1000U
#define PMW_MAX_INTERVAL_MS 100U

static uint32_t uptime_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void wait_ms(uint32_t duration_ms, bool at_startup)
{
    if (at_startup) {
        pmw3901_spi_delay_us(duration_ms * 1000U);
    } else {
        /* Tick phase may shorten vTaskDelay by almost one tick. */
        vTaskDelay(pdMS_TO_TICKS(duration_ms) + 1U);
    }
}

static bool fail(pmw3901_device_t *device, uint8_t error)
{
    pmw3901_feedback_t *feedback = &device->feedback;
    ++feedback->errors;
    feedback->last_error = error;
    feedback->valid = false;
    feedback->ready = false;
    feedback->interval_valid = false;
    feedback->state = PMW_RETRY;
    return false;
}

static bool read_spi(uint8_t state, uint8_t address, uint8_t *data, size_t length, bool burst)
{
    pmw3901_spi_context(state, 0U);
    pmw3901_spi_call_begin(PMW_SPI_TRANSFER_BUDGET_US);
    const bool transferred = pmw3901_spi_read(address, data, length, burst);
    const bool within_budget = pmw3901_spi_call_end();
    return transferred && within_budget;
}

static bool write_spi(uint8_t state, uint16_t index, uint8_t address, uint8_t value)
{
    pmw3901_spi_context(state, index);
    pmw3901_spi_call_begin(PMW_SPI_TRANSFER_BUDGET_US);
    const bool transferred = pmw3901_spi_write(address, value);
    const bool within_budget = pmw3901_spi_call_end();
    return transferred && within_budget;
}

static uint8_t check_id(uint8_t state, uint8_t address, uint8_t expected)
{
    uint8_t value = 0U;
    if (!read_spi(state, address, &value, 1U, false)) return PMW_ERROR_SPI;
    if (value == expected) return PMW_ERROR_NONE;
    pmw3901_spi_id_error(expected);
    return PMW_ERROR_ID;
}

bool pmw3901_device_init(pmw3901_device_t *device)
{
    const bool at_startup = xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED;
    pmw3901_feedback_t *feedback = &device->feedback;
    feedback->ready = feedback->valid = feedback->interval_valid = false;
    feedback->interval_ms = 0U;
    ++feedback->epoch;

    pmw3901_spi_init();
    pmw3901_spi_reset(true);
    wait_ms(2U, at_startup);
    pmw3901_spi_reset(false);
    pmw3901_spi_select(false);
    wait_ms(40U, at_startup);
    pmw3901_spi_select(true);
    wait_ms(2U, at_startup);
    pmw3901_spi_select(false);
    wait_ms(2U, at_startup);

    uint8_t error = check_id(PMW_ID, 0x00U, 0x49U);
    if (error == PMW_ERROR_NONE) error = check_id(PMW_ID, 0x5fU, 0xb6U);
    if (error != PMW_ERROR_NONE) return fail(device, error);

    if (!write_spi(PMW_RESET, 0U, 0x3aU, 0x5aU)) return fail(device, PMW_ERROR_SPI);
    wait_ms(5U, at_startup);
    for (uint8_t address = 0x02U; address <= 0x06U; ++address) {
        uint8_t ignored;
        if (!read_spi(PMW_CLEAR, address, &ignored, 1U, false)) return fail(device, PMW_ERROR_SPI);
    }
    wait_ms(1U, at_startup);
    for (uint16_t index = 0U; index < sizeof pmw3901_init_sequence / sizeof pmw3901_init_sequence[0]; ++index) {
        const uint8_t *entry = pmw3901_init_sequence[index];
        if (entry[0] == 0xffU) {
            wait_ms(entry[1], at_startup);
        } else if (!write_spi(PMW_CONFIG, index, entry[0], entry[1])) {
            return fail(device, PMW_ERROR_SPI);
        }
    }

    /* Prime the interval with a motion burst from this initialization epoch. */
    const uint32_t now = uptime_ms();
    if (!read_spi(PMW_DISCARD, 0x16U, feedback->raw, sizeof feedback->raw, true))
        return fail(device, PMW_ERROR_SPI);

    ++feedback->sequence;
    feedback->uptime_ms = now;
    feedback->last_error = PMW_ERROR_NONE;
    feedback->state = PMW_READY;
    feedback->ready = true;
    device->last_health_ms = now;
    device->needs_runtime_baseline = at_startup;
    return true;
}

bool pmw3901_device_service(pmw3901_device_t *device)
{
    pmw3901_feedback_t *feedback = &device->feedback;
    if (!feedback->ready) return false;

    const uint32_t now = uptime_ms();
    if (now - device->last_health_ms >= PMW_HEALTH_PERIOD_MS) {
        device->last_health_ms = now;
        const uint8_t error = check_id(PMW_READY, 0x00U, 0x49U);
        return error == PMW_ERROR_NONE || fail(device, error);
    }

    if (!read_spi(PMW_READY, 0x16U, feedback->raw, sizeof feedback->raw, true))
        return fail(device, PMW_ERROR_SPI);

    ++feedback->sequence;
    /* The startup burst predates the RTOS tick used for displacement timing. */
    feedback->interval_ms = device->needs_runtime_baseline ? 0U : now - feedback->uptime_ms;
    device->needs_runtime_baseline = false;
    feedback->uptime_ms = now;
    feedback->interval_valid = feedback->interval_ms > 0U && feedback->interval_ms <= PMW_MAX_INTERVAL_MS;
    feedback->valid = feedback->interval_valid;
    return true;
}
