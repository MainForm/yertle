#ifdef NDEBUG
#undef NDEBUG
#endif

#include "pmw3901_device.h"
#include "pmw3901_init_sequence.h"
#include "pmw3901_spi.h"
#include "pmw3901_spi_diagnostic.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick;
static uint8_t state;
static uint16_t config_index;
static unsigned spi_inits, writes, reads, bursts, id_errors, config_writes;
static bool reset_asserted, chip_selected, active_transfer;
static bool bad_id, fail_spi, fail_budget;
static uint32_t reset_at, release_at, cs_low_at, cs_high_at, power_reset_at, last_clear_at;
static unsigned config_delay_calls;

uint32_t xTaskGetTickCount(void) { return tick; }

void vTaskDelay(uint32_t ticks)
{
    assert(!active_transfer && ticks > 0U);
    if (state == PMW_CONFIG && ticks == 11U) ++config_delay_calls;
    tick += ticks;
}

void pmw3901_spi_context(uint8_t stage, uint16_t index)
{
    state = stage;
    config_index = index;
}

void pmw3901_spi_call_begin(uint32_t budget)
{
    assert(!active_transfer && budget == 20000U);
    active_transfer = true;
}

bool pmw3901_spi_call_end(void)
{
    assert(active_transfer);
    active_transfer = false;
    return !fail_budget;
}

void pmw3901_spi_id_error(uint8_t expected)
{
    assert(expected == 0x49U);
    ++id_errors;
}

void pmw3901_spi_init(void)
{
    ++spi_inits;
    config_writes = 0U;
    config_delay_calls = 0U;
}

void pmw3901_spi_reset(bool asserted)
{
    reset_asserted = asserted;
    if (asserted) reset_at = tick;
    else {
        assert(tick - reset_at >= 2U);
        release_at = tick;
    }
}

void pmw3901_spi_select(bool selected)
{
    assert(!reset_asserted);
    if (selected) {
        assert(tick - release_at >= 40U);
        cs_low_at = tick;
    } else if (chip_selected) {
        assert(tick - cs_low_at >= 2U);
        cs_high_at = tick;
    }
    chip_selected = selected;
}

bool pmw3901_spi_write(uint8_t address, uint8_t value)
{
    assert(active_transfer && !chip_selected);
    ++writes;
    if (state == PMW_RESET) {
        assert(address == 0x3aU && value == 0x5aU);
        power_reset_at = tick;
    } else {
        assert(state == PMW_CONFIG);
        assert(tick - last_clear_at >= 1U);
        assert(address == pmw3901_init_sequence[config_index][0]);
        assert(value == pmw3901_init_sequence[config_index][1]);
        ++config_writes;
    }
    return !fail_spi;
}

bool pmw3901_spi_read(uint8_t address, uint8_t *data, size_t size, bool burst)
{
    assert(active_transfer && !chip_selected);
    ++reads;
    if (fail_spi) return false;
    memset(data, 0, size);
    if (burst) {
        assert(address == 0x16U && size == 12U && (state == PMW_DISCARD || state == PMW_READY));
        ++bursts;
        data[0] = 0x80U;
    } else {
        assert(size == 1U);
        if (address == 0x00U || address == 0x5fU) {
            assert(state == PMW_ID || state == PMW_READY);
            if (state == PMW_ID) assert(tick - cs_high_at >= 2U);
            *data = address == 0x5fU ? 0xb6U : bad_id ? 0U : 0x49U;
        } else {
            assert(state == PMW_CLEAR && address >= 0x02U && address <= 0x06U);
            assert(tick - power_reset_at >= 5U);
            last_clear_at = tick;
        }
    }
    return true;
}

int main(void)
{
    pmw3901_device_t device = {0};
    assert(!pmw3901_device_service(&device) && reads == 0U);
    assert(pmw3901_device_init(&device));
    assert(device.feedback.ready && !device.feedback.valid && device.feedback.state == PMW_READY);
    assert(device.feedback.epoch == 1U && device.feedback.sequence == 1U);
    assert(reads == 8U && bursts == 1U && spi_inits == 1U);
    assert(config_writes + 1U == sizeof pmw3901_init_sequence / sizeof pmw3901_init_sequence[0]);
    assert(config_delay_calls == 1U && writes == config_writes + 1U);

    const unsigned init_writes = writes;
    tick += 20U;
    assert(pmw3901_device_service(&device));
    assert(device.feedback.valid && device.feedback.interval_ms == 20U && device.feedback.sequence == 2U);
    assert(writes == init_writes && bursts == 2U && spi_inits == 1U);

    tick += 1000U;
    assert(pmw3901_device_service(&device) && bursts == 2U && state == PMW_READY);
    tick += 20U;
    fail_budget = true;
    assert(!pmw3901_device_service(&device));
    assert(device.feedback.errors == 1U && !device.feedback.valid && device.feedback.state == PMW_RETRY);
    assert(device.feedback.sequence == 2U && !active_transfer);
    fail_budget = false;

    bad_id = true;
    assert(!pmw3901_device_init(&device) && id_errors == 1U);
    assert(device.feedback.last_error == PMW_ERROR_ID && device.feedback.errors == 2U);
    assert(!device.feedback.ready && device.feedback.epoch == 2U && writes == init_writes);
    assert(!pmw3901_device_service(&device) && spi_inits == 2U);

    bad_id = false;
    assert(pmw3901_device_init(&device));
    assert(device.feedback.epoch == 3U && device.feedback.sequence == 3U);
    assert(!device.feedback.valid && device.feedback.interval_ms == 0U && device.feedback.errors == 2U);
    tick += 20U;
    assert(pmw3901_device_service(&device) && device.feedback.valid);

    fail_spi = true;
    assert(!pmw3901_device_init(&device));
    assert(device.feedback.last_error == PMW_ERROR_SPI && device.feedback.errors == 3U);
    assert(!device.feedback.ready && !active_transfer);
    puts("PASS: full init before sampling, timing, health check, diagnostics and reinitialization");
    return 0;
}
