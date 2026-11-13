#include "imu_device.h"
#include "stm32f1xx.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

I2C_Type i2c;
GPIO_Type gpiob;
RCC_Type rcc;
Debug_Type debug;
DWT_Type dwt;
uint32_t SystemCoreClock = 64000000U;

#define SCL (1U << 6)
#define SDA (1U << 7)

static uint32_t cycles, primask, gpio_output = SCL | SDA;
static uint8_t registers[128], selected_register;
static unsigned starts, stops, received, writes, recovery_pulses;
static bool read_address, expect_register, recovering, hold_sda, hold_scl;
static bool suppress_start, nack_address, suppress_receive;
static uint32_t poll_step_us = 1U;

uint32_t board_apb1_clock_hz(void) { return 32000000U; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1U; }
void __set_PRIMASK(uint32_t value) { primask = value; }

uint32_t imu_i2c_test_cycles(void)
{
    cycles += poll_step_us * 64U;
    const uint32_t previous_output = gpio_output;
    gpio_output = (gpio_output & ~gpiob.BRR) | gpiob.BSRR;
    if ((gpio_output & SCL) && !(previous_output & SCL)) {
        ++recovery_pulses;
        if (recovery_pulses >= 3U) hold_sda = false;
    }
    gpiob.IDR = gpio_output;
    if (hold_sda) gpiob.IDR &= ~SDA;
    if (hold_scl) gpiob.IDR &= ~SCL;
    gpiob.BSRR = gpiob.BRR = 0U;
    if (recovering && (i2c.CR1 & I2C_CR1_PE)) {
        i2c.SR1 = i2c.SR2 = 0U;
        recovering = false;
    }
    if ((i2c.CR1 & I2C_CR1_START) && !suppress_start && !(i2c.SR1 & I2C_SR1_SB)) {
        i2c.SR1 = I2C_SR1_SB;
        i2c.SR2 = I2C_SR2_BUSY | I2C_SR2_MSL;
        ++starts;
    }
    if ((i2c.CR1 & I2C_CR1_STOP) && !primask) {
        i2c.CR1 &= ~I2C_CR1_STOP;
        i2c.SR2 = 0U;
        ++stops;
    }
    return cycles;
}

uint32_t imu_i2c_test_read_sr2(void)
{
    if (i2c.SR1 & I2C_SR1_ADDR) {
        if (read_address && selected_register == 0x75U)
            assert(primask && !(i2c.CR1 & I2C_CR1_ACK) && !(i2c.CR1 & I2C_CR1_STOP));
        if (read_address && selected_register == 0x3BU) assert(i2c.CR1 & I2C_CR1_ACK);
        i2c.SR1 = read_address && !suppress_receive ?
                  (I2C_SR1_RXNE | I2C_SR1_BTF) : 0U;
    }
    return i2c.SR2;
}

void imu_i2c_test_write_dr(uint8_t value)
{
    if (i2c.SR1 & I2C_SR1_SB) {
        assert(value == 0xD0U || value == 0xD1U);
        i2c.CR1 &= ~I2C_CR1_START;
        read_address = value == 0xD1U;
        expect_register = !read_address;
        received = 0U;
        i2c.SR1 = nack_address ? I2C_SR1_AF : I2C_SR1_ADDR;
    } else {
        if (expect_register) selected_register = value;
        else {
            registers[selected_register] = value;
            ++writes;
        }
        expect_register = false;
        i2c.SR1 = I2C_SR1_BTF;
    }
}

uint8_t imu_i2c_test_read_dr(void)
{
    assert(read_address);
    if (selected_register == 0x3BU) {
        if (received < 11U) assert(i2c.CR1 & I2C_CR1_ACK);
        if (received == 11U) assert(primask && !(i2c.CR1 & (I2C_CR1_ACK | I2C_CR1_STOP)));
        if (received == 12U) assert(primask && (i2c.CR1 & I2C_CR1_STOP));
        if (received == 13U) assert(!primask && !(i2c.CR1 & I2C_CR1_ACK));
    }
    return registers[selected_register + received++];
}

static bool initialize(void)
{
    recovering = true;
    return imu_device_init();
}

int main(void)
{
    registers[0x75] = 0x68U;
    gpiob.CRL = 0xABCDEFU;
    gpiob.IDR = SCL;
    hold_sda = true;
    assert(initialize());
    assert(recovery_pulses >= 3U);
    assert(gpiob.CRL == 0xEEABCDEFU && (rcc.APB1ENR & RCC_APB1ENR_I2C1EN));
    assert(i2c.CR2 == 32U && i2c.CCR == 200U && i2c.TRISE == 33U);
    assert(starts == 7U && stops == 6U && writes == 5U && !primask);
    assert(registers[0x6B] == 1U && registers[0x1A] == 3U && registers[0x19] == 9U);
    assert(registers[0x1B] == 0U && registers[0x1C] == 0U);

    const uint8_t sample_bytes[] = {0x40,0, 0xFF,0xFE, 0,3, 0x80,0, 0,5, 0xFF,0xFA, 0,7};
    memcpy(registers + 0x3B, sample_bytes, sizeof sample_bytes);
    imu_raw_sample_t sample = {0};
    assert(imu_device_read(&sample));
    assert(starts == 9U && stops == 7U && received == 14U && !primask);
    assert(sample.accel_x == 16384 && sample.accel_y == -2 && sample.accel_z == 3);
    assert(sample.temperature == -32768 && sample.gyro_x == 5 && sample.gyro_y == -6 && sample.gyro_z == 7);

    uint32_t begin = cycles;
    suppress_receive = true;
    assert(!imu_device_read(&sample));
    assert(sample.gyro_z == 7 && i2c.CR1 == 0U && !primask);
    assert(cycles - begin >= 1000U * 64U && cycles - begin < 2000U * 64U);
    suppress_receive = false;
    assert(initialize() && imu_device_read(&sample));

    /* Delays spread across successful flags must still hit the transaction cap. */
    poll_step_us = 800U;
    assert(!imu_device_read(&sample) && i2c.CR1 == 0U && !primask);
    poll_step_us = 1U;
    assert(initialize());

    nack_address = true;
    assert(!imu_device_read(&sample) && i2c.CR1 == 0U);
    nack_address = false;
    assert(initialize());
    suppress_start = true;
    assert(!imu_device_read(&sample) && i2c.CR1 == 0U);
    suppress_start = false;
    assert(initialize());

    const unsigned starts_before_busy = starts;
    i2c.SR2 = I2C_SR2_BUSY;
    assert(!imu_device_read(&sample) && i2c.CR1 == 0U && starts == starts_before_busy);
    assert(initialize() && imu_device_read(&sample));

    hold_scl = true;
    begin = cycles;
    assert(!initialize() && i2c.CR1 == 0U);
    assert(cycles - begin >= 250U * 64U && cycles - begin < 300U * 64U);
    hold_scl = false;
    assert(initialize());
    puts("IMU I2C1 setup, receive sequences, register data, deadlines and recovery passed");
    return 0;
}
