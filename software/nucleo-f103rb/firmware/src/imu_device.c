#include "imu_device.h"

#include <stddef.h>

#include "stm32f1xx.h"

#define SCL_PIN (1U << 6U)
#define SDA_PIN (1U << 7U)
#define MPU_ADDRESS 0x68U
#define REG_SMPLRT_DIV 0x19U
#define REG_CONFIG 0x1AU
#define REG_GYRO_CONFIG 0x1BU
#define REG_ACCEL_CONFIG 0x1CU
#define REG_ACCEL_XOUT_H 0x3BU
#define REG_PWR_MGMT_1 0x6BU
#define REG_WHO_AM_I 0x75U
#define I2C_HALF_PERIOD_US 5U
#define I2C_STRETCH_TIMEOUT_US 250U

static bool initialized;

static uint32_t cycles_per_us(void)
{
    return SystemCoreClock / 1000000U;
}

static void delay_us(uint32_t microseconds)
{
    const uint32_t start = DWT->CYCCNT;
    const uint32_t duration = microseconds * cycles_per_us();
    while ((uint32_t)(DWT->CYCCNT - start) < duration) {
    }
}

static void scl_low(void) { GPIOB->BRR = SCL_PIN; }
static void scl_release(void) { GPIOB->BSRR = SCL_PIN; }
static void sda_low(void) { GPIOB->BRR = SDA_PIN; }
static void sda_release(void) { GPIOB->BSRR = SDA_PIN; }
static bool sda_high(void) { return (GPIOB->IDR & SDA_PIN) != 0U; }

static bool wait_scl_high(void)
{
    const uint32_t start = DWT->CYCCNT;
    scl_release();
    while ((GPIOB->IDR & SCL_PIN) == 0U) {
        if ((uint32_t)(DWT->CYCCNT - start) >=
            I2C_STRETCH_TIMEOUT_US * cycles_per_us()) {
            return false;
        }
    }
    return true;
}

static bool i2c_start(void)
{
    sda_release();
    if (!wait_scl_high()) return false;
    delay_us(I2C_HALF_PERIOD_US);
    if (!sda_high()) return false;
    sda_low();
    delay_us(I2C_HALF_PERIOD_US);
    scl_low();
    return true;
}

static void i2c_stop(void)
{
    sda_low();
    delay_us(I2C_HALF_PERIOD_US);
    (void)wait_scl_high();
    delay_us(I2C_HALF_PERIOD_US);
    sda_release();
    delay_us(I2C_HALF_PERIOD_US);
}

static bool i2c_write_byte(uint8_t value)
{
    for (unsigned bit = 0U; bit < 8U; ++bit) {
        if ((value & 0x80U) != 0U) sda_release();
        else sda_low();
        delay_us(I2C_HALF_PERIOD_US);
        if (!wait_scl_high()) return false;
        delay_us(I2C_HALF_PERIOD_US);
        scl_low();
        value <<= 1U;
    }
    sda_release();
    delay_us(I2C_HALF_PERIOD_US);
    if (!wait_scl_high()) return false;
    const bool acknowledged = !sda_high();
    delay_us(I2C_HALF_PERIOD_US);
    scl_low();
    return acknowledged;
}

static bool i2c_read_byte(uint8_t *value, bool acknowledge)
{
    uint8_t result = 0U;
    sda_release();
    for (unsigned bit = 0U; bit < 8U; ++bit) {
        delay_us(I2C_HALF_PERIOD_US);
        if (!wait_scl_high()) return false;
        result = (uint8_t)((result << 1U) | (sda_high() ? 1U : 0U));
        delay_us(I2C_HALF_PERIOD_US);
        scl_low();
    }
    if (acknowledge) sda_low();
    else sda_release();
    delay_us(I2C_HALF_PERIOD_US);
    if (!wait_scl_high()) return false;
    delay_us(I2C_HALF_PERIOD_US);
    scl_low();
    sda_release();
    *value = result;
    return true;
}

static bool write_register(uint8_t reg, uint8_t value)
{
    const bool ok = i2c_start() && i2c_write_byte(MPU_ADDRESS << 1U) &&
                    i2c_write_byte(reg) && i2c_write_byte(value);
    i2c_stop();
    return ok;
}

static bool read_registers(uint8_t reg, uint8_t *data, size_t length)
{
    bool ok = data != NULL && length > 0U && i2c_start() &&
              i2c_write_byte(MPU_ADDRESS << 1U) && i2c_write_byte(reg);
    if (ok) {
        /* Repeated START: SDA rises while SCL is low, then falls while high. */
        ok = i2c_start() && i2c_write_byte((MPU_ADDRESS << 1U) | 1U);
    }
    for (size_t index = 0U; ok && index < length; ++index) {
        ok = i2c_read_byte(&data[index], index + 1U < length);
    }
    i2c_stop();
    return ok;
}

static int16_t unpack_be16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

bool imu_device_init(void)
{
    initialized = false;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    GPIOB->BSRR = SCL_PIN | SDA_PIN;
    /* PB6/PB7: 2 MHz general-purpose open-drain output; external pull-ups. */
    GPIOB->CRL = (GPIOB->CRL & ~((0xFU << 24U) | (0xFU << 28U))) |
                 (0x6U << 24U) | (0x6U << 28U);

    /* Release an SDA line left low by an interrupted read. */
    sda_release();
    for (unsigned pulse = 0U; pulse < 9U && !sda_high(); ++pulse) {
        scl_low();
        delay_us(I2C_HALF_PERIOD_US);
        if (!wait_scl_high()) return false;
        delay_us(I2C_HALF_PERIOD_US);
    }
    i2c_stop();

    uint8_t who_am_i = 0U;
    if (!read_registers(REG_WHO_AM_I, &who_am_i, 1U) || who_am_i != MPU_ADDRESS) {
        return false;
    }
    initialized = write_register(REG_PWR_MGMT_1, 0x01U) &&
                  write_register(REG_CONFIG, 0x03U) &&
                  write_register(REG_SMPLRT_DIV, 0x09U) &&
                  write_register(REG_GYRO_CONFIG, 0x00U) &&
                  write_register(REG_ACCEL_CONFIG, 0x00U);
    return initialized;
}

bool imu_device_read(imu_raw_sample_t *sample)
{
    uint8_t bytes[14];
    if (sample == NULL || !initialized ||
        !read_registers(REG_ACCEL_XOUT_H, bytes, sizeof(bytes))) {
        return false;
    }
    sample->accel_x = unpack_be16(&bytes[0]);
    sample->accel_y = unpack_be16(&bytes[2]);
    sample->accel_z = unpack_be16(&bytes[4]);
    sample->temperature = unpack_be16(&bytes[6]);
    sample->gyro_x = unpack_be16(&bytes[8]);
    sample->gyro_y = unpack_be16(&bytes[10]);
    sample->gyro_z = unpack_be16(&bytes[12]);
    return true;
}
