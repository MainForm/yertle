#include "imu_i2c.h"

#include "board_clock.h"
#include "stm32f1xx.h"

#define SCL_PIN (1U << 6U)
#define SDA_PIN (1U << 7U)
#define I2C_PIN_CONFIG_MASK ((0xFU << 24U) | (0xFU << 28U))
#define GPIO_OPEN_DRAIN_2MHZ 0x6U
#define GPIO_AF_OPEN_DRAIN_2MHZ 0xEU

/* Below the 88–100 kHz repeated-START erratum range (ST ES096 §2.8.5). */
#define I2C_BUS_HZ 80000U
#define I2C_FLAG_TIMEOUT_US 1000U
#define I2C_TRANSFER_TIMEOUT_US 10000U
#define I2C_RECOVERY_EDGE_TIMEOUT_US 250U
#define I2C_RECOVERY_HALF_PERIOD_US 5U
#define I2C_OAR1_REQUIRED_BIT (1U << 14U)
#define I2C_ERROR_FLAGS (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR | I2C_SR1_TIMEOUT)

#ifdef IMU_I2C_TEST
/* Mock the hardware side effects of SR2/DR access and the clock in host tests. */
uint32_t imu_i2c_test_cycles(void);
uint32_t imu_i2c_test_read_sr2(void);
uint8_t imu_i2c_test_read_dr(void);
void imu_i2c_test_write_dr(uint8_t value);
#define i2c_cycles() imu_i2c_test_cycles()
#define read_sr2() imu_i2c_test_read_sr2()
#define read_dr() imu_i2c_test_read_dr()
#define write_dr(value) imu_i2c_test_write_dr(value)
#else
#define i2c_cycles() (DWT->CYCCNT)
#define read_sr2() (I2C1->SR2)
#define read_dr() ((uint8_t)I2C1->DR)
#define write_dr(value) (I2C1->DR = (value))
#endif

static uint32_t us_to_cycles(uint32_t us)
{
    return us * (SystemCoreClock / 1000000U);
}

static void recovery_delay(void)
{
    const uint32_t start = i2c_cycles();
    while (i2c_cycles() - start < us_to_cycles(I2C_RECOVERY_HALF_PERIOD_US)) {
    }
}

static void configure_pins(uint32_t mode)
{
    GPIOB->CRL = (GPIOB->CRL & ~I2C_PIN_CONFIG_MASK) | (mode << 24U) | (mode << 28U);
}

static bool wait_pin(uint32_t pin, bool high)
{
    const uint32_t start = i2c_cycles();
    while (((GPIOB->IDR & pin) != 0U) != high) {
        if (i2c_cycles() - start >= us_to_cycles(I2C_RECOVERY_EDGE_TIMEOUT_US)) return false;
    }
    return true;
}

static bool recover_lines(void)
{
    /* Only recovery drives the pins as GPIO. Clock out a stranded slave
     * byte, then apply the SDA/SCL transitions from ST ES096 §2.8.7. */
    if (!wait_pin(SCL_PIN, true)) return false;
    for (unsigned pulse = 0; pulse < 9U && !(GPIOB->IDR & SDA_PIN); ++pulse) {
        GPIOB->BRR = SCL_PIN;
        recovery_delay();
        GPIOB->BSRR = SCL_PIN;
        if (!wait_pin(SCL_PIN, true)) return false;
        recovery_delay();
    }
    if (!wait_pin(SDA_PIN, true)) return false;

    GPIOB->BRR = SDA_PIN;
    if (!wait_pin(SDA_PIN, false)) return false;
    recovery_delay();
    GPIOB->BRR = SCL_PIN;
    if (!wait_pin(SCL_PIN, false)) return false;
    recovery_delay();
    GPIOB->BSRR = SCL_PIN;
    if (!wait_pin(SCL_PIN, true)) return false;
    recovery_delay();
    GPIOB->BSRR = SDA_PIN;
    if (!wait_pin(SDA_PIN, true)) return false;
    recovery_delay();
    return true;
}

static void reset_peripheral(void)
{
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0U;
}

bool imu_i2c_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    (void)RCC->APB1ENR;

    I2C1->CR1 = 0U;
    GPIOB->BSRR = SCL_PIN | SDA_PIN;
    configure_pins(GPIO_OPEN_DRAIN_2MHZ);
    const bool released = recover_lines();
    GPIOB->BSRR = SCL_PIN | SDA_PIN;

    /* The board uses the reset I2C1 pin map, PB6=SCL and PB7=SDA. */
    configure_pins(GPIO_AF_OPEN_DRAIN_2MHZ);
    reset_peripheral();
    if (!released) return false;

    const uint32_t apb1_hz = board_apb1_clock_hz();
    const uint32_t apb1_mhz = apb1_hz / 1000000U;
    I2C1->CR2 = apb1_mhz;
    I2C1->OAR1 = I2C_OAR1_REQUIRED_BIT;
    I2C1->CCR = apb1_hz / (2U * I2C_BUS_HZ);
    I2C1->TRISE = apb1_mhz + 1U;
    I2C1->CR1 = I2C_CR1_PE;
    return true;
}

static bool wait_status(const volatile uint32_t *reg, uint32_t mask, bool set,
                        uint32_t transfer_started)
{
    const uint32_t stage_started = i2c_cycles();
    for (;;) {
        const uint32_t now = i2c_cycles();
        if (now - transfer_started >= us_to_cycles(I2C_TRANSFER_TIMEOUT_US)) return false;

        /* Read the current flags before declaring a stage timeout. The
         * peripheral can finish while a higher-priority task runs. */
        const uint32_t status = I2C1->SR1;
        if (status & I2C_ERROR_FLAGS) return false;
        const uint32_t value = reg == &I2C1->SR1 ? status :
                               reg == &I2C1->SR2 ? read_sr2() : *reg;
        if (((value & mask) != 0U) == set) return true;
        if (now - stage_started >= us_to_cycles(I2C_FLAG_TIMEOUT_US)) return false;
    }
}

static bool wait_event(uint32_t flag, uint32_t started)
{
    return wait_status(&I2C1->SR1, flag, true, started);
}

static void clear_address(void)
{
    (void)I2C1->SR1;
    (void)read_sr2();
}

static bool finish_transfer(uint32_t started)
{
    return wait_status(&I2C1->CR1, I2C_CR1_STOP, false, started);
}

static bool abort_transfer(void)
{
    /* Release an owned transfer if possible. A stuck START or bus error
     * still gets SWRST; the next IMU probe also recovers the physical lines. */
    if ((read_sr2() & I2C_SR2_MSL) && !(I2C1->CR1 & I2C_CR1_START)) {
        I2C1->SR1 = 0U;
        I2C1->CR1 |= I2C_CR1_STOP;
        (void)finish_transfer(i2c_cycles());
    }
    reset_peripheral();
    return false;
}

static bool address_device(uint8_t address_byte, uint32_t started)
{
    I2C1->CR1 |= I2C_CR1_START;
    if (!wait_event(I2C_SR1_SB, started)) return false;
    write_dr(address_byte);
    return wait_event(I2C_SR1_ADDR, started);
}

static bool select_register(uint8_t address, uint8_t reg, uint32_t started)
{
    if (!(I2C1->CR1 & I2C_CR1_PE) ||
        !wait_status(&I2C1->SR2, I2C_SR2_BUSY, false, started) ||
        !address_device((uint8_t)(address << 1U), started)) return false;
    clear_address();
    write_dr(reg);
    return wait_event(I2C_SR1_BTF, started);
}

bool imu_i2c_write(uint8_t address, uint8_t reg, uint8_t value)
{
    const uint32_t started = i2c_cycles();
    if (!select_register(address, reg, started)) return abort_transfer();
    write_dr(value);
    if (!wait_event(I2C_SR1_BTF, started)) return abort_transfer();

    I2C1->CR1 |= I2C_CR1_STOP;
    return finish_transfer(started) || abort_transfer();
}

static bool receive_one(uint8_t *data, uint32_t started)
{
    /* AN2824 EV6_3: issue STOP before the single byte can be ACKed. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    I2C1->CR1 &= ~I2C_CR1_ACK;
    clear_address();
    I2C1->CR1 |= I2C_CR1_STOP;
    __set_PRIMASK(primask);

    if (!wait_event(I2C_SR1_RXNE, started)) return false;
    *data = read_dr();
    return true;
}

static bool receive_multiple(uint8_t *data, size_t length, uint32_t started)
{
    clear_address();
    while (length > 3U) {
        /* BTF holds SCL low while both receive registers are full. This
         * tolerates preemption between the polling iterations. */
        if (!wait_event(I2C_SR1_BTF, started)) return false;
        *data++ = read_dr();
        --length;
    }
    if (!wait_event(I2C_SR1_BTF, started)) return false;

    /* ST ES096 §2.8.2: mask IRQs across the final BTF/STOP sequence. */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    I2C1->CR1 &= ~I2C_CR1_ACK;
    *data++ = read_dr();
    I2C1->CR1 |= I2C_CR1_STOP;
    *data++ = read_dr();
    __set_PRIMASK(primask);

    if (!wait_event(I2C_SR1_RXNE, started)) return false;
    *data = read_dr();
    return true;
}

bool imu_i2c_read(uint8_t address, uint8_t reg, uint8_t *data, size_t length)
{
    if (!data || (length != 1U && length < 3U)) return false;

    const uint32_t started = i2c_cycles();
    if (!select_register(address, reg, started)) return abort_transfer();
    if (length > 1U) I2C1->CR1 |= I2C_CR1_ACK;
    if (!address_device((uint8_t)((address << 1U) | 1U), started)) return abort_transfer();

    const bool received = length == 1U ? receive_one(data, started) :
                                        receive_multiple(data, length, started);
    if (!received || !finish_transfer(started)) return abort_transfer();
    return true;
}
