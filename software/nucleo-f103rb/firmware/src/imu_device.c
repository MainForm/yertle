#include "imu_device.h"
#include "imu_i2c.h"

#include <stddef.h>

#define MPU_ADDRESS 0x68U
#define REG_SMPLRT_DIV 0x19U
#define REG_CONFIG 0x1AU
#define REG_GYRO_CONFIG 0x1BU
#define REG_ACCEL_CONFIG 0x1CU
#define REG_ACCEL_XOUT_H 0x3BU
#define REG_PWR_MGMT_1 0x6BU
#define REG_WHO_AM_I 0x75U

static bool initialized;

static int16_t unpack_be16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0] << 8U) | data[1]);
}

bool imu_device_init(void)
{
    initialized = false;
    if (!imu_i2c_init()) return false;

    uint8_t who_am_i = 0U;
    if (!imu_i2c_read(MPU_ADDRESS, REG_WHO_AM_I, &who_am_i, 1U) || who_am_i != MPU_ADDRESS) {
        return false;
    }
    initialized = imu_i2c_write(MPU_ADDRESS, REG_PWR_MGMT_1, 0x01U) &&
                  imu_i2c_write(MPU_ADDRESS, REG_CONFIG, 0x03U) &&
                  imu_i2c_write(MPU_ADDRESS, REG_SMPLRT_DIV, 0x09U) &&
                  imu_i2c_write(MPU_ADDRESS, REG_GYRO_CONFIG, 0x00U) &&
                  imu_i2c_write(MPU_ADDRESS, REG_ACCEL_CONFIG, 0x00U);
    return initialized;
}

bool imu_device_is_ready(void)
{
    return initialized;
}

bool imu_device_read(imu_raw_sample_t *sample)
{
    uint8_t bytes[14];
    if (sample == NULL || !initialized) return false;
    if (!imu_i2c_read(MPU_ADDRESS, REG_ACCEL_XOUT_H, bytes, sizeof(bytes))) {
        initialized = false;
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
