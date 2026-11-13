#ifndef IMU_I2C_H
#define IMU_I2C_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Single owner: sensor_task. A failed transfer requires reinitialization. */
bool imu_i2c_init(void);
bool imu_i2c_write(uint8_t address, uint8_t reg, uint8_t value);
/* The IMU uses single-byte ID and multi-byte (at least three) sample reads. */
bool imu_i2c_read(uint8_t address, uint8_t reg, uint8_t *data, size_t length);

#endif
