#ifndef IMU_DEVICE_H
#define IMU_DEVICE_H

#include <stdbool.h>
#include <stdint.h>

/* MPU-6050 raw values at +/-2 g and +/-250 deg/s. */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temperature;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} imu_raw_sample_t;

/* I2C on PB6=SCL, PB7=SDA. Both lines need 3.3 V pull-ups. */
bool imu_device_init(void);
bool imu_device_read(imu_raw_sample_t *sample);

#endif /* IMU_DEVICE_H */
