// sensor_imu.h

#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

typedef struct {
    float x;
    float y;
    float z;
} AccelerationData;

typedef struct {
    float x;
    float y;
    float z;
} GyroscopeData;

esp_err_t imu_init(uint8_t devAddr, gpio_num_t sda_pin, gpio_num_t scl_pin);
esp_err_t imu_get_acceleration_data(AccelerationData *data);
esp_err_t imu_get_gyroscope_data(GyroscopeData *data);
esp_err_t imu_deinit();

#endif // SENSOR_IMU_H