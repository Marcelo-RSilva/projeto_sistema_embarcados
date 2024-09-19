// sensor_imu.c

#include "sensor_imu.h"
#include "driver/i2c.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

static uint8_t imu_address;

esp_err_t imu_init(uint8_t devAddr, gpio_num_t sda_pin, gpio_num_t scl_pin) {
    imu_address = devAddr;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    if (i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0) != ESP_OK) {
        return ESP_FAIL;
    }

    uint8_t who_am_i;
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, imu_address, &(uint8_t){0x75}, 1, &who_am_i, 1, pdMS_TO_TICKS(1000));
    if (err != ESP_OK || who_am_i != imu_address) {
        return ESP_ERR_NOT_FOUND;
    }

    // Sair do modo de sono
    return i2c_master_write_to_device(I2C_MASTER_NUM, imu_address, (uint8_t[]){0x6B, 0x00}, 2, pdMS_TO_TICKS(1000));
}

esp_err_t imu_get_acceleration_data(AccelerationData *data) {
    uint8_t raw_data[6];
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, imu_address, &(uint8_t){0x3B}, 1, raw_data, 6, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) return ESP_FAIL;

    data->x = (int16_t)((raw_data[0] << 8) | raw_data[1]) / 16384.0;
    data->y = (int16_t)((raw_data[2] << 8) | raw_data[3]) / 16384.0;
    data->z = (int16_t)((raw_data[4] << 8) | raw_data[5]) / 16384.0;

    return ESP_OK;
}

esp_err_t imu_get_gyroscope_data(GyroscopeData *data) {
    uint8_t raw_data[6];
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, imu_address, &(uint8_t){0x43}, 1, raw_data, 6, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) return ESP_FAIL;

    data->x = (int16_t)((raw_data[0] << 8) | raw_data[1]) / 131.0;
    data->y = (int16_t)((raw_data[2] << 8) | raw_data[3]) / 131.0;
    data->z = (int16_t)((raw_data[4] << 8) | raw_data[5]) / 131.0;

    return ESP_OK;
}

esp_err_t imu_deinit() {
    return i2c_driver_delete(I2C_MASTER_NUM);
}