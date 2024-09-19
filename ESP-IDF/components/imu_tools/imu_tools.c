// imu_tools.c

#include "imu_tools.h"
#include <math.h>

esp_err_t imu_read_data(IMUData *data) {
    if (imu_get_acceleration_data(&data->accel) != ESP_OK) return ESP_FAIL;
    if (imu_get_gyroscope_data(&data->gyro) != ESP_OK) return ESP_FAIL;
    return ESP_OK;
}

esp_err_t imu_calculate_quaternion(const IMUData *data, Quaternion *quaternion) {
    // Exemplo simplificado de cálculo de quaternion
    quaternion->w = 1.0;
    quaternion->x = data->accel.x;
    quaternion->y = data->accel.y;
    quaternion->z = data->accel.z;
    return ESP_OK;
}

esp_err_t imu_calculate_euler_angles(const Quaternion *quaternion, EulerAngle *euler) {
    // Exemplo simplificado de cálculo dos ângulos de Euler
    euler->roll = atan2f(2.0f * (quaternion->w * quaternion->x + quaternion->y * quaternion->z),
                         1.0f - 2.0f * (quaternion->x * quaternion->x + quaternion->y * quaternion->y));
    euler->pitch = asinf(2.0f * (quaternion->w * quaternion->y - quaternion->z * quaternion->x));
    euler->yaw = atan2f(2.0f * (quaternion->w * quaternion->z + quaternion->x * quaternion->y),
                        1.0f - 2.0f * (quaternion->y * quaternion->y + quaternion->z * quaternion->z));
    return ESP_OK;
}