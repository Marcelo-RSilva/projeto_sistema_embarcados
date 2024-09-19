#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "imu_tools.h"

/**
 * @brief Função principal do aplicativo
 * 
 * Esta função inicializa o IMU, lê continuamente os dados do sensor e os imprime no console.
 */
void app_main() {
    uint8_t addr = 0x68;  // Endereço padrão do IMU no barramento I2C

    // Inicializa o IMU com o endereço especificado e os pinos SDA/SCL
    if (imu_init(addr, SDA_PIN, SCL_PIN) != ESP_OK) {
        printf("Failed to initialize IMU.\n");
        return;  // Encerra o programa se a inicialização falhar
    }

    // Declaração das variáveis para armazenar os dados do IMU, quaternion e ângulos de Euler
    IMUData imu_data;
    Quaternion quaternion;
    EulerAngle euler;

    while (1) {
        // Lê os dados de aceleração e giroscópio do IMU
        if (imu_read_data(&imu_data) == ESP_OK) {
            // Calcula o quaternion com base nos dados lidos
            imu_calculate_quaternion(&imu_data, &quaternion);

            // Calcula os ângulos de Euler a partir do quaternion
            imu_calculate_euler_angles(&quaternion, &euler);

            // Imprime os valores de aceleração, rotação e ângulos de Euler
            printf("\nAcelerômetro [X, Y, Z]: %f, %f, %f\nGiroscopio [X, Y, Z]: %f, %f, %f\n",
                   imu_data.accel.x, imu_data.accel.y, imu_data.accel.z,
                   imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z);

            printf("\nQuaternion [W, X, Y, Z]: %f, %f, %f, %f\n",
                   quaternion.w, quaternion.x, quaternion.y, quaternion.z);

            printf("\nAngulos de Euler [Roll, Pitch, Yaw]: %f, %f, %f\n",
                   euler.roll, euler.pitch, euler.yaw);
        } else {
            printf("Falha ao ler os dados IMU.\n");
        }
      // Delay de 1 segundo
      vTaskDelay(pdMS_TO_TICKS(3000));
    }

    imu_deinit();  // Desinstala o driver I2C e desinicializa o IMU
}
