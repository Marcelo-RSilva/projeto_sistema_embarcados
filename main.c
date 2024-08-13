#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "driver/gpio.h"

// Definições para configuração do I2C
#define I2C_MASTER_NUM I2C_NUM_0             // Número do I2C utilizado
#define I2C_MASTER_FREQ_HZ 100000            // Frequência do I2C (100kHz)
#define SDA_PIN 21                           // Pino para o SDA (dados) do I2C
#define SCL_PIN 22                           // Pino para o SCL (clock) do I2C

// Estruturas para armazenar os dados do IMU
typedef struct {
    float x;
    float y;
    float z;
} AccelerationData;                           // Estrutura para os dados de aceleração

typedef struct {
    float x;
    float y;
    float z;
} GyroscopeData;                              // Estrutura para os dados do giroscópio

typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;                                 // Estrutura para representar um quaternion

typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngle;                                 // Estrutura para representar os ângulos de Euler

typedef struct {
    AccelerationData accel;
    GyroscopeData gyro;
} IMUData;                                    // Estrutura para armazenar os dados completos do IMU

// Variável para armazenar o endereço do IMU
static uint8_t imu_address;

/*
  @brief Inicializa o sensor IMU
  
  Esta função configura a comunicação I2C e inicializa o sensor IMU. Ela também verifica
 se o sensor está respondendo corretamente através do registro "Who am I".
 
  @param devAddr Endereço do dispositivo IMU no barramento I2C
  @param sda_pin Pino utilizado para o SDA
  @param scl_pin Pino utilizado para o SCL
  @return esp_err_t Retorna ESP_OK se a inicialização for bem-sucedida, caso contrário, retorna um erro.
 */
esp_err_t imu_init(uint8_t devAddr, gpio_num_t sda_pin, gpio_num_t scl_pin) {
    imu_address = devAddr;  // Armazena o endereço do IMU

    // Configuração do I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);

    // Instala o driver I2C e verifica se houve sucesso
    if (i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0) != ESP_OK) {
        return ESP_FAIL;  // Retorna falha se o driver não for instalado corretamente
    }

    // Verifica o registro "Who am I" do IMU para garantir que o dispositivo está respondendo corretamente
    uint8_t who_am_i;
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, imu_address, &(uint8_t){0x75}, 1, &who_am_i, 1, pdMS_TO_TICKS(1000));
    if (err != ESP_OK || who_am_i != imu_address) {
        return ESP_ERR_NOT_FOUND;  // Retorna erro se o dispositivo não responder corretamente
    }

    // Sair do modo de sono, configurando o registro de gerenciamento de energia (0x6B)
    return i2c_master_write_to_device(I2C_MASTER_NUM, imu_address, (uint8_t[]){0x6B, 0x00}, 2, pdMS_TO_TICKS(1000));
}

/*
  @brief Lê os dados de aceleração do IMU
  
  Esta função lê os dados brutos do acelerômetro do IMU e os converte para unidades de gravidade (g).
 
  @param data Ponteiro para a estrutura onde os dados de aceleração serão armazenados
  @return esp_err_t Retorna ESP_OK se a leitura for bem-sucedida, caso contrário, retorna um erro.
 */
esp_err_t imu_get_acceleration_data(AccelerationData *data) {
    uint8_t raw_data[6];  // Buffer para armazenar os dados brutos

    // Lê 6 bytes de dados do acelerômetro (2 bytes para cada eixo)
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, imu_address, &(uint8_t){0x3B}, 1, raw_data, 6, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) return ESP_FAIL;

    // Converte os dados brutos para valores de aceleração em g (considerando a faixa de ±2g)
    data->x = (int16_t)((raw_data[0] << 8) | raw_data[1]) / 16384.0;
    data->y = (int16_t)((raw_data[2] << 8) | raw_data[3]) / 16384.0;
    data->z = (int16_t)((raw_data[4] << 8) | raw_data[5]) / 16384.0;

    return ESP_OK;  // Retorna sucesso na leitura
}

/*
  @brief Lê os dados do giroscópio do IMU
  
  Esta função lê os dados brutos do giroscópio do IMU e os converte para valores de rotação em graus por segundo (°/s).
 
  @param data Ponteiro para a estrutura onde os dados do giroscópio serão armazenados
  @return esp_err_t Retorna ESP_OK se a leitura for bem-sucedida, caso contrário, retorna um erro.
 */
esp_err_t imu_get_gyroscope_data(GyroscopeData *data) {
    uint8_t raw_data[6];  // Buffer para armazenar os dados brutos

    // Lê 6 bytes de dados do giroscópio (2 bytes para cada eixo)
    esp_err_t err = i2c_master_write_read_device(I2C_MASTER_NUM, imu_address, &(uint8_t){0x43}, 1, raw_data, 6, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) return ESP_FAIL;

    // Converte os dados brutos para valores de rotação em °/s (considerando a faixa de ±250°/s)
    data->x = (int16_t)((raw_data[0] << 8) | raw_data[1]) / 131.0;
    data->y = (int16_t)((raw_data[2] << 8) | raw_data[3]) / 131.0;
    data->z = (int16_t)((raw_data[4] << 8) | raw_data[5]) / 131.0;

    return ESP_OK;  // Retorna sucesso na leitura
}

/*
  @brief Desinstala o driver I2C e desinicializa o IMU
  
  Esta função desinstala o driver I2C utilizado para se comunicar com o IMU.
 
  @return esp_err_t Retorna ESP_OK se a desinstalação for bem-sucedida, caso contrário, retorna um erro.
 */
esp_err_t imu_deinit() {
    return i2c_driver_delete(I2C_MASTER_NUM);  // Desinstala o driver I2C
}

/*
  @brief Lê todos os dados (aceleração e giroscópio) do IMU
  
  Esta função combina as leituras de aceleração e giroscópio e armazena os resultados na estrutura IMUData.
 
  @param data Ponteiro para a estrutura onde os dados completos do IMU serão armazenados
  @return esp_err_t Retorna ESP_OK se a leitura for bem-sucedida, caso contrário, retorna um erro.
 */
esp_err_t imu_read_data(IMUData *data) {
    if (imu_get_acceleration_data(&data->accel) != ESP_OK) return ESP_FAIL;  // Lê os dados de aceleração
    if (imu_get_gyroscope_data(&data->gyro) != ESP_OK) return ESP_FAIL;      // Lê os dados do giroscópio
    return ESP_OK;  // Retorna sucesso na leitura dos dados
}

/*
  @brief Calcula o quaternion com base nos dados do IMU
  
  Esta função calcula um quaternion simplificado utilizando os dados de aceleração do IMU.
 
  @param data Ponteiro para os dados do IMU
  @param quaternion Ponteiro para a estrutura onde o quaternion será armazenado
  @return esp_err_t Retorna ESP_OK se o cálculo for bem-sucedido, caso contrário, retorna um erro.
 */
esp_err_t imu_calculate_quaternion(const IMUData *data, Quaternion *quaternion) {
    // Exemplo simplificado de cálculo de quaternion com base nos dados de aceleração
    quaternion->w = 1.0;
    quaternion->x = data->accel.x;
    quaternion->y = data->accel.y;
    quaternion->z = data->accel.z;
    return ESP_OK;  // Retorna sucesso no cálculo
}

/**
  @brief Calcula os ângulos de Euler com base no quaternion
  
  Esta função calcula os ângulos de Euler (roll, pitch, yaw) com base no quaternion fornecido.
 
  @param quaternion Ponteiro para o quaternion calculado
  @param euler Ponteiro para a estrutura onde os ângulos de Euler serão armazenados
  @return esp_err_t Retorna ESP_OK se o cálculo for bem-sucedido, caso contrário, retorna um erro.
 */
esp_err_t imu_calculate_euler_angles(const Quaternion *quaternion, EulerAngle *euler) {
    // Exemplo simplificado de cálculo dos ângulos de Euler com base no quaternion
    euler->roll = atan2f(2.0f * (quaternion->w * quaternion->x + quaternion->y * quaternion->z),
                         1.0f - 2.0f * (quaternion->x * quaternion->x + quaternion->y * quaternion->y));
    euler->pitch = asinf(2.0f * (quaternion->w * quaternion->y - quaternion->z * quaternion->x));
    euler->yaw = atan2f(2.0f * (quaternion->w * quaternion->z + quaternion->x * quaternion->y),
                        1.0f - 2.0f * (quaternion->y * quaternion->y + quaternion->z * quaternion->z));
    return ESP_OK;  // Retorna sucesso no cálculo
}

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
