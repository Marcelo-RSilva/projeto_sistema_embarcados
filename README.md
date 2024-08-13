# Projeto Sistemas Embarcados IFPB

Este projeto visa a implementação do sensor MPU6050, criando nossa própria biblioteca para tal, que tem as funções de acelerômetro, giroscópio e medir a temperatura.

## Requisitos

- 1x MCU, no caso escolhemos o ESP32
- 1x MPU6050
- Builder
  - ESP-IDF
- Bibliotecas
  - stdio.h
  - math.h
  - freertos/FreeRTOS.h
  - freertos/task.h
  - driver/i2c.h
  - esp_err.h
  - driver/gpio.h

## Portas utilizadas na ESP32 pelo MPU6050
- SDA - GPIO PIN 21
- SCL - GPIO PIN 22


## Funções implementadas

- `imu_init(uint8_t devAddr, gpio_num_t sda_pin, gpio_num_t scl_pin)`: Inicializa a comunicação com o sensor IMU através do protocolo I2C, configurando os pinos SDA e SCL e verificando a presença do dispositivo através do registro "Who am I". Se o IMU responder corretamente, ele sai do modo de sono e está pronto para operar. A função retorna `ESP_OK` se a inicialização for bem-sucedida ou um código de erro em caso de falha.

- `imu_get_acceleration_data(AccelerationData *data)`: Lê os dados brutos de aceleração (nos eixos X, Y e Z) do IMU através da comunicação I2C e converte esses valores em unidades de gravidade (g). Os dados são armazenados na estrutura AccelerationData fornecida, e a função retorna `ESP_OK` em caso de sucesso ou `ESP_FAIL` se ocorrer um erro durante a leitura.

- `imu_get_gyroscope_data(GyroscopeData *data)`: Lê os dados brutos de rotação do giroscópio do IMU nos eixos X, Y e Z e os converte em graus por segundo (°/s). Os resultados são armazenados na estrutura `GyroscopeData` fornecida, e a função retorna `ESP_OK` se a leitura for bem-sucedida ou `ESP_FAIL` em caso de erro.

- `imu_deinit()`: Desinstala o driver I2C utilizado para a comunicação com o IMU, liberando os recursos associados ao driver e ao barramento I2C. A função retorna `ESP_OK` se a desinstalação for realizada com sucesso ou um código de erro se houver falha.

- `imu_read_data(IMUData *data)`: Combina a leitura dos dados de aceleração e giroscópio do IMU, utilizando as funções `imu_get_acceleration_data` e `imu_get_gyroscope_data`, armazenando os resultados na estrutura `IMUData`. A função retorna `ESP_OK` se ambas as leituras forem bem-sucedidas ou `ESP_FAIL` em caso de falha em qualquer uma das leituras.

- `imu_calculate_quaternion(const IMUData *data, Quaternion *quaternion)`: Calcula um quaternion simplificado a partir dos dados de aceleração lidos do IMU. Os valores calculados são armazenados na estrutura `Quaternion` fornecida. A função é um exemplo básico e retorna `ESP_OK` se o cálculo for bem-sucedido.

- `imu_calculate_euler_angles(const Quaternion *quaternion, EulerAngle *euler)`: Calcula os ângulos de Euler (roll, pitch, yaw) a partir de um quaternion fornecido. Os ângulos calculados são armazenados na estrutura `EulerAngle`, e a função retorna `ESP_OK` se o cálculo for realizado corretamente.


## Esquemático

![Esquemático](images/Schematic_ESP32_MPU6050.png)

## Autores

- [Antonio Roberto](https://github.com/antoniojunior2222)
- [Aryelson Gonçalves](https://github.com/aryelson1)
- [Clarissa Lucena](https://github.com/Clarissa-de-Lucena)
- [Felipe Bras](https://github.com/felipebrazfb333) 
- [Guilherme Santos](https://github.com/GuilhermexL)
- [Marcelo Ribeiro](https://github.com/Marcelo-RSilva)
