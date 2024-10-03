#include <stdio.h>
#include "mpu6050.h"
#include "sht30.h"
#include "lora_module.h"
#include "ir_sensor.h"
#include "esp_log.h"
#include "utils.h"
#include "uart_driver.h"

#define TAG "MAIN"

typedef struct local_data_t {
    mpu6050_t mpu6050_config;
    lora_module_t lora_config;
    uart_t uart_config;
} local_data_t;

static local_data_t local_data = {};

void vTaskAccelerometer(void *pvParameters);
void vTaskLora(void *pvParameters);

void app_main(void) {

    local_data.mpu6050_config = (mpu6050_t) {
        .i2c = {
            .i2c_port = I2C_NUM_0,
            .sda_pin = GPIO_NUM_21,
            .scl_pin = GPIO_NUM_22,
        },
        .addr = MPU6050_DEFAULT_ADDR,
        .accel_range = A2G,
        .gyro_range = G250DPS,
    };  

    // if (!mpu6050_init(&local_data.mpu6050_config)) {
    //     ESP_LOGE(TAG, "MPU6050 initialization failed");
    //     RESTART(TAG, TIME_TO_RESTART);
    // }

    // ESP_LOGI(TAG, "MPU6050 initialized\n");


    local_data.lora_config = (lora_module_t){
        .uart_config = {
            .uart_port = UART_NUM_2,
            .baud_rate = 9600,
            .tx_pin = GPIO_NUM_17,
            .rx_pin = GPIO_NUM_16,
        },
        .spread_factor = SF_7,
        .coding_rate = CR_4_5,
        .bandwidth = BANDWIDTH_125KHZ,
        .lora_class = LORA_CLASS_C,
        .lora_window = LORA_WINDOW_5s,
    };

    if (!lora_module_init(&local_data.lora_config)) {
        ESP_LOGE(TAG, "LoRa module initialization failed");
        RESTART(TAG, TIME_TO_RESTART);
        uint8_t data[] = "Erro ao inicializar o módulo LoRa\r\n";
        uart_write(&local_data.lora_config.uart_config, data, sizeof(data));
    }

    ESP_LOGI(TAG, "LoRa module initialized");

    // Configuração utilizada pelo sensor de distância laser
    local_data.uart_config = (uart_t) {
        .uart_port = UART_NUM_1,
        .baud_rate = 115200,
        .tx_pin = GPIO_NUM_4,
        .rx_pin = GPIO_NUM_5,
    };

    if (!uart_init(&local_data.uart_config)) {
        ESP_LOGE(TAG, "UART initialization failed");
        RESTART(TAG, TIME_TO_RESTART);
    }
    
    // xTaskCreatePinnedToCore(vTaskAccelerometer, 
    //                         "Accelerometer Task", 
    //                         2048, 
    //                         NULL, 
    //                         5, 
    //                         NULL, 
    //                         APP_CPU_NUM);

    xTaskCreatePinnedToCore(vTaskLora,
                            "Lora Task",
                            2048,
                            NULL,
                            5,
                            NULL,
                            APP_CPU_NUM);
}

void vTaskAccelerometer(void *pvParameters) {
    acceleration_data_t accel_data = {};

    while (true) {
        if(mpu6050_get_acceleration(&local_data.mpu6050_config, &accel_data, ACCEL_RAW)) {
            ESP_LOGI("RAW", "X: %d, Y: %d, Z: %d", accel_data.accel_x.raw, accel_data.accel_y.raw, accel_data.accel_z.raw);
        }

        if(mpu6050_get_acceleration(&local_data.mpu6050_config, &accel_data, ACCEL_G)) {
            ESP_LOGI("g-force", "X: %f, Y: %f, Z: %f", accel_data.accel_x.converted, accel_data.accel_y.converted, accel_data.accel_z.converted);
        }

        if (mpu6050_get_acceleration(&local_data.mpu6050_config, &accel_data, ACCEL_MPS2)) {
            ESP_LOGI("m/s²", "X: %f, Y: %f, Z: %f\n", accel_data.accel_x.converted, accel_data.accel_y.converted, accel_data.accel_z.converted);
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

// Teste com UART
// TODO: Implementar a leitura e o envio de dados do módulo LoRa
void vTaskLora(void *pvParameters) {
    UNUSED(pvParameters);

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    uint8_t data[] = "Embedded Systems Competition (ESC) 2024 - Boia para Monitoramento e Prevenção de Inundações\r\n";
    uart_write(&local_data.uart_config, data, sizeof(data));

    while (true) {
        uint8_t data[13] = {};  // Aumenta o buffer para acomodar \r\n
        size_t bytes_read = uart_read(&local_data.uart_config, data, 11);  // Continua lendo até 11 bytes
        if (bytes_read) {
            // Adiciona \r\n no final dos dados recebidos
            data[bytes_read] = '\r';
            data[bytes_read + 1] = '\n';
            uart_write(&local_data.uart_config, data, bytes_read + 2);  // Escreve incluindo \r\n

            // Log da mensagem recebida
            ESP_LOGI(TAG, "Received: %.*s", bytes_read, data);  // Imprime apenas os dados lidos
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
