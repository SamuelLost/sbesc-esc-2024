#include <stdio.h>
#include <string.h>
#include "mpu6050.h"
#include "sht30.h"
#include "laser_sensor.h"
#include "lora_module.h"
#include "esp_log.h"
#include "utils.h"
#include "uart_driver.h"

#define TAG "MAIN"

typedef struct local_data_t {
    mpu6050_t mpu6050_config;
    lora_module_t lora_config;
    laser_sensor_t laser_config;
    sht30_t sht30_config;
} local_data_t;

static local_data_t local_data = {};

void vTaskAccelerometer(void *pvParameters);
void vTaskLaserSensor(void *pvParameters);
void vTaskLora(void *pvParameters);
void vTaskTemperature(void *pvParameters);

void app_main(void) {

    local_data.mpu6050_config = (mpu6050_t) {
        .i2c = {
            .i2c_port = I2C_NUM_0,
            .sda_pin = GPIO_NUM_21,
            .scl_pin = GPIO_NUM_22,
            .i2c_freq_t = I2C_FREQ_400KH,
        },
        .addr = MPU6050_DEFAULT_ADDR,
        .accel_range = A2G,
        .gyro_range = G250DPS,
    };  

    if (!mpu6050_init(&local_data.mpu6050_config)) {
        ESP_LOGE(TAG, "MPU6050 initialization failed");
        RESTART(TAG, TIME_TO_RESTART);
    }

    ESP_LOGI(TAG, "MPU6050 initialized");


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
        .device_id = 0x00,
    };

    if (!lora_module_init(&local_data.lora_config)) {
        ESP_LOGE(TAG, "LoRa module initialization failed");
        RESTART(TAG, TIME_TO_RESTART);
    }

    ESP_LOGI(TAG, "LoRa module initialized");

    // Configuração utilizada pelo sensor de distância laser
    local_data.laser_config = (laser_sensor_t) {
        .uart_port = UART_NUM_1,
        .baud_rate = 9600,
        .tx_pin = GPIO_NUM_4,
        .rx_pin = GPIO_NUM_5,
    };

    if (!laser_sensor_init(&local_data.laser_config)) {
        ESP_LOGE(TAG, "Laser sensor initialization failed");
        RESTART(TAG, TIME_TO_RESTART);
    }

    ESP_LOGI(TAG, "Laser sensor initialized");

    // Configuração utilizada pelo sensor de temperatura e umidade
    local_data.sht30_config = (sht30_t) {
        .i2c = {
            .i2c_port = I2C_NUM_1,
            .sda_pin = GPIO_NUM_33,
            .scl_pin = GPIO_NUM_32,
            .i2c_freq_t = I2C_FREQ_100KH,
        },
        .addr = SHT30_DEFAULT_ADDR,
        .mode = SHT30_SINGLE_SHOT,
        .repeat = SHT30_HIGH,
    };

    if (!sht30_init(&local_data.sht30_config)) {
        ESP_LOGE(TAG, "SHT30 initialization failed");
        RESTART(TAG, TIME_TO_RESTART);
    }

    ESP_LOGI(TAG, "SHT30 initialized");
    
    xTaskCreatePinnedToCore(vTaskAccelerometer, 
                            "Accelerometer Task", 
                            2048, 
                            NULL, 
                            5, 
                            NULL, 
                            APP_CPU_NUM);

    xTaskCreatePinnedToCore(vTaskLaserSensor, 
                            "Laser sensor Task", 
                            2048, 
                            NULL, 
                            5, 
                            NULL, 
                            APP_CPU_NUM);

    xTaskCreatePinnedToCore(vTaskLora,
                            "Lora Task",
                            2048,
                            NULL,
                            5,
                            NULL,
                            APP_CPU_NUM);

    xTaskCreatePinnedToCore(vTaskTemperature,
                            "Temperature Task",
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

        vTaskDelay(1500 / portTICK_PERIOD_MS);
    }
}

void vTaskLora(void *pvParameters) {
    UNUSED(pvParameters);

    lora_packet_t packet;
    char data[] = "Hello, LoRa!";
    uint16_t id_broadcast = 2047;
    prepare_lora_packet(id_broadcast, CMD_APPLICATION, data, &packet);

    while (true) {
        if (!lora_module_send(&local_data.lora_config, &packet)) {
            ESP_LOGE(TAG, "Error sending packet");
        } else {
            ESP_LOGI(TAG, "Packet with %lu bytes sent successfully", packet.size);
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void vTaskLaserSensor(void *pvParameters) {
    UNUSED(pvParameters);

    uint16_t distance = 0;

    while (true) {
        distance = laser_sensor_get_value(&local_data.laser_config);
        ESP_LOGI(TAG, "Distance: %d mm", distance);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void vTaskTemperature(void *pvParameters) {
    UNUSED(pvParameters);

    sht30_data_t sht30_data = {};

    while (true) {
        if (sht30_measure(&local_data.sht30_config, &sht30_data)) {
            ESP_LOGI(TAG, "Temperature: %.2f °C, Humidity: %.2f %%", sht30_data.temperature, sht30_data.humidity);
        }

        vTaskDelay(1500 / portTICK_PERIOD_MS);
    }
}