#include <stdio.h>
#include <string.h>
#include "mpu6050.h"
#include "sht30.h"
#include "laser_sensor.h"
#include "lora_module.h"
#include "esp_log.h"
#include "utils.h"
#include "uart_driver.h"
#include "freertos/queue.h"

#define TAG "MAIN"
#define ID_BROADCAST 1

static mpu6050_t mpu6050_config = {
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

static lora_module_t lora_config = {
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

static laser_sensor_t laser_config = {
    .uart_port = UART_NUM_1,
    .baud_rate = 9600,
    .tx_pin = GPIO_NUM_4,
    .rx_pin = GPIO_NUM_5,
};

static sht30_t sht30_config = {
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

static QueueHandle_t queue_lora_packets;

static SemaphoreHandle_t lora_mutex;

void vTaskAccelerometer(void *pvParameters);
void vTaskLaserSensor(void *pvParameters);
void vTaskLora(void *pvParameters);
void vTaskTemperature(void *pvParameters);

void app_main(void) {

    if (!mpu6050_init(&mpu6050_config)) {
        ESP_LOGE(TAG, "MPU6050 initialization failed");
        RESTART(TAG, TIME_TO_RESTART);
    }

    ESP_LOGI(TAG, "MPU6050 initialized");

    if (!lora_module_init(&lora_config)) {
        ESP_LOGE(TAG, "LoRa module initialization failed");
        RESTART(TAG, TIME_TO_RESTART);
    }

    ESP_LOGI(TAG, "LoRa module initialized");

    if (!laser_sensor_init(&laser_config)) {
        ESP_LOGE(TAG, "Laser sensor initialization failed");
        RESTART(TAG, TIME_TO_RESTART);
    }

    ESP_LOGI(TAG, "Laser sensor initialized");

    if (!sht30_init(&sht30_config)) {
        ESP_LOGE(TAG, "SHT30 initialization failed");
        RESTART(TAG, TIME_TO_RESTART);
    }

    ESP_LOGI(TAG, "SHT30 initialized");

    queue_lora_packets = xQueueCreate(10, sizeof(lora_packet_t));

    lora_mutex = xSemaphoreCreateMutex();
    
    xTaskCreatePinnedToCore(vTaskAccelerometer, 
                            "Accelerometer Task", 
                            2048 * 2, 
                            NULL, 
                            5, 
                            NULL, 
                            APP_CPU_NUM);

    xTaskCreatePinnedToCore(vTaskLaserSensor, 
                            "Laser sensor Task", 
                            2048*2, 
                            NULL, 
                            5, 
                            NULL, 
                            APP_CPU_NUM);

    xTaskCreatePinnedToCore(vTaskLora,
                            "Lora Task",
                            2048*2,
                            NULL,
                            5,
                            NULL,
                            APP_CPU_NUM);

    xTaskCreatePinnedToCore(vTaskTemperature,
                            "Temperature Task",
                            2048 * 2,
                            NULL,
                            5,
                            NULL,
                            APP_CPU_NUM);
}

void vTaskAccelerometer(void *pvParameters) {
    acceleration_data_t accel_data = {};
    lora_packet_t packet;
    char data[50];

    while (true) {

        if (mpu6050_get_acceleration(&mpu6050_config, &accel_data, ACCEL_MPS2)) {
            ESP_LOGI("m/s²", "X: %.2f, Y: %.2f, Z: %.2f", accel_data.accel_x.converted, accel_data.accel_y.converted, accel_data.accel_z.converted);

            snprintf(data, sizeof(data), "ACC,%d,%.2f,%.2f,%.2f", lora_config.device_id, accel_data.accel_x.converted, accel_data.accel_y.converted, accel_data.accel_z.converted);
            prepare_lora_packet(ID_BROADCAST, CMD_ACCELEROMETER, data, &packet); 

            xQueueSend(queue_lora_packets, &packet, portMAX_DELAY);
        }

        vTaskDelay(4000 / portTICK_PERIOD_MS);
    }
}

void vTaskLaserSensor(void *pvParameters) {
    UNUSED(pvParameters);

    uint16_t distance = 0, last_distance = 0;
    lora_packet_t packet;
    char data[50];

    while (true) {
        distance = laser_sensor_get_value(&laser_config);
        
        if (abs(distance - last_distance) > 10) {
            ESP_LOGI(TAG, "Distance: %d mm", distance);

            snprintf(data, sizeof(data), "LS,%d,%d", lora_config.device_id, distance);
            prepare_lora_packet(ID_BROADCAST, CMD_LASER, data, &packet);

            xQueueSend(queue_lora_packets, &packet, portMAX_DELAY);
            last_distance = distance;
        }

        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

void vTaskTemperature(void *pvParameters) {
    UNUSED(pvParameters);

    sht30_data_t sht30_data = {};
    lora_packet_t packet;
    char data[50];

    while (true) {
        if (sht30_measure(&sht30_config, &sht30_data)) {
            ESP_LOGI(TAG, "Temperature: %.2f °C, Humidity: %.2f %%", sht30_data.temperature, sht30_data.humidity);
            
            snprintf(data, sizeof(data), "TMP,%d,%.2f,%.2f", lora_config.device_id, sht30_data.temperature, sht30_data.humidity);
            prepare_lora_packet(ID_BROADCAST, CMD_TEMPERATURE, data, &packet);

            xQueueSend(queue_lora_packets, &packet, portMAX_DELAY);
        }

        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void vTaskLora(void *pvParameters) {
    UNUSED(pvParameters);

    lora_packet_t packet;

    while (true) {
        if (xQueueReceive(queue_lora_packets, &packet, portMAX_DELAY)) {
            if (xSemaphoreTake(lora_mutex, portMAX_DELAY) == pdTRUE) {
                if (!lora_module_send(&lora_config, &packet)) {
                    ESP_LOGE(TAG, "Error sending packet");
                } else {
                    ESP_LOGI(TAG, "Packet with %lu bytes sent successfully", packet.size);
                }
                xSemaphoreGive(lora_mutex);
            }
        }

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}