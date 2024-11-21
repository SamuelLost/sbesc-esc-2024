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
    .password = 4321,
};

static laser_sensor_t laser_config = {
    .uart_port = UART_NUM_1,
    .baud_rate = 9600,
    .tx_pin = GPIO_NUM_4, // Fio amarelo
    .rx_pin = GPIO_NUM_5, // Fio branco
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
static SemaphoreHandle_t laser_mutex;
static SemaphoreHandle_t sht30_mutex;
static SemaphoreHandle_t mpu6050_mutex;
static TaskHandle_t lora_handle = NULL;
static TaskHandle_t laser_handle = NULL;
static TaskHandle_t temperature_handle = NULL;
static TaskHandle_t accelerometer_handle = NULL;
static TaskHandle_t heartbeat_handle = NULL;

void vTaskAccelerometer(void *pvParameters);
void vTaskLaserSensor(void *pvParameters);
void vTaskLora(void *pvParameters);
void vTaskTemperature(void *pvParameters);
void vTaskHeartbeat(void *pvParameters);

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
    laser_mutex = xSemaphoreCreateMutex();
    sht30_mutex = xSemaphoreCreateMutex();
    mpu6050_mutex = xSemaphoreCreateMutex();

    if (!queue_lora_packets || !lora_mutex || !laser_mutex || !sht30_mutex || !mpu6050_mutex) {
        ESP_LOGE(TAG, "Error creating queue or mutex");
        RESTART(TAG, TIME_TO_RESTART);
    }

    xTaskCreatePinnedToCore(vTaskAccelerometer, 
                            "AccelTask", 
                            2048 * 2, 
                            NULL, 
                            tskIDLE_PRIORITY + 3, 
                            &accelerometer_handle, 
                            APP_CPU_NUM);

    xTaskCreatePinnedToCore(vTaskLaserSensor, 
                            "LaserTask", 
                            2048*2, 
                            NULL, 
                            tskIDLE_PRIORITY + 2, 
                            &laser_handle, 
                            APP_CPU_NUM);

    xTaskCreatePinnedToCore(vTaskLora,
                            "LoRaTask",
                            2048*2,
                            NULL,
                            configMAX_PRIORITIES - 1,
                            &lora_handle,
                            PRO_CPU_NUM);

    xTaskCreatePinnedToCore(vTaskTemperature,
                            "TempTask",
                            2048 * 2,
                            NULL,
                            tskIDLE_PRIORITY + 2,
                            &temperature_handle,
                            APP_CPU_NUM);

    xTaskCreatePinnedToCore(vTaskHeartbeat,
                            "HeartbeatTask",
                            2048,
                            NULL,
                            tskIDLE_PRIORITY + 1,
                            &heartbeat_handle,
                            PRO_CPU_NUM);

    if (!lora_handle || !laser_handle || !temperature_handle || !accelerometer_handle || !heartbeat_handle) {
        ESP_LOGE(TAG, "Error creating tasks");
        RESTART(TAG, TIME_TO_RESTART);
    }
}

void vTaskAccelerometer(void *pvParameters) {
    // acceleration_data_t accel_data = {};
    // gyroscope_data_t gyro_data = {};
    angles_data_t angles_data = {};
    lora_packet_t packet;
    char data[30];

    while (true) {

        // if (mpu6050_get_acceleration(&mpu6050_config, &accel_data, ACCEL_MPS2)) {
        //     ESP_LOGI("m/s²", "X: %.2f, Y: %.2f, Z: %.2f", accel_data.accel_x.converted, accel_data.accel_y.converted, accel_data.accel_z.converted);

        //     snprintf(data, sizeof(data), "ACC,%d,%.2f,%.2f,%.2f", lora_config.device_id, accel_data.accel_x.converted, accel_data.accel_y.converted, accel_data.accel_z.converted);
        //     prepare_lora_packet(ID_BROADCAST, CMD_ACCELEROMETER, data, &packet); 

        //     xQueueSend(queue_lora_packets, &packet, portMAX_DELAY);
        // }
        if (xSemaphoreTake(mpu6050_mutex, portMAX_DELAY) == pdTRUE) {
            
            if (mpu6050_get_angles(&mpu6050_config, &angles_data, -1)) {
                ESP_LOGI(TAG, "ACC Pitch: %.2f, ACC Roll: %.2f", angles_data.accel.pitch, angles_data.accel.roll);
                
                snprintf(data, sizeof(data), "ACC,%d,%.2f,%.2f", lora_config.device_id, angles_data.accel.pitch, angles_data.accel.roll);
                prepare_lora_packet(ID_BROADCAST, CMD_ACCELEROMETER, data, &packet);

                xQueueSend(queue_lora_packets, &packet, portMAX_DELAY);
            }
            xSemaphoreGive(mpu6050_mutex);
        }

        vTaskDelay(4000 / portTICK_PERIOD_MS);
    }
}

void vTaskLaserSensor(void *pvParameters) {
    UNUSED(pvParameters);

    uint16_t distance = 0, last_distance = 0;
    lora_packet_t packet;
    char data[15];

    while (true) {
        if (xSemaphoreTake(laser_mutex, portMAX_DELAY) == pdTRUE) {
            distance = laser_sensor_get_value(&laser_config);
            xSemaphoreGive(laser_mutex);
        }
        
        if (abs(distance - last_distance) > 10) {
            ESP_LOGI(TAG, "Distance: %hu mm", distance);

            snprintf(data, sizeof(data), "LS,%d,%hu", lora_config.device_id, distance);
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
    char data[30];

    while (true) {
        if (xSemaphoreTake(sht30_mutex, portMAX_DELAY) == pdTRUE) {
            
            if (sht30_measure(&sht30_config, &sht30_data)) {
                ESP_LOGI(TAG, "Temperature: %.2f °C, Humidity: %.2f %%", sht30_data.temperature, sht30_data.humidity);
                
                snprintf(data, sizeof(data), "TMP,%d,%.2f,%.2f", lora_config.device_id, sht30_data.temperature, sht30_data.humidity);
                prepare_lora_packet(ID_BROADCAST, CMD_TEMPERATURE, data, &packet);

                xQueueSend(queue_lora_packets, &packet, portMAX_DELAY);
            }
            xSemaphoreGive(sht30_mutex);
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

void vTaskHeartbeat(void *pvParameters) {
    UNUSED(pvParameters);
    UBaseType_t task_acc;
    UBaseType_t task_laser;
    UBaseType_t task_temp;
    UBaseType_t task_lora;
    UBaseType_t task_heartbeat;
    uint32_t uptime = 0;
    uint32_t heap = 0;
    char data[60];
    lora_packet_t packet;
    bool mpu6050_status = false;
    bool laser_status = false;
    bool sht30_status = false;
    uint16_t status = 0;


    while (true) {
        task_acc = uxTaskGetStackHighWaterMark(accelerometer_handle) * 4; // Stack size in bytes
        task_laser = uxTaskGetStackHighWaterMark(laser_handle) * 4;
        task_temp = uxTaskGetStackHighWaterMark(temperature_handle) * 4;
        task_lora = uxTaskGetStackHighWaterMark(lora_handle) * 4;
        task_heartbeat = uxTaskGetStackHighWaterMark(heartbeat_handle) * 4;
        uptime = esp_log_timestamp() / 1000; // Uptime in seconds
        // heap = esp_get_free_heap_size(); // Free heap memory in bytes
        if (xSemaphoreTake(mpu6050_mutex, portMAX_DELAY) == pdTRUE) {
            mpu6050_status = mpu6050_is_alive(&mpu6050_config);
            xSemaphoreGive(mpu6050_mutex);
        }
        if (xSemaphoreTake(laser_mutex, portMAX_DELAY) == pdTRUE) {
            laser_status = laser_sensor_get_value(&laser_config);
            xSemaphoreGive(laser_mutex);
        }
        if (xSemaphoreTake(sht30_mutex, portMAX_DELAY) == pdTRUE) {
            sht30_status = sht30_get_status(&sht30_config, &status);
            xSemaphoreGive(sht30_mutex);
        }

        snprintf(data, sizeof(data), "HB,%d,%d,%d,%d,%d,%d,%lu,%d,%d,%d", lora_config.device_id, task_acc, task_laser, task_temp, 
            task_lora, task_heartbeat, uptime, mpu6050_status, laser_status, sht30_status);
        prepare_lora_packet(ID_BROADCAST, CMD_HEARTBEAT, data, &packet);

        xQueueSend(queue_lora_packets, &packet, portMAX_DELAY);
        
        vTaskDelay(pdMS_TO_TICKS(15000));
    }
}
    