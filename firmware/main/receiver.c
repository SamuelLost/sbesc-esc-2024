#include <stdio.h>
#include "lora_module.h"
#include "mpu6050.h"
#include "esp_log.h"
#include "utils.h"
#include <string.h>
#include "mqtt.h"
#include "wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "MAIN"
#define LENGTH_PIPE 77
#define LENGTH_FLOAT 31.4

const char *ACCEL_TOPICS[] = {"acc/x", "acc/y", "acc/z"};
const char *ANGLE_TOPICS[] = {"angle/pitch", "angle/roll"};
const char *TEMP_HUM_TOPIC[] = {"temperature", "humidity"};
const char *LASER_TOPIC = "distance";


lora_module_t lora_config = {
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
    .device_id = 1, //Default: 254
    .password = 4321,
};

void vTaskAccelerometer(void *pvParameters);
void vTaskLora(void *pvParameters);
float get_water_column_height(uint16_t distance);

void app_main(void) {

    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    mqtt_app_start();

    if (!lora_module_init(&lora_config)) {
        ESP_LOGE(TAG, "LoRa module initialization failed");
        RESTART(TAG, TIME_TO_RESTART);
    }

    ESP_LOGI(TAG, "LoRa module initialized");

    xTaskCreatePinnedToCore(vTaskLora,
                            "Lora Task",
                            2048*2,
                            NULL,
                            5,
                            NULL,
                            APP_CPU_NUM);

}

void vTaskLora(void *pvParameters) {
    UNUSED(pvParameters);
    lora_packet_t packet;
    acceleration_data_t accel_data = {};
    angles_data_t angles_data = {};
    uint16_t distance;
    char device[6];
    int device_id;
    float temp, hum, water_height;
    char msg[232];
    char temp_msg[30];
    size_t message_size;

    char task_acc[5];
    char task_laser[5];
    char task_temp[5];
    char task_lora[5];
    char task_heartbeat[5];
    char uptime[5];
    char mpu6050_status[2];
    char laser_status[2];
    char sht30_status[2];
    char lora_status[1];

    while (true) {
        if (lora_module_receive(&lora_config, &packet)) {
            ESP_LOGI(TAG, "Received: %ld bytes", packet.size);
            lora_status[0] = '1';
            mqtt_publish("status/lora", lora_status);
            // Calcula o tamanho da mensagem
            message_size = packet.size - 5;  // Subtrai 3 (ID + comando) e 2 (CRC)
            if (message_size > 0) {
                memset(msg, 0, sizeof(msg));
                memcpy(msg, &packet.buffer[3], message_size);
                msg[message_size] = '\0';

                switch (packet.buffer[2]) {
                    case CMD_ACCELEROMETER:
                        // sscanf(msg, "%5[^,],%d,%f,%f,%f", device, &device_id, &accel_data.accel_x.converted, &accel_data.accel_y.converted, &accel_data.accel_z.converted);
                        // ESP_LOGI(TAG, "Acceleration received");
                        // ESP_LOGI(TAG, "Device: %s, ID: %d, X: %.2f, Y: %.2f, Z: %.2f", device, device_id, accel_data.accel_x.converted, accel_data.accel_y.converted, accel_data.accel_z.converted);
                        // for (int i = 0; i < 3; i++) {
                        //     snprintf(temp_msg, sizeof(temp_msg), "%.2f", ((float*)&accel_data)[i]);
                        //     mqtt_publish(ACCEL_TOPICS[i], temp_msg);
                        // }

                        sscanf(msg, "%5[^,],%d,%f,%f", device, &device_id, &angles_data.accel.pitch, &angles_data.accel.roll);
                        ESP_LOGI(TAG, "Angles received");
                        ESP_LOGI(TAG, "Device: %s, ID: %d, Pitch: %.2f, Roll: %.2f", device, device_id, angles_data.accel.pitch, angles_data.accel.roll);

                        for (int i = 0; i < 2; i++) {
                            snprintf(temp_msg, sizeof(temp_msg), "%.2f", ((float*)&angles_data.accel)[i]);
                            mqtt_publish(ANGLE_TOPICS[i], temp_msg);
                        }

                        break;
                    case CMD_TEMPERATURE:
                        sscanf(msg, "%5[^,],%d,%f,%f", device, &device_id, &temp, &hum);
                        ESP_LOGI(TAG, "Temperature received");
                        ESP_LOGI(TAG, "Device: %s, ID: %d, Temperature: %.2f, Humidity: %.2f", device, device_id, temp, hum);

                        for (int i = 0; i < 2; i++) {
                            snprintf(temp_msg, sizeof(temp_msg), "%.2f", (i == 0) ? temp : hum);
                            mqtt_publish(TEMP_HUM_TOPIC[i], temp_msg);
                        }

                        break;
                    case CMD_LASER:
                        sscanf(msg, "%5[^,],%d,%hu", device, &device_id, &distance);
                        ESP_LOGI(TAG, "Distance received");
                        ESP_LOGI(TAG, "Device: %s, ID: %d, Distance: %hu", device, device_id, distance);
                        water_height = get_water_column_height(distance);
                        ESP_LOGI(TAG, "Water Column: %.2f", water_height);

                        snprintf(temp_msg, sizeof(temp_msg), "%.2f", water_height);
                        mqtt_publish(LASER_TOPIC, temp_msg);

                        break;
                    case CMD_HEARTBEAT:
                        // ESP_LOGI(TAG, "Heartbeat received: %.*s", (int)message_size, msg);
                        // snprintf(data, sizeof(data), "HB,%d,%d,%d,%d,%d,%d,%lu,%lu,%d,%d,%d", lora_config.device_id, task_acc, task_laser, task_temp, 
                                // task_lora, task_heartbeat, heap, uptime, mpu6050_status, laser_status, sht30_status);
                        
                        sscanf(msg, "%5[^,],%d,%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%1[^,],%1[^,],%1[^,]", device, &device_id, task_acc, task_laser, task_temp, 
                            task_lora, task_heartbeat, uptime, mpu6050_status, laser_status, sht30_status);
                        lora_status[0] = '1';
                        ESP_LOGI(TAG, "Heartbeat received");
                        ESP_LOGI(TAG, "Device: %s, ID: %d, Task Acc: %s, Task Laser: %s, Task Temp: %s, Task Lora: %s, Task Heartbeat: %s, Uptime: %s, MPU6050: %s, Laser: %s, SHT30: %s", 
                            device, device_id, task_acc, task_laser, task_temp, task_lora, task_heartbeat, uptime, mpu6050_status, laser_status, sht30_status);
                        
                        mqtt_publish("tasks/acc", task_acc);
                        mqtt_publish("tasks/laser", task_laser);
                        mqtt_publish("tasks/temp", task_temp);
                        mqtt_publish("tasks/lora", task_lora);
                        mqtt_publish("tasks/heartbeat", task_heartbeat);

                        mqtt_publish("status/uptime", uptime);
                        mqtt_publish("status/mpu6050", mpu6050_status);
                        mqtt_publish("status/laser", laser_status);
                        mqtt_publish("status/sht30", sht30_status);

                        break;
                    default:
                        ESP_LOGI(TAG, "Message: %.*s", (int)message_size, msg);
                        break;
                }
            } else {
                lora_status[0] = '0';
                mqtt_publish("status/lora", lora_status);
            }

        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

float get_water_column_height(uint16_t distance) {
    float distance_cm = distance / 10; // Convert mm to cm
    if ((distance_cm + LENGTH_FLOAT) > LENGTH_PIPE) {
        return 0;
    }

    return (LENGTH_PIPE - (distance_cm + LENGTH_FLOAT));
}
