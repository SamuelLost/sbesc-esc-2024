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
#define LENGTH_PIPE 72

const char *ACCEL_TOPICS[] = {"acc/x", "acc/y", "acc/z"};
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
};

void vTaskAccelerometer(void *pvParameters);
void vTaskLora(void *pvParameters);

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
    uint16_t distance;
    char device[6];
    int device_id;
    float temp, hum;
    char msg[232];
    char temp_msg[30];
    size_t message_size;

    while (true) {
        if (lora_module_receive(&lora_config, &packet)) {
            ESP_LOGI(TAG, "Received: %ld bytes", packet.size);

            // Calcula o tamanho da mensagem
            message_size = packet.size - 5;  // Subtrai 3 (ID + comando) e 2 (CRC)
            if (message_size > 0) {
                memset(msg, 0, sizeof(msg));
                memcpy(msg, &packet.buffer[3], message_size);
                msg[message_size] = '\0';

                switch (packet.buffer[2]) {
                    case CMD_ACCELEROMETER:
                        sscanf(msg, "%5[^,],%d,%f,%f,%f", device, &device_id, &accel_data.accel_x.converted, &accel_data.accel_y.converted, &accel_data.accel_z.converted);
                        ESP_LOGI(TAG, "Acceleration received");
                        ESP_LOGI(TAG, "Device: %s, ID: %d, X: %.2f, Y: %.2f, Z: %.2f", device, device_id, accel_data.accel_x.converted, accel_data.accel_y.converted, accel_data.accel_z.converted);

                        for (int i = 0; i < 3; i++) {
                            snprintf(temp_msg, sizeof(temp_msg), "%.2f", ((float*)&accel_data)[i]);
                            mqtt_publish(ACCEL_TOPICS[i], temp_msg);
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
                        distance = distance - LENGTH_PIPE;
                        ESP_LOGI(TAG, "Distance received");
                        ESP_LOGI(TAG, "Device: %s, ID: %d, Distance: %hu", device, device_id, distance);

                        snprintf(temp_msg, sizeof(temp_msg), "%hu", distance);
                        mqtt_publish(LASER_TOPIC, temp_msg);

                        break;
                    default:
                        ESP_LOGI(TAG, "Message: %.*s", (int)message_size, msg);
                        break;
                }
            }

        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}
