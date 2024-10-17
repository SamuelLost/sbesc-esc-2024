#include <stdio.h>
#include "lora_module.h"
#include "mpu6050.h"
#include "esp_log.h"
#include "utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "MAIN"

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
    char device[16];
    int device_id;
    float temp, hum;

    while (true) {
        if (lora_module_receive(&lora_config, &packet)) {
            ESP_LOGI(TAG, "Received: %ld bytes", packet.size);

            // Calcula o tamanho da mensagem
            size_t message_size = packet.size - 5;  // Subtrai 3 (ID + comando) e 2 (CRC)
            if (message_size > 0) {
                char msg[message_size];
                for (int i = 0; i < message_size; i++) {
                    msg[i] = packet.buffer[i + 3];
                }

                if (packet.buffer[2] == 0x11) {
                    sscanf(msg, "%15[^,],%d,%f,%f,%f", device, &device_id, &accel_data.accel_x.converted, &accel_data.accel_y.converted, &accel_data.accel_z.converted);
                    ESP_LOGI(TAG, "Acceleration received");
                    ESP_LOGI(TAG, "Device: %s, ID: %d, X: %.2f, Y: %.2f, Z: %.2f", device, device_id, accel_data.accel_x.converted, accel_data.accel_y.converted, accel_data.accel_z.converted);
                }
                else if (packet.buffer[2] == 0x12) {
                    sscanf(msg, "%15[^,],%d,%f,%f", device, &device_id, &temp, &hum);
                    ESP_LOGI(TAG, "Temperature received");
                    ESP_LOGI(TAG, "Device: %s, ID: %d, Temperature: %.2f, Humidity: %.2f", device, device_id, temp, hum);
                } else if (packet.buffer[2] == 0x13) {
                    sscanf(msg, "%15[^,],%d,%hu", device, &device_id, &distance);
                    ESP_LOGI(TAG, "Distance received");
                    ESP_LOGI(TAG, "Device: %s, ID: %d, Distance: %hu", device, device_id, distance);
                } else {
                    ESP_LOGI(TAG, "Message: %.*s", message_size, msg);
                }
            }

        }

        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}
