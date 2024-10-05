#include <stdio.h>
#include "lora_module.h"
#include "esp_log.h"
#include "utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "MAIN"

typedef struct local_data_t {
    lora_module_t lora_config;
} local_data_t;

static local_data_t local_data = {};

void vTaskAccelerometer(void *pvParameters);
void vTaskLora(void *pvParameters);

void app_main(void) {

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
        .lora_window = LORA_WINDOW_15s,
        .device_id = 1, //Default: 254
    };

    if (!lora_module_init(&local_data.lora_config)) {
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

// Teste com UART
// TODO: Implementar a leitura e o envio de dados do módulo LoRa
void vTaskLora(void *pvParameters) {
    UNUSED(pvParameters);
    lora_packet_t packet;

    while (true) {
        if (lora_module_receive(&local_data.lora_config, &packet)) {
            ESP_LOGI(TAG, "Received: %ld bytes", packet.size);

            // Calcula o tamanho da mensagem
            size_t message_size = packet.size - 5;  // Subtrai 3 (ID + comando) e 2 (CRC)
            if (message_size > 0) {
                char msg[message_size];
                for (int i = 0; i < message_size; i++) {
                    msg[i] = packet.buffer[i + 3];
                }

                ESP_LOGI(TAG, "Message: %.*s", message_size, msg);
            }

        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
