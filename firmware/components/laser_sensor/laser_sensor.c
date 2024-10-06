#include <string.h>
#include "esp_log.h"
#include "laser_sensor.h"

uint8_t COMMAND_MEASURE_PASSIVE[] = "s5-1#"; // Command to measure distance in passive mode
uint8_t COMMAND_READ_PASSIVE[] = "r6#"; // Command to read distance in passive mode


const char *TAG_LASER = "LASER_SENSOR";

bool laser_sensor_init(laser_sensor_t *laser_sensor) {
    if (!uart_init(laser_sensor)) {
        ESP_LOGE(TAG_LASER, "Failed to initialize UART");
        return false;
    }
    
    // Set passive mode
    if (uart_write(laser_sensor, COMMAND_MEASURE_PASSIVE, sizeof(COMMAND_MEASURE_PASSIVE)-1) == 0) {
        ESP_LOGE(TAG_LASER, "Failed to send command to UART");
        return false;
    }
    
    uint8_t response[10] = {0};
    size_t size = 0;
    // Read response
    if ((size = uart_read(laser_sensor, response, 10)) == 0) {
        ESP_LOGE(TAG_LASER, "Failed to read response from UART");
        return false;
    }

    // Check if response is ok
    if (strstr((char*)response, "ok") == NULL) {
        ESP_LOGE(TAG_LASER, "Failed to set passive mode");
        return false;
    }

    return true;

}

void laser_sensor_deinit(laser_sensor_t *laser_sensor) {
    uart_deinit(laser_sensor);
}

uint16_t laser_sensor_get_value(laser_sensor_t *laser_sensor) {
    uint8_t data[12] = {0};
    uint16_t value = 0;
    size_t size = 0;

    // Send command to read distance
    if (uart_write(laser_sensor, COMMAND_READ_PASSIVE, sizeof(COMMAND_READ_PASSIVE)-1) == 0) {
        ESP_LOGE(TAG_LASER, "Failed to send command to UART");
        return 0;
    }

    // Read data
    if ((size = uart_read_timeout(laser_sensor, data, 12, 100)) == 0) {
        ESP_LOGE(TAG_LASER, "Failed to read data from UART");
        return 0;
    }

    // ESP_LOGI(TAG_LASER, "Data: %.*s", size, data);

    // Remove 'L=mm\r\n' from data manually
    // uint8_t result[size];
    // uint8_t j = 0;
    // for (size_t i = 0; i < size; i++) {
    //     if (data[i] != 'L' && data[i] != '=' && data[i] != 'm' && data[i] != '\r' && data[i] != '\n') {
    //         result[j++] = data[i];
    //     }
    // }
    // result[j] = '\0';

    // Remove 'L=mm\r\n' from data using strtok
    char *res = strtok((char*)data, "L=mm\r\n");

    if (res == NULL) {
        ESP_LOGE(TAG_LASER, "Failed to parse data");
        return 0;
    }

    value = atoi(res); // Convert string to integer

    return value;
}


