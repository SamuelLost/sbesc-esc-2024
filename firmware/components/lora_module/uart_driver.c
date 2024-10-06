#include "uart_driver.h"
#include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
#include "utils.h"
#include "esp_log.h"

#define RX_BUF_SIZE 1024
#define TX_BUF_SIZE 1024

#define IS_VALID_UART(port_num) ((port_num) >= UART_NUM_0 && (port_num) < UART_NUM_MAX)

#define TAG "UART_DRIVER"

bool uart_init(uart_t *uart) {
    if (!IS_VALID(uart)) {
        return false;
    }

    if (!IS_VALID_UART(uart->uart_port)) {
        return false;
    }

    uart_config_t uart_config = {
        .baud_rate = uart->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    if (uart_param_config(uart->uart_port, &uart_config) == ESP_FAIL) {
        return false;
    }

    if (uart_set_pin(uart->uart_port, uart->tx_pin, uart->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) == ESP_FAIL) {
        return false;
    }

    if (uart_driver_install(uart->uart_port, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0) == ESP_FAIL) {
        return false;
    }

    return true;

}

void uart_deinit(uart_t *uart) {
    if (!IS_VALID(uart)) {
        return;
    }

    if (!IS_VALID_UART(uart->uart_port)) {
        return;
    }

    uart_driver_delete(uart->uart_port);
}

size_t uart_write(uart_t *uart, uint8_t *data, uint32_t size) {
    if(!IS_VALID(uart) || !IS_VALID(data) || size == 0) {
        ESP_LOGE(TAG, "Invalid parameters");
        return 0;
    }

    if (!IS_VALID_UART(uart->uart_port)) {
        ESP_LOGE(TAG, "Invalid UART port number");
        return 0;
    }

    int bytes_written = uart_write_bytes(uart->uart_port, (const char *)data, size);
    if (bytes_written < 0) {
        ESP_LOGE(TAG, "Failed to write data to UART");
        return 0;
    }

    return bytes_written;
}

size_t uart_read(uart_t *uart, uint8_t *data, uint32_t size) {
    if(!IS_VALID(uart) || !IS_VALID(data) || size == 0) {
        ESP_LOGE(TAG, "Invalid parameters");
        return 0;
    }

    if (!IS_VALID_UART(uart->uart_port)) {
        ESP_LOGE(TAG, "Invalid UART port number");
        return 0;
    }

    int bytes_read = uart_read_bytes(uart->uart_port, data, size, 1000 / portTICK_PERIOD_MS);
    if (bytes_read < 0) {
        ESP_LOGE(TAG, "Failed to read data from UART");
        return 0;
    }

    return bytes_read;
}

size_t uart_read_timeout(uart_t *uart, uint8_t *data, uint32_t size, int timeout_ms) {
    if(!IS_VALID(uart) || !IS_VALID(data) || size == 0) {
        ESP_LOGE(TAG, "Invalid parameters");
        return 0;
    }

    if (!IS_VALID_UART(uart->uart_port)) {
        ESP_LOGE(TAG, "Invalid UART port number");
        return 0;
    }

    int bytes_read = uart_read_bytes(uart->uart_port, data, size, timeout_ms / portTICK_PERIOD_MS);
    if (bytes_read < 0) {
        ESP_LOGE(TAG, "Failed to read data from UART");
        return 0;
    }

    return bytes_read;
}