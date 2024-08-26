#include "uart_driver.h"
#include "freertos/FreeRTOS.h"

bool uart_init(uart_t *uart) {
    return true;
}

void uart_deinit(uart_t *uart) {}

bool uart_write(uart_t *uart, uint8_t *data, uint32_t size) {
    return true;
}

bool uart_read(uart_t *uart, uint8_t *data, uint32_t size) {
    return true;
}