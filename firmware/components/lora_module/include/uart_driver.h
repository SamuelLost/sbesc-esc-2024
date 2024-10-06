#ifndef _UART_DRIVER_H_
#define _UART_DRIVER_H_

#include "driver/uart.h"
#include "driver/gpio.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct uart_t {
    uart_port_t uart_port;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    uint32_t baud_rate;
} uart_t;

/**
 * @brief Initialize the UART driver
 * 
 * @param uart UART configuration structure
 * @return true
 * @return false 
 */
bool uart_init(uart_t *uart);

/**
 * @brief Deinitialize the UART driver
 * 
 * @param uart UART configuration structure
 */
void uart_deinit(uart_t *uart);

/**
 * @brief Write data to the UART bus
 * 
 * @param uart UART configuration structure
 * @param data Data buffer
 * @param size Data size
 * @return size_t - Number of bytes written
 */
size_t uart_write(uart_t *uart, uint8_t *data, uint32_t size);

/**
 * @brief Read data from the UART bus
 * 
 * @param uart UART configuration structure
 * @param data Data buffer
 * @param size Data size
 * @return size_t - Number of bytes read 
 */
size_t uart_read(uart_t *uart, uint8_t *data, uint32_t size);

/**
 * @brief Read data from the UART bus
 * 
 * @param uart UART configuration structure
 * @param data Data buffer
 * @param size Data size
 * @param timeout_ms Timeout in milliseconds
 * @return size_t - Number of bytes read 
 */
size_t uart_read_timeout(uart_t *uart, uint8_t *data, uint32_t size, int timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // _UART_DRIVER_H_