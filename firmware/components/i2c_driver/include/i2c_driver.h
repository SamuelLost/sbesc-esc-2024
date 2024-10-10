#ifndef _I2C_DRIVER_H_
#define _I2C_DRIVER_H_

#include "esp_err.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct i2c_t {
    i2c_port_t i2c_port;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    enum {
        I2C_FREQ_100KH = 100000,
        I2C_FREQ_400KH = 400000,
    } i2c_freq_t;
} i2c_t;

/**
 * @brief Initialize the I2C driver
 * 
 * @param i2c I2C configuration structure
 * @return bool 
 */

bool i2c_init(i2c_t *i2c);

/**
 * @brief Deinitialize the I2C driver
 * 
 * @param i2c I2C configuration structure
 */
void i2c_deinit(i2c_t *i2c);

/**
 * @brief Write data to the I2C bus
 * 
 * @param i2c I2C configuration structure
 * @param slave_addr Device I2C address
 * @param data_to_write Data to write to the device
 * @param write_size Size of the data to write
 * @param timeout_ms Timeout in milliseconds
 * @return true
 * @return false 
 */
bool i2c_write(i2c_t *i2c, uint8_t slave_addr, uint8_t *data_to_write, uint32_t write_size, uint32_t timeout_ms);

/**
 * @brief Write and read data from the I2C bus
 * 
 * @param i2c I2C configuration structure
 * @param slave_addr Device I2C address
 * @param data_to_write Data to write to the device
 * @param write_size Size of the data to write
 * @param data_read Data read from the device
 * @param read_size Size of the data to read
 * @param timeout_ms Timeout in milliseconds
 * @return true 
 * @return false 
 */
bool i2c_write_read(i2c_t *i2c, uint8_t slave_addr, uint8_t *data_to_write, uint32_t write_size, uint8_t *data_read, uint32_t read_size, uint32_t timeout_ms);

/**
 * @brief Read data from the I2C bus
 * 
 * @param i2c I2C configuration structure
 * @param slave_addr Device I2C address
 * @param data_read Data read from the device
 * @param read_size Size of the data to read
 * @param timeout_ms Timeout in milliseconds
 * @return true 
 * @return false 
 */
bool i2c_read(i2c_t *i2c, uint8_t slave_addr, uint8_t *data_read, uint32_t read_size, uint32_t timeout_ms);

/**
 * @brief Read a single byte from the I2C bus
 * 
 * @param i2c I2C configuration structure
 * @param slave_addr Device I2C address
 * @param reg_addr Register address
 * @param timeout_ms Timeout in milliseconds
 * @return uint8_t 
 */
uint8_t i2c_read_byte(i2c_t *i2c, uint8_t slave_addr, uint8_t reg_addr, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // _I2C_DRIVER_H_