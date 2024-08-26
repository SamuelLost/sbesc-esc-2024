#ifndef _I2C_DRIVER_H_
#define _I2C_DRIVER_H_

#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct i2c_t {
    i2c_port_t i2c_port;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
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
 * @param addr Device I2C address
 * @param data Data buffer
 * @param size Data size
 * @return bool 
 */
bool i2c_write(i2c_t *i2c, uint8_t slave_addr, uint8_t reg_addr, uint8_t data, uint32_t timeout_ms);

/**
 * @brief Read data from the I2C bus
 * 
 * @param i2c I2C configuration structure
 * @param addr Device I2C address
 * @param data Data buffer
 * @param size Data size
 * @return bool 
 */
bool i2c_read(i2c_t *i2c, uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint32_t length, uint32_t timeout_ms);

/**
 * @brief Write a byte to the I2C bus
 * 
 * @param i2c I2C configuration structure
 * @param addr Device I2C address
 * @param data Data byte
 * @return bool 
 */
uint8_t i2c_read_byte(i2c_t *i2c, uint8_t slave_addr, uint8_t reg_addr, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // _I2C_DRIVER_H_