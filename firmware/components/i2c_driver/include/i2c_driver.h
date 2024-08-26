#ifndef _I2C_DRIVER_H_
#define _I2C_DRIVER_H_

#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct i2c_t {
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
} i2c_t;

#ifdef __cplusplus
}
#endif

#endif // _I2C_DRIVER_H_