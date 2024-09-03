#include <stdio.h>
#include "i2c_driver.h"

bool i2c_init(i2c_t *i2c) {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c->sda_pin,
        .scl_io_num = i2c->scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
        .clk_flags = 0,
    };

    if (i2c_param_config(i2c->i2c_port, &i2c_config) != ESP_OK) {
        return false;
    }

    if (i2c_driver_install(i2c->i2c_port, i2c_config.mode, 0, 0, 0) != ESP_OK) {
        return false;
    }

    return true;
    
}

void i2c_deinit(i2c_t *i2c) {
    i2c_driver_delete(i2c->i2c_port);
}

bool i2c_write(i2c_t *i2c, uint8_t slave_addr, uint8_t reg_addr, uint8_t data, uint32_t timeout_ms) {
    uint8_t buffer[2] = {reg_addr, data};

    esp_err_t ret = i2c_master_write_to_device(i2c->i2c_port, slave_addr, buffer, 
        2, pdMS_TO_TICKS(timeout_ms));

    if (ret != ESP_OK) {
        return false;
    }

    return true;
}

bool i2c_read(i2c_t *i2c, uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint32_t length, uint32_t timeout_ms) {
    esp_err_t ret = i2c_master_write_read_device(i2c->i2c_port, slave_addr, &reg_addr,
        1, data, length, pdMS_TO_TICKS(timeout_ms));

    if (ret != ESP_OK) {
        return false;
    }

    return true;
}

uint8_t i2c_read_byte(i2c_t *i2c, uint8_t slave_addr, uint8_t reg_addr, uint32_t timeout_ms) {
    uint8_t data;
    esp_err_t ret = i2c_master_write_read_device(i2c->i2c_port, slave_addr, &reg_addr,
        1, &data, 1, pdMS_TO_TICKS(timeout_ms));

    if (ret != ESP_OK) {
        return 0;
    }

    return data;

}