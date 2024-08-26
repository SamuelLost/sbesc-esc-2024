#include <stdio.h>
#include "mpu6050.h"
#include "sht30.h"
#include "lora_module.h"
#include "ir_sensor.h"

void app_main(void) {
    mpu6050_t config = {
        .i2c = {
            .i2c_port = I2C_NUM_0,
            .sda_pin = GPIO_NUM_21,
            .scl_pin = GPIO_NUM_22,
        },
        .addr = MPU6050_DEFAULT_ADDR,
    };

    mpu6050_init(&config);

    printf("MPU6050 initialized\n");

    sht30_t sht30_config = {
        .i2c = {
            .i2c_port = I2C_NUM_1,
            .sda_pin = GPIO_NUM_16,
            .scl_pin = GPIO_NUM_17,
        },
        .addr = SHT30_DEFAULT_ADDR,
    };
}
