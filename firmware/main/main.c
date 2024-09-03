#include <stdio.h>
#include "mpu6050.h"
#include "sht30.h"
#include "lora_module.h"
#include "ir_sensor.h"
#include "esp_log.h"

#define TAG "MAIN"

typedef struct local_data_t {
    mpu6050_t mpu6050_config;
} local_data_t;

static local_data_t local_data = {};

void vTaskAccelerometer(void *pvParameters);

void app_main(void) {

    local_data.mpu6050_config = (mpu6050_t) {
        .i2c = {
            .i2c_port = I2C_NUM_0,
            .sda_pin = GPIO_NUM_21,
            .scl_pin = GPIO_NUM_22,
        },
        .addr = MPU6050_DEFAULT_ADDR,
        .accel_range = A2G,
        .gyro_range = G250DPS,
    };  

    if (!mpu6050_init(&local_data.mpu6050_config)) {
        ESP_LOGE("MAIN", "MPU6050 initialization failed\n");
    }

    ESP_LOGI("MAIN", "MPU6050 initialized\n");
    
    xTaskCreatePinnedToCore(vTaskAccelerometer, 
                            "Accelerometer Task", 
                            2048, 
                            NULL, 
                            5, 
                            NULL, 
                            APP_CPU_NUM);
}

void vTaskAccelerometer(void *pvParameters) {
    acceleration_data_t accel_data = {};

    while (true) {
        if(mpu6050_get_acceleration(&local_data.mpu6050_config, &accel_data, ACCEL_RAW)) {
            ESP_LOGI("RAW", "X: %d, Y: %d, Z: %d", accel_data.accel_x.raw, accel_data.accel_y.raw, accel_data.accel_z.raw);
        }

        if(mpu6050_get_acceleration(&local_data.mpu6050_config, &accel_data, ACCEL_G)) {
            ESP_LOGI("g-force", "X: %f, Y: %f, Z: %f", accel_data.accel_x.converted, accel_data.accel_y.converted, accel_data.accel_z.converted);
        }

        if (mpu6050_get_acceleration(&local_data.mpu6050_config, &accel_data, ACCEL_MPS2)) {
            ESP_LOGI("m/s²", "X: %f, Y: %f, Z: %f\n", accel_data.accel_x.converted, accel_data.accel_y.converted, accel_data.accel_z.converted);
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
