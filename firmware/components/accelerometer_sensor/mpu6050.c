#include <stdio.h>
#include "mpu6050.h"
#include "esp_log.h"
#include "utils.h"

#define TIMEOUT 50

/******************************* PRIVATE FUNCTIONS *******************************/
static inline void _convert_to_mps2(acceleration_data_t* accel_data, uint16_t scale_factor);
static inline void _convert_to_g(acceleration_data_t* accel_data, uint16_t scale_factor);
static inline uint16_t _mpu6050_get_lsb_sensitivity(accel_range_t accel_range);

/******************************* PRIVATE ATTRIBUTES *******************************/
static uint16_t _accel_sensitivity;

bool mpu6050_init(mpu6050_t* config) {

    if (!IS_VALID(config)) {
        return false;
    }

    if (!i2c_init(&config->i2c)) {
        return false;
    }

    if (!i2c_write(&config->i2c, config->addr, MPU6050_PWR_MGMT_1, 0x00, TIMEOUT)) {
        return false;
    }

    if (!i2c_write(&config->i2c, config->addr, MPU6050_ACCEL_CONFIG, config->accel_range, TIMEOUT)) {
        return false;
    }

    if (!i2c_write(&config->i2c, config->addr, MPU6050_GYRO_CONFIG, config->gyro_range, TIMEOUT)) {
        return false;
    }

    if (!mpu6050_is_alive(config)) {
        return false;
    }

    _accel_sensitivity = _mpu6050_get_lsb_sensitivity(config->accel_range);

    return true;
}

void mpu6050_deinit(mpu6050_t* config) {
    if (IS_VALID(config)) {
        i2c_deinit(&config->i2c);
    }
}

bool mpu6050_is_alive(mpu6050_t* config) {

    if (!IS_VALID(config)) {
        return false;
    }

    uint8_t data = i2c_read_byte(&config->i2c, config->addr, MPU6050_WHO_AM_I, TIMEOUT);

    return data == MPU6050_DEFAULT_ADDR;
}

bool mpu6050_get_acceleration(mpu6050_t* config, acceleration_data_t* accel_data, unit_measurement_t unit) {

    if (!IS_VALID(config) || !IS_VALID(accel_data)) {
        return false;
    }

    if (!mpu6050_is_alive(config)) {
        return false;
    }

    uint8_t buffer[6];

    i2c_read(&config->i2c, config->addr, MPU6050_ACCEL_XOUT_H, buffer, 6, TIMEOUT);

    accel_data->accel_x.raw = (buffer[0] << 8) | buffer[1];
    accel_data->accel_y.raw = (buffer[2] << 8) | buffer[3];
    accel_data->accel_z.raw = (buffer[4] << 8) | buffer[5];

    if (unit != ACCEL_RAW) {

        if (unit == ACCEL_MPS2) {
            _convert_to_mps2(accel_data, _accel_sensitivity);
        }
        else if (unit == ACCEL_G) {
            _convert_to_g(accel_data, _accel_sensitivity);
        }
    }

    return true;
}

static inline uint16_t _mpu6050_get_lsb_sensitivity(accel_range_t accel_range) {
    switch (accel_range) {
    case A2G:
        return 16384;
    case A4G:
        return 8192;
    case A8G:
        return 4096;
    case A16G:
        return 2048;
    default:
        return 0;
    }
}

static inline void _convert_to_mps2(acceleration_data_t* accel_data, uint16_t accel_sensitivity) {
    if (!IS_VALID(accel_data) || accel_sensitivity == 0) {
        return;
    }

    accel_data->accel_x.converted = ((float)accel_data->accel_x.raw / accel_sensitivity) * G_FORCE;
    accel_data->accel_y.converted = ((float)accel_data->accel_y.raw / accel_sensitivity) * G_FORCE;
    accel_data->accel_z.converted = ((float)accel_data->accel_z.raw / accel_sensitivity) * G_FORCE;
}

static inline void _convert_to_g(acceleration_data_t* accel_data, uint16_t accel_sensitivity) {
    if (!IS_VALID(accel_data) || accel_sensitivity == 0) {
        return;
    }

    accel_data->accel_x.converted = (float)accel_data->accel_x.raw / accel_sensitivity;
    accel_data->accel_y.converted = (float)accel_data->accel_y.raw / accel_sensitivity;
    accel_data->accel_z.converted = (float)accel_data->accel_z.raw / accel_sensitivity;
}