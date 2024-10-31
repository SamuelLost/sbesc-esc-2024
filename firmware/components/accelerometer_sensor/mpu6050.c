#include <stdio.h>
#include "mpu6050.h"
#include "esp_log.h"
#include "utils.h"
#include "math.h"

#define TIMEOUT 50

/******************************* PRIVATE FUNCTIONS *******************************/
static inline void _convert_to_mps2(acceleration_data_t* accel_data, uint16_t scale_factor);
static inline void _convert_to_g(acceleration_data_t* accel_data, uint16_t scale_factor);
static inline void _convert_to_dps(gyroscope_data_t* gyro_data, uint16_t scale_factor);
static inline void _mpu6050_get_lsb_sensitivity(accel_range_t accel_range, gyro_range_t gyro_range);
static inline void _complementary_filter(angles_data_t* angles_data);

/******************************* PRIVATE ATTRIBUTES *******************************/
static uint16_t _accel_sensitivity;
static uint16_t _gyro_sensitivity;

bool mpu6050_init(mpu6050_t* config) {

    if (!IS_VALID(config)) {
        return false;
    }

    if (!i2c_init(&config->i2c)) {
        return false;
    }

    uint8_t buffer[2] = { MPU6050_PWR_MGMT_1, 0x00 };

    if (!i2c_write(&config->i2c, config->addr, buffer, sizeof(buffer), TIMEOUT)) {
        return false;
    }

    buffer[0] = MPU6050_ACCEL_CONFIG;
    buffer[1] = config->accel_range;

    if (!i2c_write(&config->i2c, config->addr, buffer, sizeof(buffer), TIMEOUT)) {
        return false;
    }

    buffer[0] = MPU6050_GYRO_CONFIG;
    buffer[1] = config->gyro_range;

    if (!i2c_write(&config->i2c, config->addr, buffer, sizeof(buffer), TIMEOUT)) {
        return false;
    }

    if (!mpu6050_is_alive(config)) {
        return false;
    }

    _mpu6050_get_lsb_sensitivity(config->accel_range, config->gyro_range);

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

    uint8_t write_buffer[] = { MPU6050_ACCEL_XOUT_H };
    uint8_t read_buffer[6];

    i2c_write_read(&config->i2c, config->addr, write_buffer, sizeof(write_buffer), read_buffer, sizeof(read_buffer), TIMEOUT);

    accel_data->accel_x.raw = (read_buffer[0] << 8) | read_buffer[1];
    accel_data->accel_y.raw = (read_buffer[2] << 8) | read_buffer[3];
    accel_data->accel_z.raw = (read_buffer[4] << 8) | read_buffer[5];

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

bool mpu6050_get_gyroscope(mpu6050_t* config, gyroscope_data_t* gyro_data, unit_measurement_t unit) {

    if (!IS_VALID(config) || !IS_VALID(gyro_data)) {
        return false;
    }

    if (!mpu6050_is_alive(config)) {
        return false;
    }

    uint8_t write_buffer[] = { MPU6050_GYRO_XOUT_H };
    uint8_t read_buffer[6];

    i2c_write_read(&config->i2c, config->addr, write_buffer, sizeof(write_buffer), read_buffer, sizeof(read_buffer), TIMEOUT);

    gyro_data->gyro_x.raw = (read_buffer[0] << 8) | read_buffer[1];
    gyro_data->gyro_y.raw = (read_buffer[2] << 8) | read_buffer[3];
    gyro_data->gyro_z.raw = (read_buffer[4] << 8) | read_buffer[5];

    if (unit != GYRO_RAW) {
        _convert_to_dps(gyro_data, _gyro_sensitivity);
    }

    return true;
}

bool mpu6050_get_angles(mpu6050_t* config, angles_data_t* angles_data, float delta_time) {

    if (!IS_VALID(config) || !IS_VALID(angles_data)) {
        return false;
    }

    acceleration_data_t accel_data = {};

    if (!mpu6050_get_acceleration(config, &accel_data, ACCEL_G)) {
        return false;
    }

    if ((delta_time != -1) && (delta_time >= 0.01f && delta_time <= 0.1f)) {    
        gyroscope_data_t gyro_data = {};
        if (!mpu6050_get_gyroscope(config, &gyro_data, GYRO_DPS)) {
            return false;
        }
        angles_data->gyro.pitch = gyro_data.gyro_x.converted * delta_time;
        angles_data->gyro.roll = gyro_data.gyro_y.converted * delta_time;
    }

    angles_data->accel.pitch = atan2(-accel_data.accel_x.converted, sqrt(pow(accel_data.accel_y.converted, 2) + pow(accel_data.accel_z.converted, 2))) * RAD_TO_DEG;
    angles_data->accel.roll = atan2(accel_data.accel_y.converted, sqrt(pow(accel_data.accel_x.converted, 2) + pow(accel_data.accel_z.converted, 2))) * RAD_TO_DEG;

    if (delta_time != -1) {
        _complementary_filter(angles_data);
    }

    return true;

}

static inline void _mpu6050_get_lsb_sensitivity(accel_range_t accel_range, gyro_range_t gyro_range) {
    switch (accel_range) {
    case A2G:
        _accel_sensitivity = 16384;
        break;
    case A4G:
        _accel_sensitivity = 8192;
        break;
    case A8G:
        _accel_sensitivity = 4096;
        break;
    case A16G:
        _accel_sensitivity = 2048;
        break;
    }

    switch (gyro_range) {
    case G250DPS:
        _gyro_sensitivity = 131;
        break;
    case G500DPS:
        _gyro_sensitivity = 65.5;
        break;
    case G1000DPS:
        _gyro_sensitivity = 32.8;
        break;
    case G2000DPS:
        _gyro_sensitivity = 16.4;
        break;
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

static inline void _convert_to_dps(gyroscope_data_t* gyro_data, uint16_t gyro_sensitivity) {
    if (!IS_VALID(gyro_data) || gyro_sensitivity == 0) {
        return;
    }

    gyro_data->gyro_x.converted = (float)gyro_data->gyro_x.raw / gyro_sensitivity;
    gyro_data->gyro_y.converted = (float)gyro_data->gyro_y.raw / gyro_sensitivity;
    gyro_data->gyro_z.converted = (float)gyro_data->gyro_z.raw / gyro_sensitivity;
}

static float alpha = 0.98;
static inline void _complementary_filter(angles_data_t* angles_data) {
    if (!IS_VALID(angles_data)) {
        return;
    }

    angles_data->complementary_filter.pitch = alpha * (angles_data->complementary_filter.pitch + angles_data->gyro.pitch) + (1 - alpha) * angles_data->accel.pitch;
    angles_data->complementary_filter.roll = alpha * (angles_data->complementary_filter.roll + angles_data->gyro.roll) + (1 - alpha) * angles_data->accel.roll;
}