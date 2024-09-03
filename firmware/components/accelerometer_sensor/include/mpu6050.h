#ifndef _ACCELEROMETER_SENSOR_H_
#define _ACCELEROMETER_SENSOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"
#include "i2c_driver.h"
#include "mpu6050_registers.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef union {
    int16_t raw;
    float converted;
} accel_axis_t;

// Accelerometer sensor data structure
typedef struct {
    accel_axis_t accel_x;
    accel_axis_t accel_y;
    accel_axis_t accel_z;
} acceleration_data_t;

// Gyroscope sensor data structure
typedef struct {
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} gyroscope_data_t;

typedef enum {
    A2G, // +/- 2g
    A4G, // +/- 4g: 
    A8G, // +/- 8g
    A16G // +/- 16g
} accel_range_t;

typedef enum {
    G250DPS, // +/- 250 degrees per second
    G500DPS, // +/- 500 degrees per second
    G1000DPS, // +/- 1000 degrees per second
    G2000DPS // +/- 2000 degrees per second
} gyro_range_t;

typedef enum {
    ACCEL_RAW, // raw data
    ACCEL_MPS2, // meters per second squared
    ACCEL_G // g-force
} unit_measurement_t;

// MPU6050 configuration structure
typedef struct {
    i2c_t i2c;
    uint8_t addr;
    accel_range_t accel_range;
    gyro_range_t gyro_range;
} mpu6050_t;

/**
 * @brief Initialize the MPU6050 sensor
 * 
 * @param config mpu6050_t configuration structure
 * @return bool 
 */
bool mpu6050_init(mpu6050_t *config);

/**
 * @brief Deinitialize the MPU6050 sensor
 * 
 * @param config mpu6050_t configuration structure
 */
void mpu6050_deinit(mpu6050_t *config);

/**
 * @brief Check if the MPU6050 sensor is alive
 * 
 * @param config mpu6050_t configuration structure
 * @return bool 
 */
bool mpu6050_is_alive(mpu6050_t *config);

/**
 * @brief Get the acceleration data
 * 
 * @param accel_data
 * @return bool 
 */
bool mpu6050_get_acceleration(mpu6050_t *config, acceleration_data_t *accel_data, unit_measurement_t unit);

/**
 * @brief Get the gyroscope data
 * 
 * @param gyro_data 
 * @return bool 
 */
bool mpu6050_get_gyroscope(mpu6050_t *config, gyroscope_data_t *gyro_data);


#ifdef __cplusplus
}
#endif

#endif // _ACCELEROMETER_SENSOR_H_