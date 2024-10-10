#ifndef _TEMPERATURE_SENSOR_H_
#define _TEMPERATURE_SENSOR_H_

/**
 * @file sht30.h
 * @author Samuel Henrique (samuelhenriq12@gmail.com)
 * @brief SHT30 sensor driver
 * @version 1.0
 * @date 10-10-2024
 * 
 * @copyright Copyright (c) 2024 Samuel Henrique, Larissa Matos, José Batista
 * 
 */

#include <stdint.h>
#include "esp_err.h"
#include "i2c_driver.h"

#define SHT30_DEFAULT_ADDR 0x44 // Default I2C address

#ifdef __cplusplus
extern "C" {
#endif

// SHT30 sensor modes
typedef enum {
    SHT30_SINGLE_SHOT = 0, // One single measurement
    SHT30_PERIODIC_0_5_MPS, // Periodic measurement, 0.5 measurements per second
    SHT30_PERIODIC_1_MPS,   // Periodic measurement, 1 measurement per second
    SHT30_PERIODIC_2_MPS,   // Periodic measurement, 2 measurements per second
    SHT30_PERIODIC_4_MPS,   // Periodic measurement, 4 measurements per second
    SHT30_PERIODIC_10_MPS,  // Periodic measurement, 10 measurements per second
} sht30_mode_t;

// SHT30 repeatability modes
typedef enum {
    SHT30_HIGH = 0, // High repeatability
    SHT30_MEDIUM,   // Medium repeatability
    SHT30_LOW,      // Low repeatability
} sht30_repeat_t;

// SHT30 sensor data structure
typedef struct {
    i2c_t i2c;
    uint8_t addr; // Device I2C address
    sht30_mode_t mode; // Sensor mode
    sht30_repeat_t repeat; // Repeatability mode
} sht30_t;

// SHT30 data structure
typedef struct {
    float temperature; // Temperature value
    float humidity;    // Humidity value
} sht30_data_t;


/**
 * @brief Initialize the SHT30 sensor
 * 
 * @param sht30 
 * @return true 
 * @return false 
 */
bool sht30_init(sht30_t *sht30);

/**
 * @brief Deinitialize the SHT30 sensor
 * 
 * @param sht30 
 */
void sht30_deinit(sht30_t *sht30);

bool sht30_measure(sht30_t *sht30, sht30_data_t *data);

/**
 * @brief Get the temperature value
 * 
 * @param sht30 
 * @return float 
 */
float sht30_get_temperature(sht30_t *sht30);

/**
 * @brief Get the humidity value
 * 
 * @param sht30 
 * @return float 
 */
float sht30_get_humidity(sht30_t *sht30);

/**
 * @brief Get the status of the sensor
 * 
 * @param sht30
 * @return true
 * @return false
 */
bool sht30_get_status(sht30_t *sht30, uint16_t *status);

#ifdef __cplusplus
}
#endif

#endif // _TEMPERATURE_SENSOR_H_