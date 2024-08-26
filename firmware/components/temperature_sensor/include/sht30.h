#ifndef _TEMPERATURE_SENSOR_H_
#define _TEMPERATURE_SENSOR_H_

#include <stdint.h>
#include "esp_err.h"
#include "i2c_driver.h"

#define SHT30_DEFAULT_ADDR 0x00 // TODO: Change this to the actual address

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_t i2c;
    uint8_t addr; // Device I2C address
    float temperature; // (Internal) Temperature value
    float humidity;   // (Internal) Humidity value
} sht30_t;

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

#ifdef __cplusplus
}
#endif

#endif // _TEMPERATURE_SENSOR_H_