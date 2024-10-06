#ifndef _LASER_SENSOR_H_
#define _LASER_SENSOR_H_

/**
 * @file laser_sensor.h
 * @author Samuel Henrique (samuelhenriq12@gmail.com)
 * @brief Driver for TOF10120 laser distance sensor - UART
 * @version 1.0
 * @date 05-10-2024
 * 
 * @copyright Copyright (c) 2024 Samuel Henrique, Larissa Matos, José Batista
 * 
 */

#include "uart_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uart_t laser_sensor_t;

/**
 * @brief Initialize the laser sensor
 * 
 * @param laser laser sensor configuration structure
 * @return true
 * @return false 
 */
bool laser_sensor_init(laser_sensor_t *laser_sensor);

/**
 * @brief Deinitialize the laser sensor
 * 
 * @param laser_sensor laser sensor configuration structure
 */
void laser_sensor_deinit(laser_sensor_t *laser_sensor);

/**
 * @brief Get the laser sensor value
 * 
 * @param laser laser sensor configuration structure
 * @return uint16_t 
 */
uint16_t laser_sensor_get_value(laser_sensor_t *laser);

#ifdef __cplusplus
}
#endif

#endif // _LASER_SENSOR_H_