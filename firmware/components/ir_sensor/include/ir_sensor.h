#ifndef _IR_SENSOR_H_
#define _IR_SENSOR_H_

#include "adc_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ir_sensor_t {
    adc_t adc;
    gpio_num_t ir_pin;
    uint32_t threshold;
} ir_sensor_t;

/**
 * @brief Initialize the IR sensor
 * 
 * @param ir IR sensor configuration structure
 * @return true
 * @return false 
 */
bool ir_sensor_init(ir_sensor_t *ir);

/**
 * @brief Deinitialize the IR sensor
 * 
 * @param ir IR sensor configuration structure
 */
void ir_sensor_deinit(ir_sensor_t *ir);

/**
 * @brief Get the IR sensor value
 * 
 * @param ir IR sensor configuration structure
 * @return uint32_t 
 */
uint32_t ir_sensor_get_value(ir_sensor_t *ir);

#ifdef __cplusplus
}
#endif

#endif // _IR_SENSOR_H_