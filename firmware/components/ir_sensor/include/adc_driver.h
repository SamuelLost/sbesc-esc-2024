#ifndef _ADC_DRIVER_H_
#define _ADC_DRIVER_H_

#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct adc_t {
    adc_oneshot_chan_cfg_t adc_oneshot;
    adc_channel_t channel;
} adc_t;

/**
 * @brief Initialize the ADC driver
 * 
 * @param adc ADC configuration structure
 * @return bool 
 */
bool adc_init(adc_t *adc);

/**
 * @brief Deinitialize the ADC driver
 * 
 * @param adc ADC configuration structure
 */
void adc_deinit(adc_t *adc);

/**
 * @brief Read the ADC value
 * 
 * @param adc ADC configuration structure
 * @return uint32_t 
 */
uint32_t adc_read(adc_t *adc);

#ifdef __cplusplus
}
#endif

#endif // _ADC_DRIVER_H_