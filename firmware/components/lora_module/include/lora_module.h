#ifndef _LORA_MODULE_H_
#define _LORA_MODULE_H_

#include "uart_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct lora_module_t {
    uart_t uart;
} lora_module_t;

/**
 * @brief Initialize the LoRa module
 * 
 * @param lora LoRa module configuration structure
 * @return true
 * @return false 
 */
bool lora_module_init(lora_module_t *lora);

/**
 * @brief Deinitialize the LoRa module
 * 
 * @param lora LoRa module configuration structure
 */
void lora_module_deinit(lora_module_t *lora);

/**
 * @brief Send data to the LoRa module
 * 
 * @param lora LoRa module configuration structure
 * @param data Data buffer
 * @param size Data size
 * @return true
 * @return false 
 */
bool lora_module_send(lora_module_t *lora, uint8_t *data, uint32_t size);

/**
 * @brief Receive data from the LoRa module
 * 
 * @param lora LoRa module configuration structure
 * @param data Data buffer
 * @param size Data size
 * @return true
 * @return false 
 */
bool lora_module_receive(lora_module_t *lora, uint8_t *data, uint32_t size);

#ifdef __cplusplus
}
#endif

#endif // _LORA_MODULE_H_