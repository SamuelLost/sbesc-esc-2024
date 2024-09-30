#ifndef _LORA_MODULE_H_
#define _LORA_MODULE_H_

/**
 * @file lora_module.h
 * @author Samuel Henrique (samuelhenriq12@gmail.com)
 * @brief Biblioteca para módulo LoRaMESH da Radioenge
 * @version 1.0
 * @date 16-09-2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "uart_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LoRa class enumeration
 * 
 */
typedef enum {
    LORA_CLASS_A = 0x00,
    LORA_CLASS_C = 0x02
} lora_class_t;

/**
 * @brief LoRa window enumeration
 * 
 */
typedef enum {
    WINDOW_5s = 0x00,
    WINDOW_10s = 0x01,
    WINDOW_15s = 0x02,
} lora_window_t; 

/**
 * @brief Spread factor enumeration
 * 
 */
typedef enum {
    SF_7 = 0x07,
    SF_8 = 0x08,
    SF_9 = 0x09,
    SF_10 = 0x0A,
    SF_11 = 0x0B,
    SF_12 = 0x0C
} spread_factor_t;

/**
 * @brief Coding rate enumeration
 * 
 */
typedef enum {
    CR_4_5 = 0x01,
    CR_4_6 = 0x02,
    CR_4_7 = 0x03,
    CR_4_8 = 0x04
} coding_rate_t;

/**
 * @brief Bandwidth enumeration
 * 
 */
typedef enum {
    BANDWIDTH_125KHZ = 0x00,
    BANDWIDTH_250 = 0x01,
    BANDWIDTH_500 = 0x02
} bandwidth_t;

/**
 * @brief LoRa module commands enumeration
 * @see Datasheet: https://www.radioenge.com.br/wp-content/uploads/2021/08/Manual_LoRaMESH_mar2024.pdf
 */
typedef enum {
    CMD_LOCAL_WRITE = 0xD6, // Leitura ou escrita de um parâmetro do módulo
    CMD_LOCAL_READ = 0xE2, // Leitura de um parâmetro do módulo
    CMD_REMOTE_READ = 0xD4, // Leitura de um parâmetro de um rádio escravo
    CMD_WRITE_RADIO_CONFIG = 0xCA, // Escrita dos parâmetros de um rádio
    CMD_GPIO_CONFIG = 0xC2, // Configuração, leitura e escrita de GPIO
    CMD_DIAGNOSIS = 0xE7, // Diagnóstico do módulo
    CMD_READ_NOISE = 0xD8, // Leitura do nível de ruído
    CMD_READ_RSSI = 0xD5, // Leitura do RSSI
    CMD_TRACE_ROUTE = 0xD2, // Rastreamento de rota
    CMD_SEND_TRANSPARENT = 0x28, // Envio de dados transparentes
    CMD_CONFIG_CLASS = 0xC1, // Configuração da classe de operação
    CMD_PERIODIC_TEST = 0x01, // Teste periodico enviado dos escravos para o mestre. Configurável pelo comando 0xCA
    CMD_SET_PASSWORD = 0xCD, // Configuração de senha
} lora_cmd_t;


/**
 * @brief LoRa module configuration structure
 * 
 */
typedef struct lora_module_t {
    uart_t uart_config; // Configuração da UART
    spread_factor_t spread_factor; // Fator de espalhamento
    coding_rate_t coding_rate; // Taxa de codificação
    bandwidth_t bandwidth; // Largura de banda
    lora_class_t lora_class; // Classe de operação
    lora_window_t window; // Janela de recepção
    
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