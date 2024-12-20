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

#define MAX_BUFFER_SIZE 237

typedef struct {
    uint8_t buffer[MAX_BUFFER_SIZE];
    uint32_t size;
} lora_packet_t;

/**
 * @brief LoRa class enumeration
 * 
 */
typedef enum {
    LORA_CLASS_A = 0x00,
    LORA_CLASS_C = 0x02,
    LORA_CLASS_NUM_MAX // Número máximo de classes
} lora_class_t;

/**
 * @brief LoRa window enumeration
 * 
 */
typedef enum {
    LORA_WINDOW_5s = 0x00,
    LORA_WINDOW_10s = 0x01,
    LORA_WINDOW_15s = 0x02,
    LORA_WINDOW_NUM_MAX // Número máximo de janelas
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
    SF_12 = 0x0C,
    SF_NUM_MAX // Número máximo de fatores de espalhamento
} spread_factor_t;

/**
 * @brief Coding rate enumeration
 * 
 */
typedef enum {
    CR_4_5 = 0x01,
    CR_4_6 = 0x02,
    CR_4_7 = 0x03,
    CR_4_8 = 0x04,
    CR_NUM_MAX // Número máximo de taxas de codificação
} coding_rate_t;

/**
 * @brief Bandwidth enumeration
 * 
 */
typedef enum bandwidth_t {
    BANDWIDTH_125KHZ = 0x00,
    BANDWIDTH_250KHZ = 0x01,
    BANDWIDTH_500KHZ = 0x02,
    BANDWIDTH_NUM_MAX // Número máximo de larguras de banda
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
    SUBCMD_WRITE = 0x01, // Subcomando de escrita
    SUBCMD_READ = 0x00, // Subcomando de leitura
    CMD_APPLICATION = 0x10, // Comando de aplicação (envio de dados)
    CMD_ACCELEROMETER = 0x11, // Comando de acelerômetro
    CMD_TEMPERATURE = 0x12, // Comando de temperatura
    CMD_LASER = 0x13, // Comando de sensor de distância laser
    CMD_HEARTBEAT = 0x14, // Comando de verificação do funcionamento da rede
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
    lora_window_t lora_window; // Janela de recepção
    uint8_t device_id; // ID do dispositivo
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
bool lora_module_send(lora_module_t *lora, lora_packet_t *lora_packet);

/**
 * @brief Receive data from the LoRa module
 * 
 * @param lora LoRa module configuration structure
 * @param data Data buffer
 * @param size Data size
 * @return true
 * @return false 
 */
bool lora_module_receive(lora_module_t *lora, lora_packet_t *lora_packet);

/**
 * @brief Prepare a LoRa packet
 * 
 * @param id the device ID to be sent
 * @param command the command to be sent
 * @param data the data to be sent
 * @param packet the packet structure
 * @return true 
 * @return false 
 */
bool prepare_lora_packet(uint16_t id, lora_cmd_t command, char *data, lora_packet_t *packet);

const char* get_message_from_lora_packet(lora_packet_t *packet);

#ifdef __cplusplus
}
#endif

#endif // _LORA_MODULE_H_