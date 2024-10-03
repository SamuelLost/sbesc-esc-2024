#include "lora_module.h"
#include "utils.h"
#include "esp_log.h"
#include <string.h>

#define MAX_PAYLOAD_SIZE 232
#define MAX_BUFFER_SIZE 237
const char *TAG = "LORA_MODULE";

typedef struct {
    uint8_t buffer[MAX_BUFFER_SIZE];
    uint32_t size;
} lora_packet_t;

typedef struct {
    uint16_t id;
    uint32_t uid;
    lora_class_t lora_class;
    lora_window_t lora_window;
    spread_factor_t spread_factor; // Fator de espalhamento
    coding_rate_t coding_rate; // Taxa de codificação
    bandwidth_t bandwidth; // Largura de banda
    uint8_t power_dbm; // Potência de transmissão
    uint8_t fw_version;  // Firmware version
    uint8_t fw_revision; // Firmware revision
    uint32_t password;
} local_device_info_t;

static local_device_info_t local_device_info;

static inline uint16_t _calculate_crc16(uint8_t *data, uint32_t length);
static inline bool _prepare_packet(uint16_t id, lora_cmd_t command, uint8_t *data, uint32_t size, lora_packet_t *packet);
static bool _read_local_device_info(lora_module_t *lora_module, local_device_info_t *device_info);
static bool _config_class(lora_module_t *lora_module, lora_class_t lora_class, lora_window_t lora_window);
static bool _config_bps(lora_module_t *lora_module, bandwidth_t bandwidth, spread_factor_t spread_factor, coding_rate_t coding_rate);
static void _print_device_info(local_device_info_t *device_info);
bool lora_module_init(lora_module_t *lora_module) {
    if (!IS_VALID(lora_module)) {
        return false;
    }

    if (!uart_init(&lora_module->uart_config)) {
        return false;
    }

    ESP_LOGI(TAG, "UART initialized");

    if (!_read_local_device_info(lora_module, &local_device_info)) {
        return false;
    }
    
    ESP_LOGI(TAG, "Local device information read successfully");

    if (!_config_class(lora_module, lora_module->lora_class, lora_module->lora_window)) {
        return false;
    }

    ESP_LOGI(TAG, "Class configured successfully");

    if (!_config_bps(lora_module, lora_module->bandwidth, lora_module->spread_factor, lora_module->coding_rate)) {
        return false;
    }

    ESP_LOGI(TAG, "Bandwidth, spread factor and coding rate configured successfully");
    
    _print_device_info(&local_device_info);

    return true;
}

void lora_module_deinit(lora_module_t *lora_module) {
    if (!IS_VALID(lora_module)) {
        return;
    }

    uart_deinit(&lora_module->uart_config); 
}

bool lora_module_send(lora_module_t *lora_module, uint8_t *data, uint32_t size) {
    if (!IS_VALID(lora_module) || !IS_VALID(data)) {
        return false;
    }

    if (size == 0 || size > MAX_PAYLOAD_SIZE) {
        return false;
    }

    if (uart_write(&lora_module->uart_config, data, size) != size) {
        return false;
    }

    return true;
}

bool lora_module_receive(lora_module_t *lora_module, uint8_t *data, uint32_t size) {
    if (!IS_VALID(lora_module) || !IS_VALID(data)) {
        return false;
    }

    if (size == 0 || size > MAX_PAYLOAD_SIZE) {
        return false;
    }

    size_t bytes_read = uart_read(&lora_module->uart_config, data, size);
    // packet.size = bytes_read;
    // printf("Bytes read: %d\n", bytes_read);
    // for (int i = 0; i < bytes_read; i++) {
    //     printf("0x%02X ", data[i]);
    // }
    // printf("\n");

    return true;
}

void _print_device_info(local_device_info_t *device_info) {
    printf("==========================================\n");
    printf("            DEVICE INFORMATION            \n");
    printf("==========================================\n");
    printf("ID:                 %d\n", device_info->id);
    printf("UID:                %lu\n", device_info->uid);
    printf("Password:           %lu\n", device_info->password);
    printf("Firmware version:   %u.%u\n", device_info->fw_version, device_info->fw_revision);
    printf("LoRa class:         %d\n", device_info->lora_class);
    printf("LoRa window:        %d\n", device_info->lora_window);
    printf("Spread factor:      %d\n", device_info->spread_factor);
    printf("Coding rate:        %d\n", device_info->coding_rate);
    printf("Bandwidth:          %d\n", device_info->bandwidth);
    printf("Power (dBm):        %d\n", device_info->power_dbm);
    printf("==========================================\n");
}
/**
 * @brief Calculate the CRC16 of a data buffer
 * 
 * @param data: Pointer to the data buffer
 * @param length: Length of the data buffer
 * @return uint16_t 16-bit value of the CRC16 of the data buffer
 */
#define CRC_POLY 0xA001
static inline uint16_t _calculate_crc16(uint8_t *data, uint32_t length) {
    uint32_t i;
    uint8_t bitbang, j;
    uint16_t crc_calc = 0xC181;

    for (i = 0; i < length; i++) {
        crc_calc ^= ((uint16_t)data[i]) & 0x00FF;

        for (j = 0; j < 8; j++) {
            bitbang = crc_calc;
            crc_calc >>= 1;

            if (bitbang & 1) {
                crc_calc ^= CRC_POLY;
            }
        }
    }

    return crc_calc;
}

/**
 * @brief Prepare a packet to be sent to the LoRa module
 * 
 * @param id: Packet ID
 * @param command: Command to be sent
 * @param data: Data buffer
 * @param size: Data size
 * @param packet: Packet structure
 * @return true: Packet prepared successfully
 * @return false: Error preparing the packet
 */
static inline bool _prepare_packet(uint16_t id, lora_cmd_t command, uint8_t *data, uint32_t size, lora_packet_t *packet) {
    // Verifica se o tamanho do payload é válido
    if (size == 0 || size > MAX_PAYLOAD_SIZE) {
        return false;
    }

    // Verifica se os dados são válidos se o tamanho for maior que 0
    if (!IS_VALID(data) && size > 0) {
        return false;
    }

    packet->size = size + 5; // ID (2 bytes) + Command (1 byte) + CRC16 (2 bytes)

    packet->buffer[0] = id & 0xFF; // Bits 0-7 (LSB)
    packet->buffer[1] = (id >> 8) & 0xFF; // Bits 8-15 (MSB)
    packet->buffer[2] = command; // Command

    if (size > 0) {
        memcpy(&packet->buffer[3], data, size); // Payload
    }

    // Calcula o CRC16 sobre o ID, comando e payload
    uint16_t crc = _calculate_crc16(packet->buffer, size + 3);
    packet->buffer[size + 3] = crc & 0xFF; // Bits 0-7 (LSB)
    packet->buffer[size + 4] = (crc >> 8) & 0xFF; // Bits 8-15 (MSB)

    return true;
}

/**
 * @brief Read the local device information
 * 
 * @param lora_module: LoRa module configuration structure
 * @param data: Data buffer
 * @param size: Data size
 * @return true: Device information read successfully
 * @return false: Error reading the device information
 */
static bool _read_local_device_info(lora_module_t *lora_module, local_device_info_t *device_info) {
    uint8_t i = 0;
    uint8_t buffer_payload[MAX_BUFFER_SIZE] = {0};

    // Prepara o pacote para solicitar as informações do dispositivo
    buffer_payload[i++] = 0x00; // Command: Get device information
    buffer_payload[i++] = 0x00; // Payload size
    buffer_payload[i++] = 0x00; // Payload size
    
    lora_packet_t packet;
    if (!_prepare_packet(0x0000, CMD_LOCAL_READ, buffer_payload, i, &packet)) {
        return false;
    }
    
    // Envia o pacote para o módulo LoRa
    if (!lora_module_send(lora_module, packet.buffer, packet.size)) {
        printf("Error sending packet\n");
        return false;
    }

    // Aguarda a resposta do módulo LoRa
    if (lora_module_receive(lora_module, packet.buffer, 31)) { // CMD_LOCAL_READ: 31 bytes of data
        // Verifica se o comando recebido é o esperado
        if (packet.buffer[2] == CMD_LOCAL_READ) {
            device_info->id = packet.buffer[0] | (packet.buffer[1] << 8);
            /**
            * |   5       |  6  |  7  |     8
            * | UID (MSB) | UID | UID | UID (LSB)
            *
            */ 
            device_info->uid = packet.buffer[5] | (packet.buffer[6] << 8) | (packet.buffer[7] << 16) | (packet.buffer[8] << 24);
            device_info->password = packet.buffer[3] | (packet.buffer[4] << 8);
            device_info->fw_version = packet.buffer[17];
            device_info->fw_revision = packet.buffer[11];

            return true;
        }
    } else {
        printf("Error receiving packet\n");
    }

    return false;
}

/**
 * @brief Configure the LoRa module class and window
 * 
 * @param lora_class: LoRa class
 * @param window: LoRa window
 * @return true: Configuration successful
 * @return false: Error configuring the LoRa module
 */
static bool _config_class(lora_module_t *lora_module, lora_class_t lora_class, lora_window_t lora_window) {
    if (lora_class >= LORA_CLASS_NUM_MAX) {
        return false;
    }

    if (lora_window >= LORA_WINDOW_NUM_MAX) {
        return false;
    }

    uint8_t i = 0;
    uint8_t buffer_payload[MAX_BUFFER_SIZE] = {0};

    buffer_payload[i++] = 0x00;
    buffer_payload[i++] = lora_class;
    buffer_payload[i++] = lora_window;
    buffer_payload[i++] = 0x00;

    lora_packet_t packet;
    if (!_prepare_packet(local_device_info.id, CMD_CONFIG_CLASS, buffer_payload, i, &packet)) {
        return false;
    }

    if (!lora_module_send(lora_module, packet.buffer, packet.size)) {
        return false;
    }

    packet.size = 9;
    if (lora_module_receive(lora_module, packet.buffer, packet.size)) { // CMD_CONFIG_CLASS: 9 bytes of data
        if (packet.buffer[2] == CMD_CONFIG_CLASS) {
            local_device_info.lora_class = packet.buffer[5]; // Atualiza a classe de operação
            local_device_info.lora_window = packet.buffer[6]; // Atualiza a janela de recepção
            return true;
        }
    }

    return false;
}

/**
 * @brief Configure the LoRa module bandwidth, spread factor and coding rate
 * 
 * @param bandwidth: LoRa module bandwidth
 * @param spread_factor: LoRa module spread factor
 * @param coding_rate: LoRa module coding rate
 * @return true: Configuration successful
 * @return false: Error configuring the LoRa module
 */
static bool _config_bps(lora_module_t *lora_module, bandwidth_t bandwidth, spread_factor_t spread_factor, coding_rate_t coding_rate) {
   if (bandwidth >= BANDWIDTH_NUM_MAX) {
        return false;
    }

    if (spread_factor >= SF_NUM_MAX) {
        return false;
    }

    if (coding_rate >= CR_NUM_MAX) {
        return false;
    }

    uint8_t i = 0;
    uint8_t buffer_payload[MAX_BUFFER_SIZE] = {0};

    buffer_payload[i++] = SUBCMD_WRITE;
    buffer_payload[i++] = 0x14; // Potency (20 dBm)
    buffer_payload[i++] = bandwidth;
    buffer_payload[i++] = spread_factor;
    buffer_payload[i++] = coding_rate;

    lora_packet_t packet;
    if (!_prepare_packet(local_device_info.id, CMD_LOCAL_WRITE, buffer_payload, i, &packet)) {
        return false;
    }

    if (!lora_module_send(lora_module, packet.buffer, packet.size)) {
        return false;
    }

    packet.size = 10; // CMD_LOCAL_WRITE: 10 bytes of data
    if (lora_module_receive(lora_module, packet.buffer, packet.size)) {
        if (packet.buffer[2] == CMD_LOCAL_WRITE) {
            local_device_info.power_dbm = packet.buffer[4]; // Atualiza a potência de transmissão
            local_device_info.bandwidth = packet.buffer[5]; // Atualiza a largura de banda
            local_device_info.spread_factor = packet.buffer[6]; // Atualiza o fator de espalhamento
            local_device_info.coding_rate = packet.buffer[7]; // Atualiza a taxa de codificação
            return true;
        }
    }

    return false;
}