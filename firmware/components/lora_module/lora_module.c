#include "lora_module.h"
#include "utils.h"
#include "esp_log.h"
#include <string.h>


/* ======================== PRIVATE STRUCTURES ======================== */
const char *TAG = "LORA_MODULE";
#define MAX_PAYLOAD_SIZE 232

typedef struct {
    uint16_t id; // Device ID
    uint32_t uid; // Unique ID
    lora_class_t lora_class; // Classe de operação
    lora_window_t lora_window; // Janela de recepção
    spread_factor_t spread_factor; // Fator de espalhamento
    coding_rate_t coding_rate; // Taxa de codificação
    bandwidth_t bandwidth; // Largura de banda
    uint8_t power_dbm; // Potência de transmissão
    uint8_t fw_version;  // Firmware version
    uint8_t fw_revision; // Firmware revision
    uint8_t channel; // Canal de operação
    uint32_t password; // Senha da rede
} local_device_info_t;

static local_device_info_t local_device_info;

/* ======================== PRIVATE FUNCTIONS ======================== */
static inline uint16_t _calculate_crc16(uint8_t *data, uint32_t length);
static inline bool _prepare_packet_command(uint16_t id, lora_cmd_t command, uint8_t *data, uint32_t size, lora_packet_t *packet);
static bool _read_local_device_info(lora_module_t *lora_module);
static bool _config_class(lora_module_t *lora_module);
static bool _config_bps(lora_module_t *lora_module);
static bool _set_network_id(lora_module_t *lora_module);
static void _print_device_info();
const char* _lora_class_to_string(lora_class_t lora_class);
const char* _lora_window_to_string(lora_window_t lora_window);
const char* _spread_factor_to_string(spread_factor_t spread_factor);
const char* _coding_rate_to_string(coding_rate_t coding_rate);
const char* _bandwidth_to_string(bandwidth_t bandwidth);

bool lora_module_init(lora_module_t *lora_module) {
    if (!IS_VALID(lora_module)) {
        return false;
    }

    if (!uart_init(&lora_module->uart_config)) {
        return false;
    }

    ESP_LOGI(TAG, "UART initialized");

    if (!_read_local_device_info(lora_module)) {
        return false;
    }
    
    ESP_LOGI(TAG, "Local device information read successfully");

    if (!_set_network_id(lora_module)) {
        return false;
    }

    if (!_config_class(lora_module)) {
        return false;
    }

    ESP_LOGI(TAG, "Class configured successfully");

    if (!_config_bps(lora_module)) {
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

bool lora_module_send(lora_module_t *lora_module, lora_packet_t *lora_packet) {
    if (!IS_VALID(lora_module) || !IS_VALID(lora_packet)) {
        return false;
    }

    if (lora_packet->size == 0 || lora_packet->size > MAX_PAYLOAD_SIZE) {
        return false;
    }

    if (uart_write(&lora_module->uart_config, lora_packet->buffer, lora_packet->size) != lora_packet->size) {
        return false;
    }

    return true;
}

bool lora_module_receive(lora_module_t *lora_module, lora_packet_t *lora_packet) {
    if (!IS_VALID(lora_module) || !IS_VALID(lora_packet)) {
        return false;
    }

    size_t bytes_read = uart_read(&lora_module->uart_config, lora_packet->buffer, MAX_PAYLOAD_SIZE);
    if (bytes_read == 0) {
        return false;
    }

    lora_packet->size = bytes_read;

    return true;
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
static inline bool _prepare_packet_command(uint16_t id, lora_cmd_t command, uint8_t *data, uint32_t size, lora_packet_t *packet) {
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
static bool _read_local_device_info(lora_module_t *lora_module) {
    uint8_t i = 0;
    uint8_t buffer_payload[MAX_BUFFER_SIZE] = {0};

    // Prepara o pacote para solicitar as informações do dispositivo
    buffer_payload[i++] = 0x00; // Command: Get device information
    buffer_payload[i++] = 0x00; // Payload size
    buffer_payload[i++] = 0x00; // Payload size
    
    lora_packet_t packet;
    if (!_prepare_packet_command(0x0000, CMD_LOCAL_READ, buffer_payload, i, &packet)) {
        return false;
    }
    
    // Envia o pacote para o módulo LoRa
    if (!lora_module_send(lora_module, &packet)) {
        ESP_LOGE(TAG, "Error sending packet");
        return false;
    }

    // Aguarda a resposta do módulo LoRa
    if (!lora_module_receive(lora_module, &packet)) {
        ESP_LOGE(TAG, "Error receiving packet");
        return false;
    }

    // Verifica se o comando recebido é o esperado
    if (packet.buffer[2] != CMD_LOCAL_READ) {
        ESP_LOGE("CMD_LOCAL_READ", "Invalid command received: %02X", packet.buffer[2]);
        return false;
    }

    // Preenche o device_info com os dados recebidos
    local_device_info.id = packet.buffer[0] | (packet.buffer[1] << 8);
    local_device_info.uid = (packet.buffer[5] << 24) | (packet.buffer[6] << 16) | (packet.buffer[7] << 8) | packet.buffer[8];
    local_device_info.password = packet.buffer[3] | (packet.buffer[4] << 8);
    local_device_info.fw_version = packet.buffer[17];
    local_device_info.fw_revision = packet.buffer[11];
    local_device_info.channel = packet.buffer[12];

    return true;
}

/**
 * @brief Configure the LoRa module class and window
 * 
 * @param lora_class: LoRa class
 * @param window: LoRa window
 * @return true: Configuration successful
 * @return false: Error configuring the LoRa module
 */
static bool _config_class(lora_module_t *lora_module) {
    if (lora_module->lora_class >= LORA_CLASS_NUM_MAX) {
        return false;
    }

    if (lora_module->lora_window >= LORA_WINDOW_NUM_MAX) {
        return false;
    }

    uint8_t i = 0;
    uint8_t buffer_payload[MAX_BUFFER_SIZE] = {0};

    buffer_payload[i++] = 0x00;
    buffer_payload[i++] = lora_module->lora_class;
    buffer_payload[i++] = lora_module->lora_window;
    buffer_payload[i++] = 0x00;

    lora_packet_t packet;
    if (!_prepare_packet_command(local_device_info.id, CMD_CONFIG_CLASS, buffer_payload, i, &packet)) {
        return false;
    }

    if (!lora_module_send(lora_module, &packet)) {
        ESP_LOGE(TAG, "Error sending packet");
        return false;
    }

    if (!lora_module_receive(lora_module, &packet)) {
        ESP_LOGE(TAG, "Error receiving packet");
        return false;
    }

    if (packet.buffer[2] != CMD_CONFIG_CLASS) {
        ESP_LOGE("CMD_CONFIG_CLASS", "Invalid command received: %02X", packet.buffer[2]);
        return false;
    }

    local_device_info.lora_class = packet.buffer[5]; // Atualiza a classe de operação
    local_device_info.lora_window = packet.buffer[6]; // Atualiza a janela de recepção
    
    return true;
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
static bool _config_bps(lora_module_t *lora_module) {
   if (lora_module->bandwidth >= BANDWIDTH_NUM_MAX) {
        return false;
    }

    if (lora_module->spread_factor >= SF_NUM_MAX) {
        return false;
    }

    if (lora_module->coding_rate >= CR_NUM_MAX) {
        return false;
    }

    uint8_t i = 0;
    uint8_t buffer_payload[MAX_BUFFER_SIZE] = {0};

    buffer_payload[i++] = SUBCMD_WRITE;
    buffer_payload[i++] = 0x14; // Potency (20 dBm)
    buffer_payload[i++] = lora_module->bandwidth;
    buffer_payload[i++] = lora_module->spread_factor;
    buffer_payload[i++] = lora_module->coding_rate;

    lora_packet_t packet;
    if (!_prepare_packet_command(local_device_info.id, CMD_LOCAL_WRITE, buffer_payload, i, &packet)) {
        return false;
    }

    if (!lora_module_send(lora_module, &packet)) {
        ESP_LOGE(TAG, "Error sending packet");
        return false;
    }

    if (!lora_module_receive(lora_module, &packet)) {
        ESP_LOGE(TAG, "Error receiving packet");
        return false;
    }

    if (packet.buffer[2] != CMD_LOCAL_WRITE) {
        ESP_LOGE("CMD_LOCAL_WRITE", "Invalid command received: %02X", packet.buffer[2]);
        return false;
    }

    local_device_info.power_dbm = packet.buffer[4]; // Atualiza a potência de transmissão
    local_device_info.bandwidth = packet.buffer[5]; // Atualiza a largura de banda
    local_device_info.spread_factor = packet.buffer[6]; // Atualiza o fator de espalhamento
    local_device_info.coding_rate = packet.buffer[7]; // Atualiza a taxa de codificação
    return true;

    return false;
}

/**
 * @brief Set the network ID
 * 
 * @param network_id: Network ID
 * @return true: Network ID set successfully
 * @return false: Error setting the network ID
 */
static bool _set_network_id(lora_module_t *lora_module) {
    uint8_t i = 0;
    uint8_t buffer_payload[MAX_BUFFER_SIZE] = {0};

    buffer_payload[i++] = 0x00; //3
    buffer_payload[i++] = 0x00; //4
    buffer_payload[i++] = local_device_info.uid >> 24; //5
    buffer_payload[i++] = local_device_info.uid >> 16; //6
    buffer_payload[i++] = local_device_info.uid >> 8; //7
    buffer_payload[i++] = local_device_info.uid & 0xFF; //8
    buffer_payload[i++] = 0x00; //9
    buffer_payload[i++] = 0x00; //10
    buffer_payload[i++] = 0x00; //11
    buffer_payload[i++] = 0x00; //12
    // buffer_payload[i++] = 0x00;
    // buffer_payload[i++] = 0x04;

    lora_packet_t packet;
    if (!_prepare_packet_command(lora_module->device_id, CMD_WRITE_RADIO_CONFIG, buffer_payload, i, &packet)) {
        return false;
    }

    if (!lora_module_send(lora_module, &packet)) {
        ESP_LOGE(TAG, "Error sending packet");
        return false;
    }

    if (!lora_module_receive(lora_module, &packet)) {
        ESP_LOGE(TAG, "Error receiving packet");
        return false;
    }

    if (packet.buffer[2] != CMD_WRITE_RADIO_CONFIG) {
        ESP_LOGE("CMD_WRITE_RADIO_CONFIG", "Invalid command received: %02X", packet.buffer[2]);
        return false;
    }

    return true;
}

const char* _lora_class_to_string(lora_class_t lora_class) {
    switch (lora_class) {
        case LORA_CLASS_A:
            return "A"; 
        case LORA_CLASS_C:
            return "C";
        default:
            return "UNKNOWN_CLASS";
    }
}

const char* _lora_window_to_string(lora_window_t lora_window) {
    switch (lora_window) {
        case LORA_WINDOW_5s:
            return "5s";
        case LORA_WINDOW_10s:
            return "10s";
        case LORA_WINDOW_15s:
            return "15s";
        default:
            return "UNKNOWN_WINDOW";
    }
}

const char* _spread_factor_to_string(spread_factor_t sf) {
    switch (sf) {
        case SF_7:
            return "SF_7";
        case SF_8:
            return "SF_8";
        case SF_9:
            return "SF_9";
        case SF_10:
            return "SF_10";
        case SF_11:
            return "SF_11";
        case SF_12:
            return "SF_12";
        default:
            return "UNKNOWN_SF";
    }
}

const char* _coding_rate_to_string(coding_rate_t cr) {
    switch (cr) {
        case CR_4_5:
            return "4/5";
        case CR_4_6:
            return "4/6";
        case CR_4_7:
            return "4/7";
        case CR_4_8:
            return "4/8";
        default:
            return "UNKNOWN_CODING_RATE";
    }
}

const char* _bandwidth_to_string(bandwidth_t bw) {
    switch (bw) {
        case BANDWIDTH_125KHZ:
            return "125 KHZ";
        case BANDWIDTH_250KHZ:
            return "250 KHZ";
        case BANDWIDTH_500KHZ:
            return "500 KHZ";
        default:
            return "UNKNOWN_BANDWIDTH";
    }
}


void _print_device_info() {
    printf("==========================================\n");
    printf("            DEVICE INFORMATION            \n");
    printf("==========================================\n");
    printf("ID:                 %d\n", local_device_info.id);
    printf("UID:                %lu\n", local_device_info.uid);
    printf("Password:           %lu\n", local_device_info.password);
    printf("Firmware version:   %u.%u\n", local_device_info.fw_version, local_device_info.fw_revision);
    printf("LoRa class:         %s\n", _lora_class_to_string(local_device_info.lora_class));
    printf("LoRa window:        %s\n", _lora_window_to_string(local_device_info.lora_window));
    printf("Spread factor:      %s\n", _spread_factor_to_string(local_device_info.spread_factor));
    printf("Coding rate:        %s\n", _coding_rate_to_string(local_device_info.coding_rate));
    printf("Bandwidth:          %s\n", _bandwidth_to_string(local_device_info.bandwidth));
    printf("Channel (RF):       %d\n", local_device_info.channel);
    printf("Power (dBm):        %d\n", local_device_info.power_dbm);
    printf("==========================================\n");
}

bool prepare_lora_packet(uint16_t id, lora_cmd_t command, char *data, lora_packet_t *packet) {
    // Ref: https://github.com/Radioenge/LoRaMESH-ESP8266/blob/9fbfb3df9ab354f1fa2564fff599df409a2ea366/LoRaMESH.ino#L353
    // uint8_t id_lsb = id & 0xFF;
    // uint8_t id_msb = (id >> 8) & 0xFF;
    // uint8_t command_byte = command;
    // int size = strlen(data) + 1; // +1 for the null terminator
    // char payload[size];
    // strncpy(payload, data, size);
    // uint8_t buffer[size + 5]; // ID (2 bytes) + Command (1 byte) + CRC16 (2 bytes)
    // buffer[0] = id_lsb;
    // buffer[1] = id_msb;
    // buffer[2] = command_byte;
    // int count = 0;
    // for (count = 3; count < (size + 3); count++) {
    //     buffer[count] = payload[count - 3];
    // }
    // uint16_t crc = _calculate_crc16(buffer, size + 3);
    // buffer[size + 3] = crc & 0xFF;
    // buffer[size + 4] = (crc >> 8) & 0xFF;
    // packet->size = size + 5;
    // for (count = 0; count < packet->size; count++) {
    //     packet->buffer[count] = buffer[count];
    // }

    return _prepare_packet_command(id, command, (uint8_t*)data, strlen(data)+1, packet);
}
