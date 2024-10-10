#include <stdio.h>
#include "sht30.h"
#include "esp_log.h"
#include "utils.h"

/* ======================== PRIVATE STRUCTURES ======================== */
#define TIMEOUT 50
const char *TAG_SHT30 = "SHT30";
typedef enum {
    SHT30_FETCH_DATA_CMD = 0xE000,
    SHT30_RESET_CMD = 0x30A2,
    STH30_STATUS_CMD = 0xF32D,
} sht30_command_t;

// SHT30 measurement commands based on the sensor mode without clock stretching
const uint16_t SHT30_MEASURE_CMD[6][3] = {
    {0x2400,0x240b,0x2416},    // [SHT30_SINGLE_SHOT     ][HIGH, MEDIUM, LOW] 
    {0x2032,0x2024,0x202f},    // [SHT30_PERIODIC_0_5_MPS][HIGH, MEDIUM, LOW]
    {0x2130,0x2126,0x212d},    // [SHT30_PERIODIC_1_MPS  ][HIGH, MEDIUM, LOW]
    {0x2236,0x2220,0x222b},    // [SHT30_PERIODIC_2_MPS  ][HIGH, MEDIUM, LOW]
    {0x2234,0x2322,0x2329},    // [SHT30_PERIODIC_4_MPS  ][HIGH, MEDIUM, LOW]
    {0x2737,0x2721,0x272a}     // [SHT30_PERIODIC_10_MPS ][HIGH, MEDIUM, LOW]
};

// SHT30 measurement duration (ms) based on the sensor mode
const uint8_t SHT30_MEASURE_DURATION_MS[3] = {15, 6, 4};

#define SHT30_RAW_DATA_SIZE 6
typedef uint8_t sht30_raw_data_t[SHT30_RAW_DATA_SIZE];

/******************************* PRIVATE FUNCTIONS *******************************/
static bool _sht30_reset(sht30_t *sht30);
static uint8_t _sht30_crc8(uint8_t *data, uint8_t len);
static bool _sht30_read(sht30_t *sht30, uint8_t *data, uint32_t len);
static bool _sht30_send_command(sht30_t *sht30, sht30_command_t sht30_cmd);

bool sht30_init(sht30_t *sht30) {
    if (!IS_VALID(sht30)) {
        return false;
    }

    if (sht30->i2c.i2c_freq_t != I2C_FREQ_100KH) {
        ESP_LOGE(TAG_SHT30, "I2C frequency must be 100 kHz");
        return false;
    }

    if (!i2c_init(&sht30->i2c)) {
        return false;
    }

    if (!_sht30_reset(sht30)) {
        ESP_LOGE(TAG_SHT30, "SHT30 reset failed");
        return false;
    }

    uint16_t status = 0;
    if (!sht30_get_status(sht30, &status)) {
        ESP_LOGE(TAG_SHT30, "SHT30 status failed");
        return false;
    }

    return true;
}

void sht30_deinit(sht30_t *sht30) {
    i2c_deinit(&sht30->i2c);
}

static bool _sht30_reset(sht30_t *sht30) {
    if (!IS_VALID(sht30)) {
        return false;
    }

    if (!_sht30_send_command(sht30, SHT30_RESET_CMD)) {
        return false;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);

    return true;
}

#define CRC_POLY 0x31
static uint8_t _sht30_crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;

    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            bool xor = crc & 0x80;
            crc <<= 1;
            if (xor) {
                crc ^= CRC_POLY;
            }
        }
    }

    return crc;
}

bool sht30_get_status(sht30_t *sht30, uint16_t *status) {
    if (!IS_VALID(sht30) || !IS_VALID(status)) {
        return false;
    }

    if (!_sht30_send_command(sht30, STH30_STATUS_CMD)) {
        return false;
    }

    uint8_t data[3] = {0};
    if (!_sht30_read(sht30, data, sizeof(data))) {
        return false;
    }

    *status = (data[0] << 8) | data[1];

    return true;
}

static bool _sht30_read(sht30_t *sht30, uint8_t *data, uint32_t len) {
    if (!IS_VALID(sht30) || !IS_VALID(data)) {
        return false;
    }

    if (!i2c_read(&sht30->i2c, sht30->addr, data, len, TIMEOUT)) {
        return false;
    }

    return true;
}

static bool _sht30_send_command(sht30_t *sht30, sht30_command_t sht30_cmd) {
    if (!IS_VALID(sht30)) {
        return false;
    }

    uint8_t data[2] = {sht30_cmd >> 8, sht30_cmd & 0xFF};

    if (!i2c_write(&sht30->i2c, sht30->addr, data, 2, TIMEOUT)) {
        ESP_LOGE(TAG_SHT30, "Error sending command");
        return false;
    }

    return true;
}

bool sht30_measure(sht30_t *sht30, sht30_data_t *data) {
    if (!IS_VALID(sht30) || !IS_VALID(data)) {
        return false;
    }

    if (!_sht30_send_command(sht30, SHT30_MEASURE_CMD[sht30->mode][sht30->repeat])) {
        return false;
    }

    // If the I2C frequency is 400 kHz, uncomment the following line
    // vTaskDelay(SHT30_MEASURE_DURATION_MS[sht30->repeat]+1 / portTICK_PERIOD_MS);
    
    sht30_raw_data_t raw_data = {0};

    if (!_sht30_read(sht30, raw_data, sizeof(raw_data))) {
        ESP_LOGE(TAG_SHT30, "Error reading data");
        return false;
    }

    if (_sht30_crc8(raw_data, 2) != raw_data[2]) {
        ESP_LOGE(TAG_SHT30, "CRC8 error on temperature");
        return false;
    }

    if (_sht30_crc8(raw_data + 3, 2) != raw_data[5]) {
        ESP_LOGE(TAG_SHT30, "CRC8 error on humidity");
        return false;
    }

    uint16_t temperature = (raw_data[0] << 8) | raw_data[1];
    uint16_t humidity = (raw_data[3] << 8) | raw_data[4];

    data->temperature = -45 + 175 * (temperature / 65535.0);
    data->humidity = 100 * (humidity / 65535.0);

    return true;

}