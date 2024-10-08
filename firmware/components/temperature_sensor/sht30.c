#include <stdio.h>
#include "sht30.h"

#define TIMEOUT 50

typedef enum {
    SHT30_FETCH_DATA_CMD = 0xE000,
    SHT30_RESET_CMD = 0x30A2,
    STH30_STATUS_CMD = 0xF32D,
    // Other commands

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

static bool _sht30_reset(sht30_t *sht30);
static uint8_t _sht30_crc8(uint8_t *data, uint8_t len);
static bool _sht30_read(sht30_t *sht30, uint8_t *data, uint32_t len);
static bool _sht30_write(sht30_t *sht30, sht30_command_t sht30_cmd);

bool sht30_init(sht30_t *sht30) {
    if (!i2c_init(&sht30->i2c)) {
        return false;
    }

    return true;
}

void sht30_deinit(sht30_t *sht30) {
    i2c_deinit(&sht30->i2c);
}

float sht30_get_temperature(sht30_t *sht30) {
    uint8_t data[6] = {0};
    uint16_t temperature = 0;

    if (!i2c_write(&sht30->i2c, sht30->addr, SHT30_FETCH_DATA_CMD, 0, TIMEOUT)) {
        return 0;
    }

    if (!i2c_read(&sht30->i2c, sht30->addr, SHT30_FETCH_DATA_CMD, data, 6, TIMEOUT)) {
        return 0;
    }

    temperature = (data[0] << 8) | data[1];

    return -45 + 175 * (temperature / 65535.0);
}