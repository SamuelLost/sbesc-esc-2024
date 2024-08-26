#include <stdio.h>
#include "sht30.h"

typedef enum sht30_command_t {
    SHT30_CMD_FETCH_DATA = 0xE000,
    // Other commands

} sht30_command_t;

bool sht30_init(sht30_t *sht30) {

    return true;
}