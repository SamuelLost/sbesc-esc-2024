#include "lora_module.h"

bool lora_module_init(lora_module_t *lora_module) {
    return true;
}

void lora_module_deinit(lora_module_t *lora_module) {}

bool lora_module_send(lora_module_t *lora_module, uint8_t *data, uint32_t size) {
    return true;
}

bool lora_module_receive(lora_module_t *lora_module, uint8_t *data, uint32_t size) {
    return true;
}