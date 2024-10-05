# Embedded Systems Competition (ESC) 2024

| Supported Targets | ESP32 |
| ----------------- | ----- |

## Team Name: Boia para Monitoramento e Prevenção de Inundações

### Team Members

- José Bastista de Sousa Júnior
- Larissa da Silva Matos
- Samuel Henrique Guimarães Alencar
- Wagner Al-Alam Guimarães (orientador)

### Module LoRa

- Default ID: 254
- Master ID: 0
- Slave ID: 1

### UART Communication

- UART1 -> TX_PIN: GPIO_NUM_4, RX_PIN: GPIO_NUM_5

```c
local_data.uart_config = (uart_t) {
        .uart_port = UART_NUM_1,
        .baud_rate = 115200,
        .tx_pin = GPIO_NUM_4,
        .rx_pin = GPIO_NUM_5,
    };
```

- UART2 -> TX_PIN: GPIO_NUM_17, RX_PIN: GPIO_NUM_16

```c
local_data.uart_config = (uart_t) {
        .uart_port = UART_NUM_2,
        .baud_rate = 115200,
        .tx_pin = GPIO_NUM_17,
        .rx_pin = GPIO_NUM_16,
    };
```
