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

## Tips FreeRTOS

- How to see space available in the queue?

```c
QueueHandle_t queue_lora_packets = xQueueCreate(10, sizeof(packet_t));

ESP_LOGI(TAG, "Space free: %d", uxQueueSpacesAvailable(queue_lora_packets));
```

- How to see the number of elements in the queue?

```c
QueueHandle_t queue_lora_packets = xQueueCreate(10, sizeof(packet_t));

ESP_LOGI(TAG, "Number of elements: %d", uxQueueMessagesWaiting(queue_lora_packets));
```

- How to see the space available in the stack?

```c
ESP_LOGI(TAG, "Space free: %d", uxTaskGetStackHighWaterMark(NULL));
```

## Sugestões de tarefas

Para organizar as tasks do seu código em uma ESP32 de maneira eficiente, aproveitando os dois núcleos, o objetivo é balancear a carga de processamento e garantir que tarefas críticas como a comunicação via LoRa e a coleta de dados dos sensores não sejam afetadas por sobrecarga. Abaixo está uma sugestão de organização:

### 1. **Core 0 (Pro CPU) – Tarefas de Sistema e Comunicação**
   
Este núcleo deve lidar com tarefas mais críticas e aquelas que exigem menor latência, como a comunicação via LoRa e o monitoramento do sistema, uma vez que o Wi-Fi e o Bluetooth também rodam nesse núcleo.

- **`vTaskLora`**: Esta task é responsável por enviar pacotes via LoRa. Como a comunicação é crítica e precisa de acesso ao hardware, deve rodar no **Core 0**. Além disso, como utiliza um mutex (para proteger o acesso ao módulo LoRa), é uma boa prática mantê-la neste núcleo para evitar conflitos com tarefas mais pesadas.
  
- **`vTaskHeartbeat`**: Esta task monitora a saúde do sistema e o uso da memória, além de enviar status via LoRa. Como é uma task de baixa prioridade e precisa de acesso a outras tasks e à fila de LoRa, ela também pode rodar no **Core 0**, pois não demanda muito processamento.

### 2. **Core 1 (App CPU) – Tarefas de Processamento de Sensores**

No **Core 1**, você pode colocar as tasks que fazem a leitura dos sensores e que não têm uma demanda crítica de tempo real, mas requerem processamento mais intensivo. Estas tasks fazem leituras periódicas e podem ser executadas de forma paralela às tarefas críticas de comunicação.

- **`vTaskAccelerometer`**: Como o acelerômetro tem leituras frequentes (a cada 4 segundos) e envolve alguma formatação de dados antes de enviá-los via LoRa, é melhor deixá-la no **Core 1** para evitar interferência nas tasks críticas de comunicação.

- **`vTaskLaserSensor`**: Esta task também faz leituras frequentes (a cada 3 segundos) e, como envolve cálculos (diferença entre distâncias) e formatação de pacotes, deve rodar no **Core 1** para liberar o **Core 0** de processamento extra.

- **`vTaskTemperature`**: Como a temperatura e a umidade são medidas com um intervalo maior (10 segundos), e a task não é tão intensiva em processamento, pode rodar no **Core 1**, junto com as outras tasks de sensor.

### 3. **Resumo da Alocação por Núcleo**

#### **Core 0 (Pro CPU)**:

- **`vTaskLora`**: Comunicação LoRa (alta prioridade).
- **`vTaskHeartbeat`**: Monitoramento de sistema (baixa prioridade, não crítica).

#### **Core 1 (App CPU)**:

- **`vTaskAccelerometer`**: Leitura do acelerômetro e envio via LoRa.
- **`vTaskLaserSensor`**: Leitura do sensor de distância e envio via LoRa.
- **`vTaskTemperature`**: Leitura da temperatura e umidade e envio via LoRa.

### 4. **Prioridades das Tasks**

Você pode ajustar as prioridades das tasks para garantir que as mais críticas, como a comunicação via LoRa, tenham maior prioridade de execução:

- **`vTaskLora`**: Alta prioridade (por exemplo, `configMAX_PRIORITIES - 1`).
- **`vTaskHeartbeat`**: Baixa prioridade (pode ser `tskIDLE_PRIORITY + 1`).
- **Tasks de Sensores**: Prioridade média (pode ser entre `tskIDLE_PRIORITY + 2` e `tskIDLE_PRIORITY + 3`).

### 5. **Exemplo de Criação das Tasks**

Aqui está um exemplo de como você poderia criar essas tasks com a função `xTaskCreatePinnedToCore()` para fixá-las no núcleo correto:

```c
xTaskCreatePinnedToCore(vTaskLora, "LoRa Task", 2048, NULL, configMAX_PRIORITIES - 1, &lora_handle, 0);    // Core 0
xTaskCreatePinnedToCore(vTaskHeartbeat, "Heartbeat Task", 2048, NULL, tskIDLE_PRIORITY + 1, &heartbeat_handle, 0);    // Core 0

xTaskCreatePinnedToCore(vTaskAccelerometer, "Accelerometer Task", 2048, NULL, tskIDLE_PRIORITY + 3, &accelerometer_handle, 1);    // Core 1
xTaskCreatePinnedToCore(vTaskLaserSensor, "Laser Task", 2048, NULL, tskIDLE_PRIORITY + 2, &laser_handle, 1);    // Core 1
xTaskCreatePinnedToCore(vTaskTemperature, "Temperature Task", 2048, NULL, tskIDLE_PRIORITY + 2, &temperature_handle, 1);    // Core 1
```

### Conclusão

Organizar as tasks dessa maneira otimiza o uso dos dois núcleos da ESP32, separando tarefas de comunicação crítica no **Core 0** e tarefas de processamento de sensores no **Core 1**, garantindo um sistema mais eficiente e responsivo.

### Acelerômetro

Para calcular os ângulos de *roll* e *pitch* usando os dados do acelerômetro e do giroscópio do MPU6050, o ideal é combinar as leituras de ambos para obter uma estimativa mais precisa e estável. Abaixo, vou detalhar o método mais comum para isso, usando a *fusão sensorial* (*sensor fusion*), que combina a resposta rápida do giroscópio com a estabilidade do acelerômetro.

### 1. Obtenha os Dados do Acelerômetro e Giroscópio

Para cada eixo:

- Acelerômetro: `Ax`, `Ay`, `Az`
- Giroscópio: `Gx`, `Gy`, `Gz`

As unidades dos dados devem ser:

- Acelerômetro em *g* (gravidade, que pode ser lida diretamente como $m/s^2$ em alguns casos).
- Giroscópio em °/s (graus por segundo).

### 2. Cálculo de Roll e Pitch pelo Acelerômetro

Os ângulos calculados apenas pelo acelerômetro são mais estáveis em baixas frequências (evitam drift), mas são sensíveis a movimentos rápidos e vibrações. A partir dos dados do acelerômetro:

1. **Pitch** ($\theta$):
   $$
   \theta = \arctan\left(\frac{-Ax}{\sqrt{Ay^2 + Az^2}}\right) \cdot \frac{180}{\pi}
   $$

2. **Roll** ($\phi$):
   $$
   \phi = \arctan\left(\frac{Ay}{\sqrt{Ax^2 + Az^2}}\right) \cdot \frac{180}{\pi}
   $$

Essas fórmulas assumem que o sensor está em um ambiente onde a gravidade é a única aceleração significativa (sem movimentos bruscos ou vibrações).

### 3. Integração dos Dados do Giroscópio

Os dados do giroscópio medem a taxa de rotação. Para calcular o ângulo a partir dessa taxa, é necessário integrar o valor ao longo do tempo:

1. **Roll e Pitch a partir do giroscópio**:
   - A cada intervalo de tempo \($dt$\), faça:
     - **Pitch**: $\theta_{gyro} += Gy \times dt$
     - **Roll**: $\phi_{gyro} += Gx \times dt$

Esses valores vão acumular drift ao longo do tempo, por isso usamos a fusão sensorial para corrigir esse desvio.

### 4. Fusão Sensorial (Complementary Filter)

A fusão sensorial combina os ângulos do acelerômetro e do giroscópio para obter uma estimativa mais precisa:

1. Escolha uma constante de filtro, geralmente entre 0,9 e 0,98 (por exemplo, 0,98 para dar mais peso ao giroscópio).
2. Aplique a fórmula de fusão a cada novo cálculo:
   - **Pitch filtrado**:
     $$ \theta = \alpha \cdot (\theta + Gy \times dt) + (1 - \alpha) \cdot \theta_{acc}$$
   - **Roll filtrado**:
     $$\phi = \alpha \cdot (\phi + Gx \times dt) + (1 - \alpha) \cdot \phi_{acc}$$

Onde:

- \($\alpha$\) é o peso do giroscópio na fusão (por exemplo, 0,98).
- \($\theta_{acc}$\) e \(\phi_{acc}\) são os ângulos calculados pelo acelerômetro.
- \($dt$\) é o intervalo de tempo entre cada amostra.

### Implementação Geral

Com isso, você deve conseguir uma estimativa mais precisa e menos sujeita ao desvio.
