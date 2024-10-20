idf_component_register(SRCS "mqtt.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "mqtt" "esp_netif" "esp_wifi" "nvs_flash" "utils")
