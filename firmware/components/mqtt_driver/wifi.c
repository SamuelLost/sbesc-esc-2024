#include "wifi.h"

#define TAG_WIFI "WIFI"
#define CONFIG_MAXIMUM_RETRY 5
// #define CONFIG_WIFI_SSID "brisa-182118"
// #define CONFIG_WIFI_PASSWORD "1yzn6sri"
#define CONFIG_WIFI_SSID "Lost"
#define CONFIG_WIFI_PASSWORD "samuel1234"

#define INFO_AP "SSID: " CONFIG_WIFI_SSID ", password: " CONFIG_WIFI_PASSWORD

static EventGroupHandle_t wifi_event_group;
static int retry_count = 0;

static void _wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    switch (event_id) {
        case WIFI_EVENT_STA_START: {
            ESP_LOGI(TAG_WIFI, "WiFi connecting...");
            esp_wifi_connect();
            break;
        }
        case WIFI_EVENT_STA_CONNECTED: {
            ESP_LOGI(TAG_WIFI, "WiFi connected");
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED: {
            ESP_LOGI(TAG_WIFI, "WiFi disconnected");
            if (retry_count < CONFIG_MAXIMUM_RETRY) {
                esp_wifi_connect();
                retry_count++;
                ESP_LOGI(TAG_WIFI, "Retry to connect to the AP");
            } else {
                ESP_LOGI(TAG_WIFI, "Failed to connect to the AP");
                xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
            }
            break;
        }
        case IP_EVENT_STA_GOT_IP: {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
            ESP_LOGI(TAG_WIFI, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
            retry_count = 0;
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
            break;
        }
        default:
            break;
    }
}

void wifi_init_sta() {

    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, 
                                               ESP_EVENT_ANY_ID, 
                                               _wifi_event_handler, 
                                               NULL));

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, 
                                               IP_EVENT_STA_GOT_IP, 
                                               _wifi_event_handler, 
                                               NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, 
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, 
                                           pdFALSE, 
                                           pdFALSE, 
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_WIFI, "Connected to AP %s", INFO_AP);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG_WIFI, "Failed to connect to %s", INFO_AP);
        RESTART(TAG_WIFI, TIME_TO_RESTART);
    } else {
        ESP_LOGE(TAG_WIFI, "UNEXPECTED EVENT");
    }
}