#include <stdio.h>
#include "mqtt.h"

#define TAG_WIFI "WIFI"
#define CONFIG_MAXIMUM_RETRY 5
#define CONFIG_WIFI_SSID "brisa-3305823"
#define CONFIG_WIFI_PASSWORD "xqq5e010"

#define INFO_AP "SSID: " CONFIG_WIFI_SSID ", password: " CONFIG_WIFI_PASSWORD
// #define CONFIG_MQTT_BROKER "mqtt://192.168.0.4:1883"
#define CONFIG_MQTT_BROKER "mqtt://mqtt.eclipseprojects.io"

static EventGroupHandle_t wifi_event_group;
static int retry_count = 0;



esp_mqtt_client_handle_t client;

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
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
                                               wifi_event_handler, 
                                               NULL));

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, 
                                               IP_EVENT_STA_GOT_IP, 
                                               wifi_event_handler, 
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
    } else {
        ESP_LOGE(TAG_WIFI, "UNEXPECTED EVENT");
    }
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "Connected to MQTT broker");
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG_MQTT, "Disconnected from MQTT broker");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG_MQTT, "Subscribed. Message ID: %d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG_MQTT, "Unsubscribed. Message ID: %d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG_MQTT, "Published message ID: %d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG_MQTT, "Data from topic [%.*s]: %.*s", event->topic_len, event->topic, event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG_MQTT, "Error");
            break;
        default:
            ESP_LOGI(TAG_MQTT, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    mqtt_event_handler_cb(event_data);
}

void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_MQTT_BROKER,
        // .broker.address.port = 1883,
        // .credentials.username = "mybroker",
        // .credentials.authentication.password = "12345"
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

void mqtt_publish(const char *topic, const char *data) {
    esp_mqtt_client_publish(client, topic, data, 0, 1, 0);
}

void mqtt_subscribe(char *topic) {
    esp_mqtt_client_subscribe(client, topic, 0);
}