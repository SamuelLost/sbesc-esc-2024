#ifndef _MQTT_H_
#define _MQTT_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"

#include "sdkconfig.h"
#include "utils.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TAG_MQTT "MQTT"

// #define INFO_AP "SSID: " CONFIG_WIFI_SSID ", password: " CONFIG_WIFI_PASSWORD

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

void wifi_init_sta();

void mqtt_app_start(void);

void mqtt_publish(const char *topic, const char *data);

void mqtt_subscribe(char *topic);

#ifdef __cplusplus
}
#endif

#endif