#ifndef _WIFI_H
#define _WIFI_H

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

#include "sdkconfig.h"
#include "utils.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TAG_MQTT "MQTT"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

void wifi_init_sta();

void wifi_stop_sta();

#ifdef __cplusplus
}
#endif

#endif