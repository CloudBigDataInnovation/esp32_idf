

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"

#define WIFI_CONNECTED_BIT          BIT0
#define WIFI_FAIL_BIT               BIT1

EventGroupHandle_t wifi_event_group;
int retry_num = 0;

void wifi_handler(void)
{
    for(;;)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    wifi_sta_list_t station_list;
    switch(event->event_id)
    {
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            printf("Station "MACSTR" leave, AID = %d\n", MAC2STR(event->event_info.sta_disconnected.mac),
                     event->event_info.sta_disconnected.aid);
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            printf("Station "MACSTR" join, AID = %d\n", MAC2STR(event->event_info.sta_connected.mac),
                     event->event_info.sta_connected.aid);
            break;
        case SYSTEM_EVENT_AP_START:
            printf("Starting Access Point...\n");
            break;
        default:
            break;
    }
    return ESP_OK;
    
}

void app_main(void)
{
    nvs_flash_init();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    esp_event_loop_init(event_handler, NULL);

    // Initializing the WiFi environment
    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_config);

    // Setting the operation mode
    esp_wifi_set_mode(WIFI_MODE_AP);
    wifi_config_t wifi_ap_config = {
        .ap = {
            .ssid = "dungdeptrai",
            .ssid_len = strlen("dungdeptrai"),
            .password = "99999999",
            .channel = 0,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        }
    };
    esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_ap_config);
    wifi_event_group = xEventGroupCreate();
    esp_wifi_start();
    xTaskCreate(wifi_handler, "wifi_handler", 4096, NULL, 10, NULL);
}