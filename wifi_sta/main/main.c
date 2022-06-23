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
        printf("task\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id)
    {
        case SYSTEM_EVENT_STA_DISCONNECTED:
            if(retry_num < 20)
            {
                esp_wifi_connect();
                printf("Retry\n");
                retry_num++;
                vTaskDelay(1000);
            }
            else
            {
                printf("WiFi error\n");
                esp_wifi_stop();
            }
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            printf("WiFi connected\n");
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            printf("IP: " IPSTR "\n", IP2STR(&event->event_info.got_ip.ip_info.ip));
            break;
        case SYSTEM_EVENT_STA_START:
            printf("WiFi connecting...\n");
            esp_wifi_connect();
            retry_num = 0;
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
    esp_netif_create_default_wifi_sta();
    esp_event_loop_init(event_handler, NULL);

    // Initializing the WiFi environment
    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_config);

    // Setting the operation mode
    esp_wifi_set_mode(WIFI_MODE_STA);
    wifi_config_t wifi_sta_config = {
        .sta = {
            .ssid = "Nhung Toan",
            .password = "99999999"
        }
    };
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_sta_config);
    wifi_event_group = xEventGroupCreate();
    esp_wifi_start();
        xTaskCreate(wifi_handler, "wifi_handler", 4096, NULL, 10, NULL);
}