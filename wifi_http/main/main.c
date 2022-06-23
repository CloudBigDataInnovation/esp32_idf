#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#define ESP_WIFI_SSID               CONFIG_ESP_WIFI_SSID
#define ESP_WIFI_PASS               CONFIG_ESP_WIFI_PASSWORD
#define ESP_MAXIMUM_RETRY           CONFIG_ESP_MAXIMUM_RETRY
#define WIFI_CONNECTED_BIT          BIT0
#define WIFI_FAIL_BIT               BIT1

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    if(event->event_id == SYSTEM_EVENT_STA_DISCONNECTED)
    {
        if(s_retry_num < ESP_MAXIMUM_RETRY) 
        {
            ESP_ERROR_CHECK(esp_wifi_connect());
            s_retry_num++;
            printf("connect to the AP fail, retry to connect to the AP\n");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    }
    if(event->event_id == SYSTEM_EVENT_STA_GOT_IP)
    {
        printf("Your IP address is " IPSTR "\n", IP2STR(&event->event_info.got_ip.ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    if(event->event_id == SYSTEM_EVENT_STA_START)
    {   
        printf("Connecting...\n");
        ESP_ERROR_CHECK(esp_wifi_connect());
        s_retry_num = 0;
    }
    return ESP_OK;
}

void app_main(void)
{
    s_wifi_event_group = xEventGroupCreate();
    nvs_flash_init();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    // Initializing the WiFi environment
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));


    // ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // Setting the operation mode
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_config_t sta_config = {
        .sta = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASS,
            .bssid_set = 0
        }
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));

    
    ESP_ERROR_CHECK(esp_wifi_start());
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdTRUE,
        pdFALSE,
        portMAX_DELAY);
    if(bits & WIFI_FAIL_BIT)
    {
        printf("Connect Error\n");
    }
    if(bits & WIFI_CONNECTED_BIT)
    {
        printf("WiFi connected\n");
    }
    while(1)
    {

    }
}

