#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/event_groups.h"
#include "esp_http_server.h"
#include "http_server.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>


#define WIFI_CONNECTED_BIT          BIT0
#define WIFI_FAIL_BIT               BIT1

EventGroupHandle_t wifi_event_group;
int retry_num;
extern bool new_ssid_password;
extern char ssid_password_buffer[100];
// void wifi_handler(void)
// {
//     EventBits_t event_bit;
//     httpd_handle_t server = NULL;
//     for(;;)
//     {
//         event_bit = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdTRUE, pdFALSE, 1000 / portTICK_PERIOD_MS);
//         if(event_bit & WIFI_CONNECTED_BIT)
//         {
//             server = start_webserver();
//         }
//         else if(event_bit & WIFI_FAIL_BIT)
//         {

//         }
//         else 
//         {
//             vTaskDelay(1000 / portTICK_PERIOD_MS);
//         }
//     }
// }

// esp_err_t event_handler(void *ctx, system_event_t *event)
// {
    // switch(event->event_id)
    // {
    //     case SYSTEM_EVENT_STA_DISCONNECTED:
    //         if(retry_num < 20)
    //         {
    //             esp_wifi_connect();
    //             printf("Retry\n");
    //             retry_num++;
    //             vTaskDelay(1000);
    //         }
    //         else
    //         {
    //             printf("WiFi error\n");
    //             xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
    //             esp_wifi_stop();
    //         }
    //         break;
    //     case SYSTEM_EVENT_STA_CONNECTED:
    //         printf("WiFi connected\n");
    //         break;
    //     case SYSTEM_EVENT_STA_GOT_IP:
    //         printf("IP: " IPSTR "\n", IP2STR(&event->event_info.got_ip.ip_info.ip));
    //         xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    //         break;
    //     case SYSTEM_EVENT_STA_START:
    //         printf("WiFi connecting...\n");
    //         esp_wifi_connect();
    //         retry_num = 0;
    //         break;
    //     default:

    //         break;
    // }
//     return ESP_OK;
// }

// void app_main(void)
// {
//     nvs_flash_init();
//     esp_netif_init();
//     esp_event_loop_create_default();
//     esp_netif_create_default_wifi_sta();
//     esp_event_loop_init(event_handler, NULL);

//     // Initializing the WiFi environment
//     wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();
//     esp_wifi_init(&wifi_config);

    // Setting the operation mode
    // esp_wifi_set_mode(WIFI_MODE_STA);
    // wifi_config_t wifi_sta_config = {
    //     .sta = {
    //         .ssid = "Nhung Toan",
    //         .password = "99999999"
    //     }
    // };
    // esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_sta_config);
    // esp_wifi_start();
    // wifi_event_group = xEventGroupCreate();
    // xTaskCreate(wifi_handler, "wifi_handler", 4096, NULL, 10, NULL);
    
// }

void wifi_handler(void)
{
    httpd_handle_t server = NULL;
    server = start_webserver();
    char ngu_ssid[10] = {0};
    char ngu_password[10] = {0};
    for(;;)
    {
        if(new_ssid_password == true)
        {
            esp_wifi_stop();
            esp_wifi_set_mode(WIFI_MODE_NULL);
            sscanf(ssid_password_buffer, ",%[^,],%[^,],", ngu_ssid, ngu_password);
            esp_wifi_set_mode(WIFI_MODE_STA);
            wifi_config_t wifi_sta_config;
            strcpy((char*)wifi_sta_config.sta.ssid, ngu_ssid);
            strcpy((char*)wifi_sta_config.sta.password, ngu_password);
            wifi_sta_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
            wifi_sta_config.sta.pmf_cfg.capable = true;
            wifi_sta_config.sta.pmf_cfg.capable = false;
            esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_sta_config);
            esp_wifi_start();
            new_ssid_password = false;           
        }
        else
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
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
            xTaskCreate(wifi_handler, "wifi_handler", 4096, NULL, 10, NULL);
            break;
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
                xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
                esp_wifi_stop();
            }
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            printf("WiFi connected\n");
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            printf("IP: " IPSTR "\n", IP2STR(&event->event_info.got_ip.ip_info.ip));
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
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
    esp_wifi_start();
}