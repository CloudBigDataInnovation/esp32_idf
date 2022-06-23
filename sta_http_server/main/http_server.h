#ifndef __HTTP_SERVER_H__
#define __HTTP_SERVER_H__

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_eth.h"

#include <esp_http_server.h>
httpd_handle_t start_webserver(void);
void stop_webserver(httpd_handle_t server);
void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);
void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data);


#endif