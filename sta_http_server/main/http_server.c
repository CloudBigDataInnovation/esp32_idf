
#include "http_server.h"

const char *TAG = "HTTP Server";
int x = 0, y = 0;
char ssid_password_buffer[100] = {0};
bool new_ssid_password = false;
esp_err_t get_hust_handler(httpd_req_t *req)
{
    extern const unsigned char hust_png_start[] asm("_binary_hust_png_start");
    extern const unsigned char hust_png_end[] asm("_binary_hust_png_end");
    const size_t hust_png_size = (hust_png_end - hust_png_start);
    httpd_resp_set_type(req, "image/png");
    httpd_resp_send(req, (const char *)hust_png_start, hust_png_size);
    return ESP_OK;
}

esp_err_t get_mandevices_handler(httpd_req_t *req)
{
    extern const unsigned char mandevices_png_start[] asm("_binary_mandevices_png_start");
    extern const unsigned char mandevices_png_end[] asm("_binary_mandevices_png_end");
    const size_t mandevices_png_size = (mandevices_png_end - mandevices_png_start);
    httpd_resp_set_type(req, "image/png");
    httpd_resp_send(req, (const char *)mandevices_png_start, mandevices_png_size);
    return ESP_OK;
}

esp_err_t index_html_get_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "307 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "/index_html");
    httpd_resp_send(req, NULL, 0); // Response body can be empty
    return ESP_OK;
}
esp_err_t get_html_handler(httpd_req_t *req)
{
    extern const unsigned char index_html_start[] asm("_binary_index_html_start");
    extern const unsigned char index_html_end[] asm("_binary_index_html_end");
    const size_t index_html_size = (index_html_end - index_html_start);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)index_html_start, index_html_size);
    return ESP_OK;
}

esp_err_t get_mpu6050_handler(httpd_req_t *req)
{
    char str[100]= {0};
    x++;
    y++;
    sprintf(str, "{\"temperature\": \"%d\",\"humidity\": \"%d\"}", x, y);
    httpd_resp_send(req, (const char*)str, strlen(str));
    return ESP_OK;
}
httpd_uri_t get_data_hust = {
    .uri       = "/get_hust",
    .method    = HTTP_GET,
    .handler   = get_hust_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = NULL
};

httpd_uri_t get_html = {
    .uri = "/get_html",
    .method = HTTP_GET,
    .handler = get_html_handler,
    .user_ctx = NULL
};

httpd_uri_t get_mpu6050 = {
    .uri = "/get_mpu6050",
    .method = HTTP_GET,
    .handler = get_mpu6050_handler,
    .user_ctx = NULL
};

httpd_uri_t get_data_mandevices = {
    .uri       = "/get_mandevices",
    .method    = HTTP_GET,
    .handler   = get_mandevices_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = NULL
};

esp_err_t post_ssid_password_handler(httpd_req_t *req)
{
    // Read data for the request
    httpd_req_recv(req, ssid_password_buffer, req->content_len);
    new_ssid_password = true;
    // End response
    // httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

httpd_uri_t post_data = {
    .uri       = "/post_ssid_password",
    .method    = HTTP_POST,
    .handler   = post_ssid_password_handler,
    .user_ctx  = NULL
};

esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    if (strcmp("/hello", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI is not available");
        /* Return ESP_OK to keep underlying socket open */
        return ESP_OK;
    } else if (strcmp("/echo", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
        /* Return ESP_FAIL to close underlying socket */
        return ESP_FAIL;
    }
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &get_data_hust);
        httpd_register_uri_handler(server, &get_data_mandevices);
        httpd_register_uri_handler(server, &post_data);
        httpd_register_uri_handler(server, &get_html);
        httpd_register_uri_handler(server, &get_mpu6050);
        httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_webserver(*server);
        *server = NULL;
    }
}

void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}
