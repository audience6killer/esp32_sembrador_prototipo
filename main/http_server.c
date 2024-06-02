/**
 * @file http_server.c
 * @author UPIITA
 * @brief
 * @version 0.1
 * @date 2024-05-22
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_wifi.h"
#include "sys/param.h"

#include "http_server.h"
#include "tasks_common.h"
#include "wifi_app.h"
#include "gps_app.h"

static const char tag[] = "http_server";

// WIFI conecct status
static int g_wifi_connect_status = NONE;

// Firmware update status
static int g_fw_update_status = OTA_UPDATE_PENDING;

// Http server task handle
static httpd_handle_t http_server_handle = NULL;

// Http server monitor task handle
static TaskHandle_t task_http_server_monitor = NULL;

// Queue handle used to manipulate the main queue of events
static QueueHandle_t http_server_monitor_queue_handle;

/**
 * @brief ESP32 timer configuration passed to esp_timer_create
 *
 */
const esp_timer_create_args_t fw_update_reset_args = {
    .callback = &http_server_update_reset_callback,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "fw_update_reset"};
esp_timer_handle_t fw_update_reset;

/**
 * @brief Checks the g_fw_status and creates the fw_update_reset timer
 * if g_fw_update_status is true.
 *
 */
static void http_server_fw_update_reset_timer(void)
{
    if (g_fw_update_status == OTA_UPDATE_SUCCESSFUL)
    {
        ESP_LOGI(tag, "http_server_fw_update_reset_timer: FW updated successfully starting FW update reset timer");

        // Give the web page a chance to receive an acknowledge back and initialize the timer
        ESP_ERROR_CHECK(esp_timer_create(&fw_update_reset_args, &fw_update_reset));
        ESP_ERROR_CHECK(esp_timer_start_once(fw_update_reset, 8000000));
    }
    else
    {
        ESP_LOGI(tag, "http_server_fw_update_reset_timer: Fw update unsuccessfully");
    }
}

/**
 * @brief HTTP server monitor task used to track events of the HTTP server
 * @param pvParameters parameter which can be passed to the task
 */
static void http_server_monitor(void *parameter)
{
    http_server_queue_message_t msg;

    for (;;)
    {
        if (xQueueReceive(http_server_monitor_queue_handle, &msg, portMAX_DELAY))
        {
            switch (msg.msgID)
            {
            case HTTP_MSG_WIFI_CONNECT_INIT:
                ESP_LOGI(tag, "HTTP_MSG_WIFI_CONNECT_INIT");
                g_wifi_connect_status = HTTP_WIFI_STATUS_CONNECTING;
                break;
            case HTTP_MSG_WIFI_CONNECT_SUCCESS:
                ESP_LOGI(tag, "HTTP_MSG_WIFI_CONNECT_SUCCESS");
                g_wifi_connect_status = HTTP_WIFI_STATUS_CONNECT_SUCCESSFUL;
                break;
            case HTTP_MSG_WIFI_CONNECT_FAIL:
                ESP_LOGI(tag, "HTTP_MSG_WIFI_CONNECT_FAIL");
                g_wifi_connect_status = HTTP_WIFI_STATUS_CONNECT_FAILED;
                break;
            case HTTP_MSG_WIFI_USER_DISCONNECT:
                ESP_LOGI(tag, "HTTP_MSG_WIFI_USER_DISCONNECT");
                g_wifi_connect_status = HTTP_WIFI_STATUS_DISCONNECTED;
                break;
            case HTTP_MSG_OTA_UPDATE_FAILED:
                ESP_LOGI(tag, "HTTP_MSG_OTA_UPDATE_FAILED");
                g_fw_update_status = OTA_UPDATE_FAILED;
                break;
            case HTTP_MSG_OTA_UPDATE_SUCCESSFUL:
                ESP_LOGI(tag, "HTTP_MSG_OTA_UPDATE_SUCCESSFUL");
                g_fw_update_status = OTA_UPDATE_SUCCESSFUL;
                http_server_fw_update_reset_timer();
                break;
            default:
                break;
            }
        }
    }
}

/**
 * @brief Uri handlers
 *
 * @param req
 * @return esp_err_t
 */
// Handlers to respond with embedded file content

/**
 * @brief Receive  file via the web and handles the firmware update
 * @param req_http request for which the uri needs to be handled
 * @return ESP_OK, otherwise ESP_FAIL if timeout occurs and the update cannot be started
 *
 */
esp_err_t http_server_ota_update_handler(httpd_req_t *req)
{
    esp_ota_handle_t ota_handle;

    char ota_buff[1024];
    int content_length = (*req).content_len;
    int content_received = 0;
    int recv_len;
    bool is_req_body_started = false;
    bool flash_successful = false;

    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);

    do
    {
        // Read the data from the request
        if ((recv_len = httpd_req_recv(req, ota_buff, MIN(content_length, sizeof(ota_buff)))) < 0)
        {
            // Occurred an error
            // Check if timeout ocurred
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT)
            {
                ESP_LOGI(tag, "http_server_ota_update_handler: Socket timeout");
                continue;
            }
            ESP_LOGI(tag, "http_server_ota_update_handler: OTA other Error %d", recv_len);
        }
        ESP_LOGI(tag, "http_server_OTA_update_handler: OTA RX: %d of %d", content_received, content_length);

        // If this is the first data we are receiving
        // If so, it will haave the information in the handler that we used
        if (!is_req_body_started)
        {
            is_req_body_started = true;

            // get the location of the .bin file content (remove the web form data)

            char *body_start_p = strstr(ota_buff, "\r\n\r\n") + 4;
            int body_part_len = recv_len - (body_start_p - ota_buff);

            // printf("Received %s\n", ota_buff);
            // printf("The body part len is: %d\n", body_part_len);

            ESP_LOGI(tag, "http_server_OTA_update_handled: OTA file size %d\r\n", content_length);

            esp_err_t err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
            if (err != ESP_OK)
            {
                ESP_LOGI(tag, "http_server_OTA_update_handler: Error with OTA begin, cancelling OTA");
                return ESP_FAIL;
            }
            else
            {
                printf("http_server_OTA_update_handler: Writing to partition subtype %d at offset 0x%x\r\n", (*update_partition).subtype, (*update_partition).address);
            }

            // Write this first part of the data
            esp_ota_write(ota_handle, body_start_p, body_part_len);
            content_received += body_part_len;
        }
        else
        {
            esp_ota_write(ota_handle, ota_buff, recv_len);
            content_received += recv_len;
        }

    } while (recv_len > 0 && content_received < content_length);

    if (esp_ota_end(ota_handle) == ESP_OK)
    {
        // Lets update the partition
        if (esp_ota_set_boot_partition(update_partition) == ESP_OK)
        {
            const esp_partition_t *boot_partition = esp_ota_get_boot_partition();
            ESP_LOGI(tag, "http_server_OTA_update_handler: Next boot partition subtype %d at offset 0x0%x", (*boot_partition).subtype, (*boot_partition).address);
            flash_successful = true;
        }
        else
        {
            ESP_LOGI(tag, "http_server_ota_update_handler: Flashed error!!");
        }
    }
    else
    {
        ESP_LOGI(tag, "http_server_OTA_update_handler: esp_ota_end Error!!");
    }

    // We won't update the global variables throughout the file, we send the message about the status
    if (flash_successful)
    {
        http_server_monitor_send_message(HTTP_MSG_OTA_UPDATE_SUCCESSFUL);
    }
    else
    {
        http_server_monitor_send_message(HTTP_MSG_OTA_UPDATE_FAILED);
    }

    return ESP_OK;
}

/**
 * @brief OTA status responds with the firmware update status after the OTA
 * update is started and responds with the compile time/date when the page is
 * first requested
 *
 * @param req HTTP request for which the url needs to be handled
 * @return esp_err_t ESP_OK
 */
esp_err_t http_server_ota_status_handler(httpd_req_t *req)
{
    char otaJSON[100];

    ESP_LOGI(tag, "OTA status requested");

    sprintf(otaJSON, "{\"ota_update_status\":%d,\"compile_time\":\"%s\",\"compile_date\":\"%s\"}", g_fw_update_status, __TIME__, __DATE__);

    // ESP_LOGI(tag, "{\"ota_update_status\":%d,\"compile_time\":\"%s\",\"compile_date\":\"%s\"}", g_fw_update_status, __TIME__, __DATE__);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_send(req, otaJSON, strlen(otaJSON));

    return ESP_OK;
}

/**
 * @brief wifiConnect.json handler is invoked after connect the button is pressed
 * and handles the receiving SSID and password entered by the use
 *
 * @param req HTTP request for which the uri needs to be handled.
 * @return esp_err_t
 */
static esp_err_t http_server_wifi_connect_json_handler(httpd_req_t *req)
{
    ESP_LOGI(tag, "/wifiConnect.json requested");

    size_t len_ssid = 0, len_pass = 0;

    char *ssid_str = NULL, *pass_str = NULL;

    // Get ssid header
    len_ssid = httpd_req_get_hdr_value_len(req, "my-connect-ssid") + 1;
    if (len_ssid > 1)
    {
        ssid_str = malloc(len_ssid);
        if (httpd_req_get_hdr_value_str(req, "my-connect-ssid", ssid_str, len_ssid) == ESP_OK)
        {
            ESP_LOGI(tag, "http_server_wifi_connect_json_handler: Found header => my-connect-ssid: %s", ssid_str);
        }
    }

    // Get pwd header
    len_pass = httpd_req_get_hdr_value_len(req, "my-connect-pwd") + 1;
    if (len_pass > 1)
    {
        pass_str = malloc(len_pass);
        if (httpd_req_get_hdr_value_str(req, "my-connect-pwd", pass_str, len_pass) == ESP_OK)
        {
            ESP_LOGI(tag, "http_server_wifi_connect_json_handler: Found header => my-connect-pass: %s", pass_str);
        }
    }

    // Update the Wifi network configuration
    wifi_config_t *wifi_config = wifi_app_get_wifi_config();
    memset(wifi_config, 0x00, sizeof(wifi_config_t));
    memcpy((*wifi_config).sta.ssid, ssid_str, len_ssid);
    memcpy((*wifi_config).sta.password, pass_str, len_pass);

    wifi_app_send_message(WIFI_APP_MSG_CONNECTING_FROM_HTTP_SERVER);

    free(ssid_str);
    free(pass_str);

    return ESP_OK;
}

static esp_err_t http_server_wifi_connect_status_json_handler(httpd_req_t *req)
{
    ESP_LOGI(tag, "/wifiConnectedStatus requested");

    char statusJSON[100];

    sprintf(statusJSON, "{\"wifi_connect_status\":%d}", g_wifi_connect_status);

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, statusJSON, strlen(statusJSON));

    return ESP_OK;
}
/**
 * @brief WifiConnectInfo.json handler updates the web page with
 * connection information
 *
 * @param req
 * @return esp_err_t
 */
static esp_err_t http_server_get_wifi_connect_info_json_handler(httpd_req_t *req)
{
    ESP_LOGI(tag, "wifi_connect_info.json requested");

    char ipInfoJson[200];
    memset(ipInfoJson, 0, sizeof(ipInfoJson));

    char ip[IP4ADDR_STRLEN_MAX];

    if (g_wifi_connect_status == HTTP_WIFI_STATUS_CONNECT_SUCCESSFUL)
    {
        wifi_ap_record_t wifi_data;
        ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&wifi_data));
        char *ssid = (char *)wifi_data.ssid;

        esp_netif_ip_info_t ip_info;
        ESP_ERROR_CHECK(esp_netif_get_ip_info(esp_netif_sta, &ip_info));
        esp_ip4addr_ntoa(&ip_info.ip, ip, IP4ADDR_STRLEN_MAX);

        sprintf(ipInfoJson, "{\"ip\":\"%s\", \"ssid\":\"%s\"}", ip, ssid);
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, ipInfoJson, strlen(ipInfoJson));

    return ESP_OK;
}

static esp_err_t http_server_get_gps_info_json_handler(httpd_req_t *req)
{
    ESP_LOGI(tag, "gps_info.json requested");

    char gpsInfoJSON[300];

    if (g_wifi_connect_status == HTTP_WIFI_STATUS_CONNECT_SUCCESSFUL)
    {
        ltpCoords coords;

        get_coords(&coords);

        if (coords.geodesic.alt != 0 && coords.geodesic.lat != 0 && coords.geodesic.lon != 0)
        {
            // sprintf(gpsInfoJSON, "{\"latitude\":\"%.9f\", \"longitude\":\"%.9f\", \"altitude\":\"%.4f\"}", coordinates.lat, coordinates.lon, coordinates.alt);
            sprintf(gpsInfoJSON,
                    "{ \"geodesic\": { \"altitude\": %.3f, \"latitude\": %.9f, \"longitude\": %.9f }, "
                    "\"ltp\": { \"e\": %.9f, \"n\": %.9f, \"u\": %.9f } }",
                    coords.geodesic.alt, coords.geodesic.lat, coords.geodesic.lon,
                    coords.enu.e, coords.enu.n, coords.enu.u);
            // json = {
            //     geodesic: {
            //         altitude: coords.geodesic.alt,
            //         latitude: coords.geodesic.lat,
            //         longitude: coords.geodesic.lon,
            //     },
            //     ltp {
            //         e: coords.enu.e,
            //         n: coords.enu.n,
            //         u: coords.enu.u,
            //     }
            // }
        }
        else
        {
            sprintf(gpsInfoJSON, "{\"empty\":\"true\"}");
        }
        //sprintf(gpsInfoJSON, "{\"empty\":\"true\"}");
    }

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, gpsInfoJSON, strlen(gpsInfoJSON));

    return ESP_OK;
}

static esp_err_t http_server_get_validate_connection(httpd_req_t *req)
{
    ESP_LOGI(tag, "validate.json requested");

    char info[25];

    sprintf(info, "{\"connection\":\"%s\"}", "ok");

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, info, strlen(info));

    return ESP_OK;
}

/**
 * @brief Sets up the default http server configuration
 * @return http server instance if successfull, null otherwise
 */
static httpd_handle_t http_server_configure(void)
{
    // Generate the default configuration
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Create the HTTP server monitor task
    xTaskCreatePinnedToCore(&http_server_monitor, "http_server_monitor", HTTP_SERVER_MONITOR_TASK_STACK_SIZE, NULL, HTTP_SERVER_MONITOR_TASK_PRIORITY, &task_http_server_monitor, HTTP_SERVER_MONITOR_CORE_ID);
    // Create the HTTP server monitor task
    http_server_monitor_queue_handle = xQueueCreate(3, sizeof(http_server_queue_message_t));

    // The core that the http will run on
    config.core_id = HTTP_SERVER_CORE_ID;

    // Adjust the default priority
    config.task_priority = HTTP_SERVER_TASK_PRIORITY;

    // Bump up the stack size
    config.stack_size = HTTP_SERVER_TASK_STACK_SIZE;

    // Increase uri handlers
    config.max_uri_handlers = 20;

    // Increase the timeout limits
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;

    ESP_LOGI(tag, "http_server_configure: Starting on port: '%d' with task priority: '%d'", config.server_port, config.task_priority);

    // Start httpd server
    if (httpd_start(&http_server_handle, &config) == ESP_OK)
    {
        ESP_LOGI(tag, "http_server_configure: Registering uri handlers");

        // Register OTAupdate handler
        httpd_uri_t OTA_update_handler = {
            .uri = "/OTAupdate",
            .method = HTTP_POST,
            .handler = http_server_ota_update_handler,
            .user_ctx = NULL

        };
        httpd_register_uri_handler(http_server_handle, &OTA_update_handler);

        httpd_uri_t OTA_status_handler = {
            .uri = "/OTAstatus",
            .method = HTTP_GET,
            .handler = http_server_ota_status_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(http_server_handle, &OTA_status_handler);

        httpd_uri_t wifi_connect_json = {
            .uri = "/wifiConnect.json",
            .method = HTTP_POST,
            .handler = http_server_wifi_connect_json_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(http_server_handle, &wifi_connect_json);

        httpd_uri_t wifi_connect_status_json = {
            .uri = "/wifiConnectStatus",
            .method = HTTP_GET,
            .handler = http_server_wifi_connect_status_json_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(http_server_handle, &wifi_connect_status_json);

        httpd_uri_t wifi_connect_info_json = {
            .uri = "/wifiConnectInfo.json",
            .method = HTTP_GET,
            .handler = http_server_get_wifi_connect_info_json_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(http_server_handle, &wifi_connect_info_json);

        httpd_uri_t gps_info_json = {
            .uri = "/gpsInfo.json",
            .method = HTTP_GET,
            .handler = http_server_get_gps_info_json_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(http_server_handle, &gps_info_json);

        httpd_uri_t gps_validate_json = {
            .uri = "/validate.json",
            .method = HTTP_GET,
            .handler = http_server_get_validate_connection,
            .user_ctx = NULL};
        httpd_register_uri_handler(http_server_handle, &gps_validate_json);

        return http_server_handle;
    }

    return NULL;
}

BaseType_t http_server_monitor_send_message(http_server_message_e msgID)
{
    http_server_queue_message_t msg;
    msg.msgID = msgID;
    return xQueueSend(http_server_monitor_queue_handle, &msg, portMAX_DELAY);
}

void http_server_start(void)
{
    if (http_server_handle == NULL)
    {
        http_server_handle = http_server_configure();
    }
}

void http_server_stop(void)
{
    if (http_server_handle)
    {
        httpd_stop(http_server_handle);
        ESP_LOGI(tag, "http_server_stop: stopping http server.");
        http_server_handle = NULL;
    }
}

void http_server_update_reset_callback(void *args)
{
    ESP_LOGI(tag, "http_server_fw_update_reset_callback: Timer timed_out, restarting the device");
    esp_restart();
}
