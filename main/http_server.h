/**
 * @file http_server.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-22
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MAIN_HTTP_SERVER_H
#define MAIN_HTTP_SERVER_H

#define OTA_UPDATE_PENDING      0
#define OTA_UPDATE_SUCCESSFUL   1
#define OTA_UPDATE_FAILED       -1

/**
 * @brief Connection status for wifi
 * 
 */
typedef enum http_server_wifi_connect_status
{
    NONE = 0,
    HTTP_WIFI_STATUS_CONNECTING,
    HTTP_WIFI_STATUS_CONNECT_FAILED,
    HTTP_WIFI_STATUS_CONNECT_SUCCESSFUL, 
    HTTP_WIFI_STATUS_DISCONNECTED
} http_server_wifi_connect_status_e;

typedef enum http_server_message
{
    HTTP_MSG_WIFI_CONNECT_INIT = 0,
    HTTP_MSG_WIFI_CONNECT_SUCCESS,
    HTTP_MSG_WIFI_CONNECT_FAIL,
    HTTP_MSG_WIFI_USER_DISCONNECT,
    HTTP_MSG_OTA_UPDATE_SUCCESSFUL,
    HTTP_MSG_OTA_UPDATE_FAILED
} http_server_message_e;

typedef struct http_server_queue_message
{
    http_server_message_e msgID;
} http_server_queue_message_t;

/**
 * @brief Send a message to the queue
 * @param msgID message ID from the http_server_message_e enum
 * @return pdTrue if an item was successfully sent
 */
BaseType_t http_server_monitor_send_message(http_server_message_e msgID);

/**
 * @brief Starts the http server
 * 
 */
void http_server_start(void);

/**
 * @brief Stops http server
 * 
 */
void http_server_stop(void);

/**
 * @brief Timer callback function which calls esp_restart upon seccessful firmware update
 * 
 */

void http_server_update_reset_callback(void *args);

#endif













