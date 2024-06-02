#ifndef MAIN_WIFI_APP_H
#define MAIN_WIFI_APP_H

#include "esp_netif.h"

// WIFI configuration settings
#define WIFI_AP_SSID                "ESP32_AP_PROTO"
#define WIFI_AP_PASSWORD            "sembrador_de_maiz"
#define WIFI_AP_CHANNEL             1
#define WIFI_AP_SSID_HIDDEN         0
#define WIFI_AP_MAX_CONNECTIONS     2
#define WIFI_AP_BEACON_INTERVAL     100
#define WIFI_AP_IP                  "192.168.0.1"
#define WIFI_AP_GATEWAY             "192.168.0.1"
#define WIFI_AP_NETMASK             "225.255.255.0.1"
#define WIFI_AP_BANDWIDTH           WIFI_BW_HT20
#define WIFI_STA_POWER_SAVE         WIFI_PS_NONE
#define MAX_SSID_LENGTH             32
#define MAX_PASSWORD_LENGTH         64
#define MAX_CONNECTION_RETRIES      5

// NETIF object for the station and access point
extern esp_netif_t* esp_netif_sta;
extern esp_netif_t* esp_netif_ap;

/*
* Message ID for the wifi application task
*/
typedef enum wifi_app_message
{
    WIFI_APP_MSG_START_HTTP_SERVER = 0,
    WIFI_APP_MSG_CONNECTING_FROM_HTTP_SERVER,
    WIFI_APP_MSG_STA_CONNECTED_GOT_IP,
    WIFI_APP_MSG_LOAD_SAVED_CREDENTIALS,
    WIFI_APP_MSG_STA_DISCONNECTED,
} wifi_app_message_e;

/**
 * @brief Structure for the message queue
 * 
 */
typedef struct wifi_app_queue_message
{
    wifi_app_message_e msgID;
} wifi_app_queue_message_t;

/**
 * @brief Sends  message to the queue
 * @param msgID message ID from the wifi_app_message_e enum
 * @return pdTrue if an item was successfully sent to the queue
 * 
 */
BaseType_t wifi_app_send_message(wifi_app_message_e msgID);

/**
 * @brief Starts the wifi RTOS task
 * 
 */

void wifi_app_start(void);

/**
 * @brief Gets the wifi config 
 * 
 */
wifi_config_t* wifi_app_get_wifi_config(void);

#endif