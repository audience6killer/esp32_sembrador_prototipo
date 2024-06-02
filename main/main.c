/**
 * @file main.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-30
 * 
 * @copyright Copyright (c) 2024
 * 
 */


#include "nvs_flash.h"

#include "wifi_app.h"

#include "gps_app.h"

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
         ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Start wifi app
    wifi_app_start();

    // Start gps task
    gps_task_start();
}