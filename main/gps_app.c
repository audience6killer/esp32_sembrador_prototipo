/**
 * @file gps_app.c
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2024-05-27
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "sys/param.h"
#include "math.h"

#include "gps_app.h"
#include "gps_parser.h"
#include "tasks_common.h"

#define UART_TX_PIN 17
#define UART_RX_PIN 16

const double ECCENTRICITY = 8.1819191E-2;
const double ECCENTRICITY_P2 = 6.6943801E-3;
const double A_SEMI_AXIS = 6378137.0f;

static const char *tag = "gps";

static const uart_port_t UART_NUM_PORT = UART_NUM_2;

static const size_t BUFF_SIZE = 1024;

// Parser object
static GPSData gpsData1;

// Uart handle
static QueueHandle_t uart_queue;

static originCoords origin_coord;

static int g_gps_origin_state = NO_SET;

esp_err_t gps_init_serial(void)
{
    ESP_LOGI(tag, "gps_uart_app: Configuring UART communication");

    const uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_PORT, &uart_config));

    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Setup UART buffered IO with event queue
    // const int uart_buffer_size = (1024 * 2);

    // Install UART driver using an event queue here
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_PORT, BUFF_SIZE * 2,
                                        BUFF_SIZE * 2, 10, &uart_queue, 0));

    return ESP_OK;
}

int gps_read_data(char *data, size_t max_size)
{
    // Read data from UART.

    int length = 0;
    ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_NUM_PORT, (size_t *)&length));
    length = uart_read_bytes(UART_NUM_PORT, data, MIN(length, max_size), pdMS_TO_TICKS(100));

    return length;
}

static void gps_app_task(void *pvParameter)
{
    // Configure the port
    gps_init_serial();

    char data[BUFF_SIZE];

    for (;;)
    {
        int length = gps_read_data(data, BUFF_SIZE);

        if (length > 0)
        {
            data[length] = '\0';
            char *line = strtok(data, "\n");
            while (line != NULL)
            {
                const char *nmea_code = "$GPGGA,";
                if (strncmp(line, nmea_code, 7) == 0)
                {

                    //GPSData gpsData1;
                    parse_gps_data(line, &gpsData1);

                    /*printf("Packet 1 - Valid\n");
                    printf("Packet: %s\n", line);
                    printf("Latitude: %.9f %c\n", gpsData1.latitude, gpsData1.latitude_dir);
                    printf("Longitude: %.9f %c\n", gpsData1.longitude, gpsData1.longitude_dir);
                    printf("Altitude: %.2f %c\n", gpsData1.altitude, gpsData1.altitude_dir);
                    printf("Number of Satellites: %d\n", gpsData1.num_satellites);
                    printf("Fix Quality: %d\n", gpsData1.fix_quality);*/

                    // ESP_LOGI(tag, "Read %d bytes: %s", length, line);
                }
                // Get the next line
                line = strtok(NULL, "\n");
            }

            //
        }

        vTaskDelay(pdMS_TO_TICKS(1200));
    }
}
double get_latitude(void)
{
    return gpsData1.latitude;
}

float get_longitude(void)
{
    return gpsData1.longitude;
}

double get_altitude(void)
{
    return gpsData1.altitude;
}
inline double deg2rad(double deg) {
    return deg * (M_PI / 180.0);
}

double get_n0(double lon)
{
    return A_SEMI_AXIS / sqrt(1 - ECCENTRICITY_P2 * pow(sin(deg2rad(lon)), 2));
}

esp_err_t calculate_ecef_coords(geoCoords *geo_coords, ecefCoords *ecef_coords)
{
    const double n0 = get_n0(geo_coords->lon);

    ecef_coords->x = (geo_coords->alt + n0) * cos(deg2rad(geo_coords->lat)) * cos(deg2rad(geo_coords->lon));
    ecef_coords->y = (geo_coords->alt + n0) * cos(deg2rad(geo_coords->lat)) * sin(geo_coords->lon);
    ecef_coords->z = (geo_coords->alt + (1 - ECCENTRICITY_P2) * n0) * sin(deg2rad(geo_coords->lat));

    return ESP_OK;
}


esp_err_t get_coords(ltpCoords *coords)
{
    coords->geodesic.alt = gpsData1.altitude;
    coords->geodesic.lon = gpsData1.longitude;
    coords->geodesic.lat = gpsData1.latitude;

    if(g_gps_origin_state != SET)
    {
        set_LTP_origin(&(coords->geodesic));
    }

    calculate_ecef_coords(&(coords->geodesic), &(coords->ecef));

    calculate_LTPCoords(coords);

    return ESP_OK;
}

esp_err_t set_LTP_origin(geoCoords *origin)
{
    origin_coord.geodesic.alt = origin->alt;
    origin_coord.geodesic.lon = origin->lon;
    origin_coord.geodesic.lat = origin->lat;

    origin_coord.n0 = get_n0(origin_coord.geodesic.lon);

    calculate_ecef_coords(origin, &origin_coord.ecef);

    const double lon_rad = deg2rad(origin_coord.geodesic.lon);
    const double lat_rad = deg2rad(origin_coord.geodesic.lat);

    origin_coord.cos_lonXsin_lat = cos(lon_rad) * sin(lat_rad);
    origin_coord.cos_latXcos_lon = cos(lat_rad) * cos(lon_rad);
    origin_coord.sin_lonXsin_lat = sin(lon_rad) * sin(lat_rad);
    origin_coord.cos_latXsin_lon = cos(lat_rad) * sin(lon_rad);

    origin_coord.cos_lat = cos(lat_rad);
    origin_coord.cos_lon = cos(lon_rad);
    origin_coord.sin_lat = cos(lat_rad);
    origin_coord.sin_lon = cos(lon_rad);

    ESP_LOGI(tag, "gps_app: Origin coordinate was set!");

    g_gps_origin_state = SET;

    return ESP_OK;
}

esp_err_t calculate_LTPCoords(ltpCoords *coords)
{
    const float x = coords->ecef.x - origin_coord.ecef.x;
    const float y = coords->ecef.y - origin_coord.ecef.y;
    const float z = coords->ecef.z - origin_coord.ecef.z;

    coords->enu.e = -origin_coord.sin_lon * x + origin_coord.cos_lon * y;  
    coords->enu.n = -origin_coord.cos_lonXsin_lat * x -origin_coord.sin_lonXsin_lat * y + origin_coord.cos_lat * z;
    coords->enu.u = origin_coord.cos_latXcos_lon * x + origin_coord.cos_latXsin_lon * y + origin_coord.sin_lat * z;

    return ESP_OK;
}


void gps_task_start(void)
{
    ESP_LOGI(tag, "gps_app: Creating Task");
    xTaskCreatePinnedToCore(&gps_app_task, "gps_task", GPS_UART_TASK_STACK_SIZE, NULL, GPS_UART_TASK_PRIORITY, NULL, GPS_UART_CORE_ID);
    // xTaskCreatePinnedToCore(&gps_app_task, "gps_task", GPS_UART_TASK_STACK_SIZE, NULL, GPS_UART_TASK_PRIORITY, NULL, GPS_UART_CORE_ID);
}
