/**
 * @file gps_app.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-05-27
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef MAIN_GPS_APP_H
#define MAIN_GPS_APP_H

typedef struct {
    double lat;
    float lon;
    float alt;
    
} geoCoords;

typedef struct {
    float x;
    float y;
    float z;
} ecefCoords;

typedef struct {
    float e;
    float n;
    float u;
} enuCoords;

typedef enum gps_app_origin_coord_status {
    NO_SET = 0,
    SET,
} gps_app_origin_coord_status_e;

typedef struct 
{
    geoCoords geodesic;
    ecefCoords ecef;

    double cos_lonXsin_lat;
    double cos_latXcos_lon;
    double sin_lonXsin_lat;
    double cos_latXsin_lon;
    double cos_lon;
    double cos_lat;
    double sin_lon;
    double sin_lat;

    double n0;
    
} originCoords;

typedef struct {
    geoCoords geodesic;
    ecefCoords ecef;
    enuCoords enu;

} ltpCoords;


/**
 * @brief Task start
 * 
 */
void gps_task_start(void);

/**
 * @brief Initializes gps serial configuration
 * 
 * @return esp_err_t 
 */
esp_err_t gps_init_serial(void); 

/**
 * @brief Begins serial communication
 * 
 * @return esp_err_t 
 */
//esp_err_t gps_begin_serial_comm(void);


/**
 * @brief Reads data from the serial port
 * 
 * @return esp_err_t 
 */
int gps_read_data(char* data, size_t max_size);

/**
 * @brief Get the latitude object
 * 
 * @return double 
 */
double get_latitude(void);

float get_longitude(void);

double get_altitude(void);

esp_err_t set_LTP_origin(geoCoords *origin);

esp_err_t get_coords(ltpCoords *coords);

esp_err_t calculate_LTPCoords(ltpCoords *coords);

#endif