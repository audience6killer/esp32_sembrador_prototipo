#ifndef GPS_PARSER_H
#define GPS_PARSER_H

#include <stdbool.h>
#include <stdio.h>
/* If you're using with Arduino IDE or C++ compiler
#ifdef __cplusplus
 extern "C" {
#endif
*/


#define MAX_PACKET_LENGTH 128

typedef struct {
    char time[10];
    double latitude;
    char latitude_dir;
    double longitude;
    char longitude_dir;
    int fix_quality;
    int num_satellites;
    float hdop;
    float altitude;
    char altitude_dir;
    float geoid_height;
    char geoid_height_dir;
    float time_since_dgps_update;
    char checksum[10];
} GPSData;

bool validate_checksum(char *packet);
void parse_gps_data(char *packet, GPSData *gpsData);

/* If you're using with Arduino IDE or C++ compiler
#ifdef __cplusplus
}
*/
#endif  // GPS_PARSER_H