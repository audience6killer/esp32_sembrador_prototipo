#include "gps_parser.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static double coords_to_decimal_degree(double value)
{  
    int degrees = (int)(value / 100);
    double minutes = value - (degrees * 100.0);
    return degrees + (minutes / 60.0);
}

bool validate_checksum(char *packet)
{
    int i = 1;
    char checksum = 0;

    // Calculating the checksum of recieved data uptil * character
    while (packet[i] != '*' && packet[i] != '\0')
    {
        checksum = checksum ^ packet[i];
        i++;
    }

    // Extracting the checksum from the string - last 3 characters
    char extracted_checksum[3];
    int length = strlen(packet);
    extracted_checksum[0] = packet[length - 2];
    extracted_checksum[1] = packet[length - 1];
    extracted_checksum[2] = '\0';

    // converts calculated checksum in to long integer
    char *endptr;
    char calculated_checksum = strtol(extracted_checksum, &endptr, 16);

    // Null packet
    if (packet == NULL)
    {
        printf("NULL Found\n");
        return 0;
    }

    // Invalid format of packet or empty string
    if (strncmp(packet, "$GPGGA,", 7) != 0)
    {
        // Not a GGA packet
        printf("Invalid Packet\n");
        return 0;
    }
    return (checksum == calculated_checksum);
}

void parse_gps_data(char *packet, GPSData *gpsData)
{

    // Create a copy of the packet to avoid modifying the original string
    char copy[MAX_PACKET_LENGTH];
    strncpy(copy, packet, MAX_PACKET_LENGTH);

    char *token;
    // Extract individual parameter, this will return first token before first comma
    token = strtok(copy, ",");
    int i = 0;
    

    while (token != NULL && i < 14)
    {
        switch (i)
        {
        case 1:
            // Copy the token to the `time` field of `gpsData`
            strncpy(gpsData->time, token, sizeof(gpsData->time));
            break;
        case 2:
            // Convert the token to a floating-point latitude value
            // convert it to decimal degrees
            
            gpsData->latitude = coords_to_decimal_degree(strtod(token, NULL));
            // gpsData->latitude = strtof(token, NULL);
            break;
        case 3:
            // Store the first character of the token as the latitude direction
            gpsData->latitude_dir = token[0];
            if (gpsData->latitude_dir == 'S')
            {
                gpsData->latitude = -gpsData->latitude;
            }
            break;
        case 4:
            // Convert the token to a floating-point longitude value
            
            gpsData->longitude = coords_to_decimal_degree(strtod(token, NULL));
            // gpsData->longitude = value;
            break;
        case 5:
            // Store the first character of the token as the longitude direction
            gpsData->longitude_dir = token[0];
            if (gpsData->longitude_dir == 'W')
            {
                gpsData->longitude = -gpsData->longitude;
            }
            break;
        case 6:
            // Convert the token to an integer fix quality value
            gpsData->fix_quality = atoi(token);
            break;
        case 7:
            // Convert the token to an integer number of satellites value
            gpsData->num_satellites = atoi(token);
            break;
        case 8:
            // Convert the token to a floating-point HDOP (horizontal dilution of precision) value
            gpsData->hdop = strtof(token, NULL);
            break;
        case 9:
            // Convert the token to a floating-point altitude value
            gpsData->altitude = strtof(token, NULL);
            break;
        case 10:
            // Store the first character of the token as the altitude direction
            gpsData->altitude_dir = token[0];
            break;
        case 11:
            // Convert the token to a floating-point geoid height value
            gpsData->geoid_height = strtof(token, NULL);
            break;
        case 12:
            // Store the first character of the token as the geoid height direction
            gpsData->geoid_height_dir = token[0];
            break;
        case 13:
            // Convert the token to a floating-point time since DGPS (Differential GPS) update value
            gpsData->time_since_dgps_update = strtof(token, NULL);
            break;
        }
        // Get the next token by calling strtok() with NULL as the first argument
        token = strtok(NULL, ",");
        i++;
    }

    // Extract the checksum from the last 2 characters of the packet
    int size = strlen(packet);
    gpsData->checksum[0] = packet[size - 2];
    gpsData->checksum[1] = packet[size - 1];
    gpsData->checksum[2] = '\0';
}