#include "neo_m9n.h"
#include <string.h>
#include <stdlib.h>

HAL_StatusTypeDef NEOM9N_Init(NEOM9N_HandleTypeDef *dev, UART_HandleTypeDef *huart) {
    if (dev == NULL || huart == NULL) {
        return HAL_ERROR;
    }
    
    dev->huart = huart;
    dev->rx_index = 0;
    dev->data.latitude = 0.0f;
    dev->data.longitude = 0.0f;
    dev->data.altitude_m = 0.0f;
    dev->data.satellites = 0;
    dev->data.fix_valid = false;

    return HAL_OK;
}

/* Call this function inside your UART RX Interrupt handler */
void NEOM9N_ProcessByte(NEOM9N_HandleTypeDef *dev, uint8_t byte) {
    if (byte == '\r' || byte == '\n') {
        if (dev->rx_index > 0) {
            dev->rx_buffer[dev->rx_index] = '\0'; // Null-terminate string
            NEOM9N_ParseNMEA(dev, (char *)dev->rx_buffer);
            dev->rx_index = 0; // Reset buffer pointer
        }
    } else {
        if (dev->rx_index < (NEO_M9N_NMEA_BUFFER_SIZE - 1)) {
            dev->rx_buffer[dev->rx_index++] = byte;
        } else {
            dev->rx_index = 0; // Overflow prevention
        }
    }
}

/* Basic parser for $GNGGA / $GPGGA sentences */
bool NEOM9N_ParseNMEA(NEOM9N_HandleTypeDef *dev, char *line) {
    // Check for GGA sentence ($GNGGA or $GPGGA)
    if (strncmp(line, "$GNGGA", 6) != 0 && strncmp(line, "$GPGGA", 6) != 0) {
        return false;
    }

    char *token;
    int field = 0;
    char *rest = line;

    while ((token = strtok_r(rest, ",", &rest)) != NULL) {
        field++;

        switch (field) {
            case 1: // UTC Time (hhmmss.ss)
                if (strlen(token) >= 6) {
                    // Extract HH, MM, SS
                    int hours   = (token[0] - '0') * 10 + (token[1] - '0');
                    int minutes = (token[2] - '0') * 10 + (token[3] - '0');
                    float secs  = atof(&token[4]); // Seconds + decimals
                    
                    // Convert to total seconds of the day
                    dev->data.utc_time_sec = (hours * 3600.0f) + (minutes * 60.0f) + secs;
                }
                break;
            case 2: // Latitude (DDMM.MMMMM)
                if (strlen(token) > 0) {
                    float raw_lat = atof(token);
                    int degrees = (int)(raw_lat / 100);
                    float minutes = raw_lat - (degrees * 100);
                    dev->data.latitude = degrees + (minutes / 60.0f);
                }
                break;

            case 3: // N/S Indicator
                if (token[0] == 'S') dev->data.latitude *= -1.0f;
                break;

            case 4: // Longitude (DDDMM.MMMMM)
                if (strlen(token) > 0) {
                    float raw_lon = atof(token);
                    int degrees = (int)(raw_lon / 100);
                    float minutes = raw_lon - (degrees * 100);
                    dev->data.longitude = degrees + (minutes / 60.0f);
                }
                break;

            case 5: // E/W Indicator
                if (token[0] == 'W') dev->data.longitude *= -1.0f;
                break;

            case 6: // Fix Quality (0 = Invalid, 1/2 = Valid)
                dev->data.fix_valid = (atoi(token) > 0);
                break;

            case 7: // Satellites Tracked
                dev->data.satellites = atoi(token);
                break;

            case 9: // Altitude
                dev->data.altitude_m = atof(token);
                break;

            default:
                break;
        }
    }

    return dev->data.fix_valid;
}