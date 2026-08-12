#ifndef NEO_M9N_H
#define NEO_M9N_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h" 

#define NEO_M9N_NMEA_BUFFER_SIZE  128

typedef struct {
    float latitude;         
    float longitude;        
    float altitude_m;       
    uint8_t satellites;     
    bool fix_valid;         
    float utc_time_sec;
} NEOM9N_GPSData;

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_buffer[NEO_M9N_NMEA_BUFFER_SIZE];
    uint16_t rx_index;
    NEOM9N_GPSData data;
} NEOM9N_HandleTypeDef;

HAL_StatusTypeDef NEOM9N_Init(NEOM9N_HandleTypeDef *dev, UART_HandleTypeDef *huart);
void NEOM9N_ProcessByte(NEOM9N_HandleTypeDef *dev, uint8_t byte);
bool NEOM9N_ParseNMEA(NEOM9N_HandleTypeDef *dev, char *line);

#endif /* NEO_M9N_H */
