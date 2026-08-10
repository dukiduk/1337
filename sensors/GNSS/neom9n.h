#ifndef NEO_M9N_H
#define NEO_M9N_H

#include <stdint.h>
#include <stdbool.h>

/* Choose target MCU HAL (e.g., STM32) */
#include "stm32h7xx_hal.h" 

#define NEO_M9N_NMEA_BUFFER_SIZE  128

typedef struct {
    float latitude;         // Degrees (positive = North, negative = South)
    float longitude;        // Degrees (positive = East, negative = West)
    float altitude_m;       // Altitude above sea level in meters
    uint8_t satellites;     // Number of connected satellites
    bool fix_valid;         // True if GNSS fix is acquired
    uint32_t timestamp_seconds;
} NEOM9N_GPSData;

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_buffer[NEO_M9N_NMEA_BUFFER_SIZE];
    uint16_t rx_index;
    NEOM9N_GPSData data;
} NEOM9N_HandleTypeDef;

/* Function Prototypes */
HAL_StatusTypeDef NEOM9N_Init(
    NEOM9N_HandleTypeDef *dev,
    UART_HandleTypeDef *huart
);

void NEOM9N_ProcessByte(
    NEOM9N_HandleTypeDef *dev,
    uint8_t byte
);

bool NEOM9N_ParseNMEA(
    NEOM9N_HandleTypeDef *dev, 
    char *line
);

#endif /* NEO_M9N_H */