#ifndef MS5611_H
#define MS5611_H

#include <stm32h7xx_hal.h>
#include <stdint.h>

#define MS5611_CMD_RESET              0x1E
#define MS5611_CMD_ADC_READ           0x00

#define MS5611_CMD_CONVERT_D1_OSR256  0x40
#define MS5611_CMD_CONVERT_D1_OSR512  0x42
#define MS5611_CMD_CONVERT_D1_OSR1024 0x44
#define MS5611_CMD_CONVERT_D1_OSR2048 0x46
#define MS5611_CMD_CONVERT_D1_OSR4096 0x48 

#define MS5611_CMD_CONVERT_D2_OSR256  0x50
#define MS5611_CMD_CONVERT_D2_OSR512  0x52
#define MS5611_CMD_CONVERT_D2_OSR1024 0x54
#define MS5611_CMD_CONVERT_D2_OSR2048 0x56
#define MS5611_CMD_CONVERT_D2_OSR4096 0x58 

#define MS5611_CMD_PROM_READ_BASE     0xA0
#define MS5611_CMD_PROM_READ_C1       0xA2 
#define MS5611_CMD_PROM_READ_C2       0xA4 
#define MS5611_CMD_PROM_READ_C3       0xA6 
#define MS5611_CMD_PROM_READ_C4       0xA8 
#define MS5611_CMD_PROM_READ_C5       0xAA 
#define MS5611_CMD_PROM_READ_C6       0xAC 

typedef struct {
    uint16_t psens; 
    uint16_t off;   
    uint16_t tcs;   
    uint16_t tco;   
    uint16_t tref;  
    uint16_t tsens; 
    
    uint32_t timestamp_milliseconds;
} Ms5611_RawData;

HAL_StatusTypeDef Ms5611_ReadADC(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    uint32_t *raw_value
);

HAL_StatusTypeDef MS5611_SendCommand(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    uint8_t command
);

HAL_StatusTypeDef MS5611_ReadPROM(
    SPI_HandleTypeDef *hspi, 
    GPIO_TypeDef *CS_Port, 
    uint16_t CS_Pin, 
    Ms5611_RawData *data
);

#endif /* MS5611_H */