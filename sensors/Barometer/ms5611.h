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

/* Max conversion time in ms for OSR4096, per datasheet (~8.22ms worst
   case) -- rounded up with margin. Lower OSR settings convert faster;
   this constant is conservative so it's safe regardless of which
   MS5611_CMD_CONVERT_* command you actually use. */
#define MS5611_CONVERSION_DELAY_MS    9

typedef struct {
    uint16_t psens; 
    uint16_t off;   
    uint16_t tcs;   
    uint16_t tco;   
    uint16_t tref;  
    uint16_t tsens;
    
    /* Fix #1: pressure/temperature were declared here but nothing ever
       computed them -- only the raw PROM calibration words were
       populated. These now get filled in by MS5611_ReadCompensated(). */
    int32_t pressure;      /* mbar * 100 (i.e. centi-mbar) */
    int32_t temperature;   /* degC * 100 (i.e. centi-degC) */
    
    uint32_t timestamp_milliseconds;
} Ms5611_RawData;

/* Fix #2: names below now match ms5611.c exactly (case-sensitive --
   MS5611_SendCommand/MS5611_ReadADC(6-arg) as originally declared here
   never matched anything actually defined in the .c file, which would
   have failed at LINK time, not compile time, the moment anything
   called them). */

HAL_StatusTypeDef Ms5611_SendCommand(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    uint8_t command
);

HAL_StatusTypeDef Ms5611_ReadADC(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    uint32_t *raw_value
);

HAL_StatusTypeDef Ms5611_ReadData(
    SPI_HandleTypeDef *hspi, 
    GPIO_TypeDef *CS_Port, 
    uint16_t CS_Pin, 
    Ms5611_RawData *data
);

/* Fix #3: added -- was entirely missing. Triggers a D1 (pressure) and
   D2 (temperature) conversion, waits the required conversion time, and
   computes calibrated pressure/temperature per the datasheet's standard
   compensation formula, using coefficients already read into `data` by
   Ms5611_ReadData(). Call Ms5611_ReadData() once at startup to populate
   the calibration coefficients, then call this each time you want a
   fresh reading -- it uses HAL_Delay, so it blocks for ~2x
   MS5611_CONVERSION_DELAY_MS per call; see the note in the .c file if
   that's a problem for your loop timing. */
HAL_StatusTypeDef Ms5611_ReadCompensated(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    Ms5611_RawData *data
);

#endif /* MS5611_H */
