/* Fix #1: was #include <ms5611.h> (angle brackets -- tells the compiler
   to search system include paths first, not the local project
   directory). Quotes are the correct, standard convention for a local
   project header. */
#include "ms5611.h"


HAL_StatusTypeDef Ms5611_ReadADC(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    uint32_t *raw_value
) {
    // 4-byte buffers: byte 0 is command, bytes 1-3 are dummy bytes to clock out the 24-bit data
    uint8_t tx_buf[4] = { MS5611_CMD_ADC_READ, 0x00, 0x00, 0x00 };
    uint8_t rx_buf[4] = { 0x00, 0x00, 0x00, 0x00 };

    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        hspi,
        tx_buf,
        rx_buf,
        4,              
        HAL_MAX_DELAY
    );

    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);

    if (status == HAL_OK) {
        // Reassemble the 3 data bytes into a single 32-bit unsigned integer
        // rx_buf[0] is garbage from when we sent the 0x00 command
        *raw_value = ((uint32_t)rx_buf[1] << 16) | 
                     ((uint32_t)rx_buf[2] << 8)  | 
                      (uint32_t)rx_buf[3];
    }

    return status;
}


HAL_StatusTypeDef Ms5611_SendCommand(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    uint8_t command
) {
    
    uint8_t tx[1] = { command };
    uint8_t rx[1] = { 0x00 };

    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        hspi, 
        tx, 
        rx, 
        1, 
        HAL_MAX_DELAY
    );

    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);

    return status;
}


HAL_StatusTypeDef Ms5611_ReadData(
    SPI_HandleTypeDef *hspi, 
    GPIO_TypeDef *CS_Port, 
    uint16_t CS_Pin, 
    Ms5611_RawData *data
) {
    HAL_StatusTypeDef status;
    uint8_t tx_buf[3];
    uint8_t rx_buf[3];
    
    
    uint16_t raw_data[6];

    for (uint8_t i = 0; i < 6; i++) {
        // Generates 0xA2, 0xA4, 0xA6, 0xA8, 0xAA, 0xAC
        uint8_t cmd = MS5611_CMD_PROM_READ_C1 + (i * 2);
        
        tx_buf[0] = cmd;
        tx_buf[1] = 0x00; // Dummy byte to clock out High Data
        tx_buf[2] = 0x00; // Dummy byte to clock out Low Data

        HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);

        status = HAL_SPI_TransmitReceive(hspi, tx_buf, rx_buf, 3, HAL_MAX_DELAY);

        HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);

        if (status != HAL_OK) {
            return status;
        }

        raw_data[i] = ((uint16_t)rx_buf[1] << 8) | (uint16_t)rx_buf[2];
    }

    data->psens = raw_data[0];
    data->off = raw_data[1];
    data->tcs = raw_data[2];
    data->tco = raw_data[3];
    data->tref = raw_data[4];
    data->tsens = raw_data[5];

    data->timestamp_milliseconds = HAL_GetTick();

    return HAL_OK;
}


/* Fix #2: added -- this whole function was missing. Runs a full
   pressure + temperature conversion cycle and computes calibrated
   values using the standard MS5611 datasheet formula (first-order
   compensation; the datasheet also defines a second-order correction
   for temperatures below 20 degC that this does not implement -- add
   it if you need accuracy at low temperature).

   NOTE on timing: this function blocks for roughly
   2 x MS5611_CONVERSION_DELAY_MS (~18ms total) via HAL_Delay, since it
   has to wait for the ADC conversion to complete before reading it.
   Given the loop rates discussed for the ADRC control loops (200Hz+,
   5ms period), do NOT call this directly inside a fast control loop --
   either run it in a slower task (altitude only needs sensor updates
   on the order of the outer-loop rate, not the inner-loop rate), or
   restructure it into a non-blocking state machine that starts a
   conversion, returns immediately, and is polled/completed on a later
   call once MS5611_CONVERSION_DELAY_MS has elapsed. */
HAL_StatusTypeDef Ms5611_ReadCompensated(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    Ms5611_RawData *data
) {
    HAL_StatusTypeDef status;
    uint32_t D1_pressure = 0;
    uint32_t D2_temperature = 0;

    /* Start pressure conversion (D1), wait, read */
    status = Ms5611_SendCommand(hspi, CS_Port, CS_Pin, MS5611_CMD_CONVERT_D1_OSR4096);
    if (status != HAL_OK) return status;
    HAL_Delay(MS5611_CONVERSION_DELAY_MS);
    status = Ms5611_ReadADC(hspi, CS_Port, CS_Pin, &D1_pressure);
    if (status != HAL_OK) return status;

    /* Start temperature conversion (D2), wait, read */
    status = Ms5611_SendCommand(hspi, CS_Port, CS_Pin, MS5611_CMD_CONVERT_D2_OSR4096);
    if (status != HAL_OK) return status;
    HAL_Delay(MS5611_CONVERSION_DELAY_MS);
    status = Ms5611_ReadADC(hspi, CS_Port, CS_Pin, &D2_temperature);
    if (status != HAL_OK) return status;

    /* Standard MS5611 first-order compensation, straight from the
       datasheet, using the calibration coefficients already read into
       `data` by Ms5611_ReadData(). */
    int64_t dT = (int64_t)D2_temperature - ((int64_t)data->tref << 8);
    int64_t TEMP = 2000 + (dT * (int64_t)data->tsens) / (1LL << 23);

    int64_t OFF  = ((int64_t)data->off  << 16) + ((int64_t)data->tco * dT) / (1LL << 7);
    int64_t SENS = ((int64_t)data->psens << 15) + ((int64_t)data->tcs * dT) / (1LL << 8);

    int64_t P = (((int64_t)D1_pressure * SENS) / (1LL << 21) - OFF) / (1LL << 15);

    data->temperature = (int32_t)TEMP;   /* centi-degC, e.g. 2007 = 20.07 degC */
    data->pressure    = (int32_t)P;      /* centi-mbar, e.g. 100009 = 1000.09 mbar */
    data->timestamp_milliseconds = HAL_GetTick();

    return HAL_OK;
}
