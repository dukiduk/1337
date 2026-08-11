#include <ms5611.h>


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