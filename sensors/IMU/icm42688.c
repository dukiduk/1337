#include "icm42688.h"

HAL_StatusTypeDef ICM42688_ReadRegister(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    uint8_t reg,
    uint8_t *data
) {
    uint8_t tx[2] = { reg | ICM42688_SPI_READ_BIT, 0x00 };
    uint8_t rx[2] = { 0 };

    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);

    if (status == HAL_OK) {
        *data = rx[1];
    }

    return status;
}

uint8_t ICM42688_WhoAmI(SPI_HandleTypeDef *hspi, GPIO_TypeDef *CS_Port, uint16_t CS_Pin) {
    uint8_t who_am_i = 0;
    if (ICM42688_ReadRegister(hspi, CS_Port, CS_Pin, ICM42688_REG_WHO_AM_I, &who_am_i) != HAL_OK) {
        return 0;
    }
    return who_am_i;
}

HAL_StatusTypeDef ICM42688_WriteRegister(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    uint8_t reg,
    uint8_t data
) {
    uint8_t tx[2] = { reg & ~ICM42688_SPI_READ_BIT, data };
    uint8_t rx[2];

    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);

    return status;
}

HAL_StatusTypeDef ICM42688_ReadIMUData(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    ICM42688_RawData *data
) {
    uint8_t tx[15] = {0}; 
    uint8_t rx[15] = {0};

    tx[0] = ICM42688_REG_TEMP_DATA1 | ICM42688_SPI_READ_BIT;

    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx, rx, 15, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);

    if (status != HAL_OK) {
        return status;
    }

    int16_t raw_data[7];

    for (uint8_t i = 0; i < 7; i++) {
        uint8_t high_byte_idx = 1 + (i * 2);
        uint8_t low_byte_idx  = 2 + (i * 2);
        
        raw_data[i] = (int16_t)((rx[high_byte_idx] << 8) | rx[low_byte_idx]);
    }

    data->temperature            = raw_data[0];
    data->accel_x                = raw_data[1];
    data->accel_y                = raw_data[2];
    data->accel_z                = raw_data[3];
    data->gyro_roll              = raw_data[4];
    data->gyro_pitch             = raw_data[5];
    data->gyro_yaw               = raw_data[6];
    data->timestamp_milliseconds = HAL_GetTick();

    return HAL_OK;
}