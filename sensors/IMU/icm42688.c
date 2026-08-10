#include <icm42688.h>


HAL_StatusTypeDef ICM42688_ReadRegister(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    uint8_t reg,
    uint8_t *data
) {
    uint8_t tx = reg | ICM42688_SPI_READ_BIT;
    uint8_t rx = 0;

    HAL_GPIO_WritePin(
        CS_Port,
        CS_Pin,
        GPIO_PIN_RESET
    );

    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(
            hspi,
            &tx,
            &rx,
            1,
            HAL_MAX_DELAY
        );

    if (status != HAL_OK)
    {
        HAL_GPIO_WritePin(
            CS_Port,
            CS_Pin,
            GPIO_PIN_SET
        );

        return status;
    }

    status =
        HAL_SPI_Receive(
            hspi,
            data,
            1,
            HAL_MAX_DELAY
        );

    HAL_GPIO_WritePin(
        CS_Port,
        CS_Pin,
        GPIO_PIN_SET
    );

    return status;
}


uint8_t ICM42688_WhoAmI(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin
) {
    uint8_t who_am_i = 0;

    if (ICM42688_ReadRegister(
            hspi,
            CS_Port,
            CS_Pin,
            ICM42688_REG_WHO_AM_I,
            &who_am_i
        ) != HAL_OK)
    {
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
    // Bit 7 = 0 for Write on ICM42688
    uint8_t tx[2] = { reg & ~ICM42688_SPI_READ_BIT, data };
    uint8_t rx[2];

    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_RESET);

    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(
        hspi, 
        tx, 
        rx, 
        2, 
        HAL_MAX_DELAY
    );

    HAL_GPIO_WritePin(CS_Port, CS_Pin, GPIO_PIN_SET);

    return status;
}


HAL_StatusTypeDef ICM42688_ReadIMUData(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    ICM42688_RawData *data
) {
    uint8_t tx[15];
    uint8_t rx[15];

    /*
     * Start reading at ACCEL_DATA_X1 (0x1F).
     *
     * We need:
     *
     * 0x1F -> 0x2A
     *
     * That's 12 data bytes.
     *
     * The first transmitted byte is the
     * register address, so we transmit
     * 13 bytes total.
     */

    tx[0] =
        ICM42688_REG_TEMP_DATA1 |
        ICM42688_SPI_READ_BIT;

    for (int i = 1; i < 13; i++) {
        tx[i] = 0x00;
    }

    HAL_GPIO_WritePin(
        CS_Port,
        CS_Pin,
        GPIO_PIN_RESET
    );

    HAL_StatusTypeDef status =
        HAL_SPI_TransmitReceive(
            hspi,
            tx,
            rx,
            15,
            HAL_MAX_DELAY
        );

    HAL_GPIO_WritePin(
        CS_Port,
        CS_Pin,
        GPIO_PIN_SET
    );

    if (status != HAL_OK)
    {
        return status;
    }


    /*
     * rx[0] is the response to the
     * register-address byte.
     *
     * Actual sensor data begins at rx[1].
     */

    data->temperature = (int16_t)((rx[1] << 8) | rx[2]);

    data->accel_x = (int16_t)((rx[3] << 8) | rx[4]);

    data->accel_y = (int16_t)((rx[5] << 8) | rx[6]);

    data->accel_z = (int16_t)((rx[7] << 8) | rx[8]);

    data->gyro_roll =  (int16_t)((rx[9] << 8) | rx[10]);

    data->gyro_pitch =  (int16_t)((rx[11] << 8) | rx[12]);

    data->gyro_yaw = (int16_t)((rx[13] << 8) | rx[14]);

    data->timestamp_milliseconds = HAL.GetTick();


    return HAL_OK;
}