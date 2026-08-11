#include <mmc5938ma.h>


HAL_StatusTypeDef MMC5983MA_ReadRegister(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    uint8_t reg,
    uint8_t *data
) {
    uint8_t tx[2];
    uint8_t rx[2];

    /*
     * SPI command:
     *
     * bit 0     = 1 → READ
     * bit 1     = don't care
     * bits 2-7  = register address
     */

    tx[0] = (reg << 2) | MMC5983MA_SPI_READ;

    /*
     * We send a dummy byte to generate
     * the clock cycles required to receive
     * the register contents.
     */
    tx[1] = 0x00;


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
            2,
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
     * The first received byte corresponds
     * to the command byte.
     *
     * The second received byte is the
     * register contents.
     */

    *data = rx[1];

    return HAL_OK;
}

HAL_StatusTypeDef MMC5983MA_WriteRegister(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    uint8_t reg,
    uint8_t data
) {
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = (reg << 2) | MMC5983MA_SPI_WRITE;
    tx[1] = data;

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
            2,
            HAL_MAX_DELAY
        );

    HAL_GPIO_WritePin(
        CS_Port,
        CS_Pin,
        GPIO_PIN_SET
    );

    return status;
}

uint8_t MMC5983MA_ProductID(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin
)
{
    uint8_t product_id = 0;

    if (MMC5983MA_ReadRegister(
            hspi,
            CS_Port,
            CS_Pin,
            MMC5983MA_REG_PRODUCT_ID,
            &product_id
        ) != HAL_OK)
    {
        return 0;
    }

    return product_id;
}


HAL_StatusTypeDef MMC5983MA_ReadMagneticField(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    MMC5983MA_RawData *data
)
{

    uint8_t tx[8];
    uint8_t rx[8];

    tx[0] = (MMC5983MA_REG_XOUT_0 << 2) | MMC5983MA_SPI_READ;

    for (int i = 1; i < 8; i++) {
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
            8,
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


    data->mag_x = ((uint32_t)rx[1] << 10) | ((uint32_t)rx[2] << 2) | ((rx[7] >> 6) & 0x03);

    data->mag_y = ((uint32_t)rx[3] << 10) | ((uint32_t)rx[4] << 2) | ((rx[7] >> 4) & 0x03);

    data->mag_z = ((uint32_t)rx[5] << 10) | ((uint32_t)rx[6] << 2) | ((rx[7] >> 2) & 0x03);

    data->timestamp_milliseconds = HAL_GetTick();
    
    return HAL_OK;
}

