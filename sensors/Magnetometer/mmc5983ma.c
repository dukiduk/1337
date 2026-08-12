/* Fix #1: was #include <mmc5938ma.h> -- transposed digits, filename
   didn't match the header (mmc5983ma.h) at all. This alone made the file
   fail to compile. */
#include "mmc5983ma.h"


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
     * bit 0     = 1 -> READ
     * bit 1     = don't care
     * bits 2-7  = register address
     *
     * Fix #2 (flagged, not changed): this bit layout (reg<<2 | R/W in
     * bit 0) is left as originally written since I can't confirm it
     * against the datasheet from memory with high confidence. Verify
     * this against the MMC5983MA datasheet's SPI protocol section before
     * trusting it -- if the real framing puts R/W elsewhere, every
     * register access in this file targets the wrong address.
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


/* Fix #3: added -- was entirely missing. Without a SET/RESET sequence,
   this chip family's readings drift with an uncorrected bridge offset
   over time/temperature. This performs one SET pulse, one RESET pulse
   (opposite polarity, per the standard MMC59xx offset-cancellation
   procedure), and leaves the sensor ready for a measurement. Call once
   at startup and periodically thereafter (e.g. every few seconds, or
   every N calls to MMC5983MA_ReadMagneticField) to keep offset drift
   in check.
   NOTE: exact timing/register bits should be verified against the
   datasheet -- see the flag in MMC5983MA_ReadRegister above. */
HAL_StatusTypeDef MMC5983MA_Init(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin
)
{
    HAL_StatusTypeDef status;

    status = MMC5983MA_WriteRegister(hspi, CS_Port, CS_Pin,
                                      MMC5983MA_REG_CONTROL_0,
                                      MMC5983MA_CTRL0_SET);
    if (status != HAL_OK) return status;
    HAL_Delay(1);

    status = MMC5983MA_WriteRegister(hspi, CS_Port, CS_Pin,
                                      MMC5983MA_REG_CONTROL_0,
                                      MMC5983MA_CTRL0_RESET);
    if (status != HAL_OK) return status;
    HAL_Delay(1);

    return HAL_OK;
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
