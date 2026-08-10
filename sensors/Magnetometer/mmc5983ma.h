#ifndef MMC5983MA_H
#define MMC5983MA_H

#include <stm32h7xx_hal.h>
#include <stdint.h>


/* =========================
 * Register Addresses
 * ========================= */

#define MMC5983MA_REG_XOUT_0        0x00
#define MMC5983MA_REG_XOUT_1        0x01

#define MMC5983MA_REG_YOUT_0        0x02
#define MMC5983MA_REG_YOUT_1        0x03

#define MMC5983MA_REG_ZOUT_0        0x04
#define MMC5983MA_REG_ZOUT_1        0x05

#define MMC5983MA_REG_XYZOUT_2      0x06

#define MMC5983MA_REG_TOUT          0x07

#define MMC5983MA_REG_STATUS        0x08

#define MMC5983MA_REG_CONTROL_0     0x09
#define MMC5983MA_REG_CONTROL_1     0x0A
#define MMC5983MA_REG_CONTROL_2     0x0B
#define MMC5983MA_REG_CONTROL_3     0x0C

#define MMC5983MA_REG_PRODUCT_ID    0x2F


/* =========================
 * SPI
 * ========================= */

#define MMC5983MA_SPI_READ          0x01
#define MMC5983MA_SPI_WRITE         0x00


/* =========================
 * Expected Device ID
 * ========================= */

#define MMC5983MA_PRODUCT_ID        0x30


/* =========================
 * Status Bits
 * ========================= */

#define MMC5983MA_STATUS_MEAS_M_DONE    (1 << 0)
#define MMC5983MA_STATUS_MEAS_T_DONE    (1 << 1)


/* =========================
 * Raw Magnetometer Data
 * ========================= */

typedef struct {
    uint32_t mag_x;
    uint32_t mag_y;
    uint32_t mag_z;

    uint32_t timestamp_milliseconds;

} MMC5983MA_RawData;


/* =========================
 * Functions
 * ========================= */

HAL_StatusTypeDef MMC5983MA_ReadRegister(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    uint8_t reg,
    uint8_t *data
);


HAL_StatusTypeDef MMC5983MA_WriteRegister(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    uint8_t reg,
    uint8_t data
);


uint8_t MMC5983MA_ProductID(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin
);


HAL_StatusTypeDef MMC5983MA_ReadMagneticField(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    MMC5983MA_RawData *data
);


#endif /* MMC5983MA_H */