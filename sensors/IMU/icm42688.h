#ifndef ICM42688_H
#define ICM42688_H

#include <stm32h7xx_hal.h>
#include <stdint.h>

/* =========================
 * Register Addresses
 * ========================= */

#define ICM42688_REG_TEMP_DATA1       0x1D
#define ICM42688_REG_TEMP_DATA0       0x1E

#define ICM42688_REG_ACCEL_DATA_X1    0x1F
#define ICM42688_REG_ACCEL_DATA_X0    0x20

#define ICM42688_REG_ACCEL_DATA_Y1    0x21
#define ICM42688_REG_ACCEL_DATA_Y0    0x22

#define ICM42688_REG_ACCEL_DATA_Z1    0x23
#define ICM42688_REG_ACCEL_DATA_Z0    0x24

#define ICM42688_REG_GYRO_DATA_X1     0x25
#define ICM42688_REG_GYRO_DATA_X0     0x26

#define ICM42688_REG_GYRO_DATA_Y1     0x27
#define ICM42688_REG_GYRO_DATA_Y0     0x28

#define ICM42688_REG_GYRO_DATA_Z1     0x29
#define ICM42688_REG_GYRO_DATA_Z0     0x2A

#define ICM42688_REG_INT_STATUS       0x2D

#define ICM42688_REG_WHO_AM_I         0x75




#define ICM42688_WHO_AM_I_VALUE       0x47




#define ICM42688_SPI_READ_BIT         0x80



#define ICM42688_INT_DATA_READY       (1 << 3)


/* =========================
 * Sensor Data
 * ========================= */

typedef struct
{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t gyro_roll;
    int16_t gyro_pitch;
    int16_t gyro_yaw;

    int16_t temperature;

    uint32_t timestamp_milliseconds;

} ICM42688_RawData;


/* =========================
 * Functions
 * ========================= */

HAL_StatusTypeDef ICM42688_ReadRegister(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    uint8_t reg,
    uint8_t *data
);

HAL_StatusTypeDef ICM42688_WriteRegister(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    uint8_t reg,
    uint8_t data
);

HAL_StatusTypeDef ICM42688_ReadIMUData(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin,
    ICM42688_RawData *data
);

uint8_t ICM42688_WhoAmI(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin
);

#endif /* ICM42688_H */