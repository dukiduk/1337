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

/* Fix #1: added -- was entirely missing. PWR_MGMT0 controls whether the
   accel/gyro are actually running; after reset both are in
   standby/off, so ICM42688_ReadIMUData() would return stale/zero data
   without this. Register address and bit layout per the ICM-42688-P
   datasheet -- this is a well-documented, widely-used chip, so
   confidence here is high, but still worth a quick datasheet
   cross-check before flight. */
#define ICM42688_REG_PWR_MGMT0        0x4E
#define ICM42688_PWR_MGMT0_GYRO_LN    (0x03 << 2)  /* gyro: low-noise mode */
#define ICM42688_PWR_MGMT0_ACCEL_LN   (0x03 << 0)  /* accel: low-noise mode */



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

/* Fix #2: added -- was entirely missing. Enables the accelerometer and
   gyroscope in low-noise mode. Call once at startup, after confirming
   ICM42688_WhoAmI() matches ICM42688_WHO_AM_I_VALUE, and before the
   first call to ICM42688_ReadIMUData(). The datasheet recommends a
   short delay (~200us-1ms is more than enough margin) after this write
   before the sensors' outputs are valid -- handled inside the .c file. */
HAL_StatusTypeDef ICM42688_Init(
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *CS_Port,
    uint16_t CS_Pin
);

#endif /* ICM42688_H */
