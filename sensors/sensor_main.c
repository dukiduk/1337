
#include <stdint.h>

// Size of state space
#define EKF_N 16

// Size of observation (measurement) space
#define EKF_M 10

#include <tinyekf.h>

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t temperature;

} IMU_RawData;

// Volatile flag flipped by a hardware timer interrupt every 10ms (100Hz)
volatile uint8_t sensor_loop_trigger = 0;

int main(void) {
    // 1. Initialize System Clock (480 MHz), GPIO, SPI, I2C, UART, and Timers
    HAL_Init();
}


