#ifndef EKF_INTERFACE_H
#define EKF_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>
#include "icm42688.h"
#include "mmc5983ma.h"
#include "ms5611.h"
#include "neom9n.h"

#ifdef __cplusplus
extern "C" {
#endif

void process_imu_and_predict(ICM42688_RawData *imu);
void update_ekf_barometer(uint32_t raw_adc, Ms5611_RawData *baro);
void update_ekf_magnetometer(MMC5983MA_RawData *mag);
void update_ekf_gps(NEOM9N_GPSData *gps);
void update_aircraft_pose(void);
void update_ekf_attitude(ICM42688_RawData *imu);

#ifdef __cplusplus
}
#endif

#endif /* EKF_INTERFACE_H */
