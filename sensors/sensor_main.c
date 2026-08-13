#include <plane_state.h>
#include <Barometer/ms5611.h>
#include <Magnetometer/mmc5893ma.h>
#include <IMU/icm42688.h>
#include <GNSS/neom9n.h>

void SensorMain_Run(void)
{
    ICM42688_RawData imu;
    MMC5983MA_RawData mag;
    Ms5611_RawData baro;
    GPS_Data gps;

    HAL_StatusTypeDef imu_status;
    HAL_StatusTypeDef mag_status;
    HAL_StatusTypeDef baro_status;
    HAL_StatusTypeDef gps_status;


    imu_status =
        ICM42688_ReadRaw(
            &hspi1,
            ICM_CS_GPIO_Port,
            ICM_CS_Pin,
            &imu
        );

    mag_status =
        MMC5983MA_ReadMagneticField(
            &hspi2,
            MAG_CS_GPIO_Port,
            MAG_CS_Pin,
            &mag
        );

    baro_status =
        MS5611_ReadRaw(
            &hi2c1,
            &baro
        );

    gps_status =
        GPS_Read(
            &gps
        );


    /*
     * 2. Process successful measurements
     */

    if (baro_status == HAL_OK)
    {
        process_barometer(&baro);
    }

    if (mag_status == HAL_OK)
    {
        process_magnetometer(&mag);
    }


    /*
     * 3. Predict using IMU
     */

    if (imu_status == HAL_OK)
    {
        process_imu_and_predict(&imu);
    }


    /*
     * 4. Correct using other sensors
     */

    if (baro_status == HAL_OK)
    {
        update_ekf_barometer(...);
    }

    if (gps_status == HAL_OK)
    {
        update_ekf_gps(...);
    }

    if (mag_status == HAL_OK)
    {
        update_ekf_magnetometer(...);
    }


    /*
     * 5. Copy EKF state into global aircraft state
     */

    update_aircraft_pose();
}