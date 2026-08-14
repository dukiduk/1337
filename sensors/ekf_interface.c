/*
 * TinyEKF-based implementation for a minimal 3-state filter:
 *   state: [altitude_m, vertical_velocity_mps, yaw_deg]
 *   measurements: [altitude_m, yaw_deg]
 *
 * This keeps predict simple (identity + process noise) and performs
 * measurement updates from the barometer, magnetometer, and GPS.
 */

#include "ekf_interface.h"

#define EKF_N 6
#define EKF_M 6
#include "../tinyekf_src/tinyekf.h"

#include <math.h>

static ekf_t ekf;
static bool ekf_initialized = false;

static void ensure_initialized(void)
{
    if (ekf_initialized) return;
    _float_t pdiag[EKF_N];
    /* x, y, z, roll, pitch, yaw */
    pdiag[0] = 1000.0f; /* x (m^2) */
    pdiag[1] = 1000.0f; /* y (m^2) */
    pdiag[2] = 100.0f;  /* z (m^2) */
    pdiag[3] = 100.0f;  /* roll (deg^2) */
    pdiag[4] = 100.0f;  /* pitch (deg^2) */
    pdiag[5] = 100.0f;  /* yaw (deg^2) */
    ekf_initialize(&ekf, pdiag);
    ekf_initialized = true;
}

void process_imu_and_predict(ICM42688_RawData *imu)
{
    (void)imu; /* prediction currently identity; IMU-based predict can be added */
    ensure_initialized();
    /* fx = identity (no state change), F = I, Q = diagonal */
    _float_t fx[EKF_N];
    _float_t F[EKF_N*EKF_N];
    _float_t Q[EKF_N*EKF_N];

    for (int i = 0; i < EKF_N; ++i) fx[i] = ekf.x[i];
    for (int i = 0; i < EKF_N*EKF_N; ++i) F[i] = 0.0f;
    for (int i = 0; i < EKF_N; ++i) F[i*EKF_N + i] = 1.0f;
    for (int i = 0; i < EKF_N*EKF_N; ++i) Q[i] = 0.0f;
    Q[0*EKF_N + 0] = 0.1f;
    Q[1*EKF_N + 1] = 0.1f;
    Q[2*EKF_N + 2] = 0.1f;
    Q[3*EKF_N + 3] = 0.5f;
    Q[4*EKF_N + 4] = 0.5f;
    Q[5*EKF_N + 5] = 0.5f;

    ekf_predict(&ekf, fx, F, Q);
}

static float baro_pressure_to_altitude_m(const Ms5611_RawData *baro)
{
    /* baro->pressure is centi-mbar (mbar * 100) */
    float pressure_mbar = (float)baro->pressure / 100.0f;
    const float sea_level_mbar = 1013.25f;
    return 44330.0f * (1.0f - powf(pressure_mbar / sea_level_mbar, 0.190294957f));
}

void update_ekf_barometer(uint32_t raw_adc, Ms5611_RawData *baro)
{
    (void)raw_adc;
    ensure_initialized();

     /* Full 6-element measurement vector: [x, y, z, roll, pitch, yaw]
         For barometer we only update z. Set large variances for others. */
     _float_t z[EKF_M];
     _float_t hx[EKF_M];
     _float_t H[EKF_M*EKF_N];
     _float_t R[EKF_M*EKF_M];

     float alt_m = baro ? baro_pressure_to_altitude_m(baro) : 0.0f;

     for (int i = 0; i < EKF_M; ++i) z[i] = hx[i] = 0.0f;
     for (int i = 0; i < EKF_M*EKF_N; ++i) H[i] = 0.0f;
     for (int i = 0; i < EKF_M*EKF_M; ++i) R[i] = 0.0f;

     z[2] = alt_m;

     /* predicted measurement hx: x,y,z,roll,pitch,yaw = ekf.x[0..5] */
     for (int i = 0; i < EKF_M; ++i) hx[i] = ekf.x[i];

     /* H = identity */
     for (int i = 0; i < EKF_M; ++i) H[i*EKF_N + i] = 1.0f;

     /* R: large variances except for z */
     const float large = 1e6f;
     for (int i = 0; i < EKF_M; ++i) for (int j = 0; j < EKF_M; ++j) R[i*EKF_M + j] = 0.0f;
     R[0*EKF_M + 0] = large;
     R[1*EKF_M + 1] = large;
     R[2*EKF_M + 2] = 25.0f; /* z variance (m^2) */
     R[3*EKF_M + 3] = large;
     R[4*EKF_M + 4] = large;
     R[5*EKF_M + 5] = large;

     ekf_update(&ekf, z, hx, H, R);
}

void update_ekf_magnetometer(MMC5983MA_RawData *mag)
{
    ensure_initialized();

    /* compute 2D heading (degrees) from mag X/Y
       Note: not tilt-compensated */
    float mx = (float)mag->mag_x;
    float my = (float)mag->mag_y;
    float heading_rad = atan2f(my, mx);
    float heading_deg = heading_rad * (180.0f / (float)M_PI);
    if (heading_deg < 0.0f) heading_deg += 360.0f;

    _float_t z[EKF_M];
    _float_t hx[EKF_M];
    _float_t H[EKF_M*EKF_N];
    _float_t R[EKF_M*EKF_M];

    for (int i = 0; i < EKF_M; ++i) z[i] = hx[i] = 0.0f;
    for (int i = 0; i < EKF_M*EKF_N; ++i) H[i] = 0.0f;
    for (int i = 0; i < EKF_M*EKF_M; ++i) R[i] = 0.0f;

    /* only yaw measured here */
    z[5] = heading_deg;

    for (int i = 0; i < EKF_M; ++i) hx[i] = ekf.x[i];
    for (int i = 0; i < EKF_M; ++i) H[i*EKF_N + i] = 1.0f;

    const float large = 1e6f;
    R[0*EKF_M + 0] = large;
    R[1*EKF_M + 1] = large;
    R[2*EKF_M + 2] = large;
    R[3*EKF_M + 3] = large;
    R[4*EKF_M + 4] = large;
    R[5*EKF_M + 5] = 25.0f; /* yaw variance */

    ekf_update(&ekf, z, hx, H, R);
}

void update_ekf_gps(NEOM9N_GPSData *gps)
{
    ensure_initialized();

    if (!gps->fix_valid) return;

    _float_t z[EKF_M];
    _float_t hx[EKF_M];
    _float_t H[EKF_M*EKF_N];
    _float_t R[EKF_M*EKF_M];

    for (int i = 0; i < EKF_M; ++i) z[i] = hx[i] = 0.0f;
    for (int i = 0; i < EKF_M*EKF_N; ++i) H[i] = 0.0f;
    for (int i = 0; i < EKF_M*EKF_M; ++i) R[i] = 0.0f;

    /* Convert lat/lon to local meters relative to first fix */
    static double origin_lat = 0.0;
    static double origin_lon = 0.0;
    static bool origin_set = false;
    if (!origin_set) {
        origin_lat = gps->latitude;
        origin_lon = gps->longitude;
        origin_set = true;
    }

    double dlat = gps->latitude - origin_lat;
    double dlon = gps->longitude - origin_lon;
    const double meters_per_deg_lat = 111132.92;
    const double meters_per_deg_lon = 111319.0 * cos(origin_lat * M_PI / 180.0);

    float x_m = (float)(dlon * meters_per_deg_lon);
    float y_m = (float)(dlat * meters_per_deg_lat);
    float z_m = gps->altitude_m;

    z[0] = x_m;
    z[1] = y_m;
    z[2] = z_m;

    for (int i = 0; i < EKF_M; ++i) hx[i] = ekf.x[i];
    for (int i = 0; i < EKF_M; ++i) H[i*EKF_N + i] = 1.0f;

    const float large = 1e6f;
    R[0*EKF_M + 0] = 25.0f; /* x var (m^2) */
    R[1*EKF_M + 1] = 25.0f; /* y var */
    R[2*EKF_M + 2] = 16.0f; /* z var */
    R[3*EKF_M + 3] = large;
    R[4*EKF_M + 4] = large;
    R[5*EKF_M + 5] = large;

    ekf_update(&ekf, z, hx, H, R);
}

void update_aircraft_pose(void)
{
    /* No-op for now; downstream code can read ekf.x[] if needed. */
}

void update_ekf_attitude(ICM42688_RawData *imu)
{
    ensure_initialized();

    /* Compute roll/pitch from accelerometer (static assumption) */
    float ax = (float)imu->accel_x;
    float ay = (float)imu->accel_y;
    float az = (float)imu->accel_z;

    float roll = atan2f(ay, az) * (180.0f / (float)M_PI);
    float pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * (180.0f / (float)M_PI);

    _float_t z[EKF_M];
    _float_t hx[EKF_M];
    _float_t H[EKF_M*EKF_N];
    _float_t R[EKF_M*EKF_M];

    for (int i = 0; i < EKF_M; ++i) z[i] = hx[i] = 0.0f;
    for (int i = 0; i < EKF_M*EKF_N; ++i) H[i] = 0.0f;
    for (int i = 0; i < EKF_M*EKF_M; ++i) R[i] = 0.0f;

    z[3] = roll;
    z[4] = pitch;

    for (int i = 0; i < EKF_M; ++i) hx[i] = ekf.x[i];
    for (int i = 0; i < EKF_M; ++i) H[i*EKF_N + i] = 1.0f;

    const float large = 1e6f;
    R[0*EKF_M + 0] = large;
    R[1*EKF_M + 1] = large;
    R[2*EKF_M + 2] = large;
    R[3*EKF_M + 3] = 9.0f;   /* roll variance (deg^2) */
    R[4*EKF_M + 4] = 9.0f;   /* pitch variance */
    R[5*EKF_M + 5] = large;

    ekf_update(&ekf, z, hx, H, R);
}
