/*
 * uav-safe-sim - C sensor interface ABI (verified component)
 *
 * Language rationale: C is assigned here because real sensor drivers and
 * embedded HALs ship as C ABIs. This header is the cross-language boundary
 * contract: every field name carries its unit (_m, _mps, _s) and every value
 * must pass the same validation rules as src/uav/contract.py.
 *
 * Status: VERIFIED - compiled and executed with gcc in CI (see
 * components/README.md). Compatibility with the Python core is NOT assumed;
 * it is enforced by mirroring the contract table below.
 */
#ifndef UAV_SENSOR_ABI_H
#define UAV_SENSOR_ABI_H

#include <stdbool.h>

/* Health states - MUST match src/uav/health.py HealthState exactly. */
typedef enum {
    SENSOR_HEALTHY = 0,
    SENSOR_DEGRADED = 1,
    SENSOR_STALE = 2,
    SENSOR_FAILED = 3,
    SENSOR_UNKNOWN = 4
} sensor_status_t;

/* Contract limits - MUST match src/uav/contract.py. */
#define UAV_WORLD_MAX_M      10000.0   /* position plausibility bound (m)     */
#define UAV_ACCEL_MAX_MPS2    100.0    /* acceleration plausibility (m/s^2)   */
#define UAV_RANGE_MAX_M       100.0    /* depth ray plausibility (m)          */
#define UAV_GPS_TIMEOUT_S       2.0    /* GPS freshness deadline (s)          */
#define UAV_IMU_TIMEOUT_S       0.5    /* IMU freshness deadline (s)         */
#define UAV_DEPTH_TIMEOUT_S     1.0    /* depth freshness deadline (s)        */
#define UAV_CONTRACT_VERSION "contracts-v0.1"

/* GPS fix: positions in meters (world frame, ENU), sigma in meters. */
typedef struct {
    double t_s;        /* monotonic clock, seconds                          */
    double x_m;
    double y_m;
    double sigma_m;    /* reported 1-sigma; may lie under silent degradation */
} gps_fix_t;

/* IMU sample: world-frame accelerations in m/s^2 (attitude-compensated). */
typedef struct {
    double t_s;
    double ax_mps2;
    double ay_mps2;
} imu_sample_t;

/* One depth ray: angle in radians (absolute, ENU), distance in meters. */
typedef struct {
    double angle_rad;
    double distance_m;
} depth_ray_t;

/* Validation: returns false for NaN/inf/out-of-range/stale. A rejecting
 * value is DROPPED by the caller, never partially used. */
bool uav_gps_fix_valid(const gps_fix_t *fix);
bool uav_imu_sample_valid(const imu_sample_t *s);
bool uav_depth_ray_valid(const depth_ray_t *ray);

/* Freshness: age in seconds vs the contract deadline. */
bool uav_is_fresh(double age_s, double deadline_s);

#endif /* UAV_SENSOR_ABI_H */
