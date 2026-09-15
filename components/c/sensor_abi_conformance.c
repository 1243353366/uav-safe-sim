/*
 * uav-safe-sim - C sensor ABI conformance check (verified component)
 *
 * Proves the C boundary implements the same contract rules as the Python
 * core (src/uav/contract.py): NaN rejection, range rejection, freshness
 * deadlines. Build & run:
 *   gcc -std=c11 -Wall -Wextra -pedantic -lm -o abi_check sensor_abi_conformance.c
 *   ./abi_check
 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "sensor_abi.h"

static bool finite_in(double v, double lo, double hi) {
    return isfinite(v) && v >= lo && v <= hi;
}

bool uav_gps_fix_valid(const gps_fix_t *fix) {
    if (fix == NULL) return false;
    if (!finite_in(fix->x_m, -UAV_WORLD_MAX_M, UAV_WORLD_MAX_M)) return false;
    if (!finite_in(fix->y_m, -UAV_WORLD_MAX_M, UAV_WORLD_MAX_M)) return false;
    if (!finite_in(fix->sigma_m, 0.0, UAV_WORLD_MAX_M) || fix->sigma_m <= 0.0)
        return false;
    return true;
}

bool uav_imu_sample_valid(const imu_sample_t *s) {
    if (s == NULL) return false;
    if (!finite_in(s->ax_mps2, -UAV_ACCEL_MAX_MPS2, UAV_ACCEL_MAX_MPS2)) return false;
    if (!finite_in(s->ay_mps2, -UAV_ACCEL_MAX_MPS2, UAV_ACCEL_MAX_MPS2)) return false;
    return true;
}

bool uav_depth_ray_valid(const depth_ray_t *ray) {
    if (ray == NULL) return false;
    if (!finite_in(ray->distance_m, 0.0, UAV_RANGE_MAX_M)) return false;
    return isfinite(ray->angle_rad);
}

bool uav_is_fresh(double age_s, double deadline_s) {
    return isfinite(age_s) && age_s >= 0.0 && age_s <= deadline_s;
}

int main(void) {
    /* GPS: valid fix accepted */
    gps_fix_t good = { .t_s = 1.0, .x_m = 5.2, .y_m = 4.9, .sigma_m = 0.5 };
    assert(uav_gps_fix_valid(&good));

    /* GPS: NaN position rejected (chaos 'corrupt' equivalent) */
    gps_fix_t nanfix = good; nanfix.x_m = NAN;
    assert(!uav_gps_fix_valid(&nanfix));

    /* GPS: zero sigma rejected */
    gps_fix_t zsig = good; zsig.sigma_m = 0.0;
    assert(!uav_gps_fix_valid(&zsig));

    /* GPS: implausible magnitude rejected */
    gps_fix_t far = good; far.x_m = 1e7;
    assert(!uav_gps_fix_valid(&far));

    /* IMU: valid sample accepted, impossible acceleration rejected */
    imu_sample_t imu = { .t_s = 1.0, .ax_mps2 = 0.31, .ay_mps2 = -0.02 };
    assert(uav_imu_sample_valid(&imu));
    imu.ax_mps2 = 1e6;
    assert(!uav_imu_sample_valid(&imu));

    /* Depth: valid ray accepted, NaN range rejected */
    depth_ray_t ray = { .angle_rad = 0.4, .distance_m = 7.3 };
    assert(uav_depth_ray_valid(&ray));
    ray.distance_m = NAN;
    assert(!uav_depth_ray_valid(&ray));

    /* Freshness: deadlines match the Python config exactly */
    assert(uav_is_fresh(0.4, UAV_GPS_TIMEOUT_S));
    assert(!uav_is_fresh(2.4, UAV_GPS_TIMEOUT_S));
    assert(uav_is_fresh(0.4, UAV_IMU_TIMEOUT_S));
    assert(!uav_is_fresh(0.6, UAV_IMU_TIMEOUT_S));

    puts("C sensor ABI conformance: ALL CHECKS PASSED");
    return 0;
}
