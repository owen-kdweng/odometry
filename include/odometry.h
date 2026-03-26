#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* Configuration */
    double wheel_radius_m;         /* wheel radius in meters */
    double wheel_base_m;           /* distance between left/right wheels in meters */
    double ticks_per_revolution;   /* encoder ticks per wheel revolution */

    int64_t encoder_min;           /* encoder minimum raw value */
    int64_t encoder_max;           /* encoder maximum raw value */

    bool left_inverted;            /* true if left encoder direction is reversed */
    bool right_inverted;           /* true if right encoder direction is reversed */

    /* State */
    double x_m;                    /* position x in meters */
    double y_m;                    /* position y in meters */
    double theta_rad;              /* heading in radians */

    double linear_velocity_mps;    /* linear velocity in m/s */
    double angular_velocity_rps;   /* angular velocity in rad/s */

    int32_t prev_left_tick;
    int32_t prev_right_tick;
    bool has_prev_tick;
} odometry_t;

/* Return codes */
typedef enum
{
    ODOM_OK = 0,
    ODOM_ERR_NULL = -1,
    ODOM_ERR_INVALID_PARAM = -2,
    ODOM_ERR_INVALID_DT = -3,
    ODOM_ERR_TICK_OUT_OF_RANGE = -4
} odom_status_t;

/**
 * Initialize odometry object.
 *
 * @param odom                  Pointer to odometry object
 * @param wheel_radius_m        Wheel radius in meters
 * @param wheel_base_m          Distance between wheels in meters
 * @param ticks_per_revolution  Encoder ticks per wheel revolution
 * @param encoder_min           Encoder minimum raw value
 * @param encoder_max           Encoder maximum raw value
 * @param left_inverted         Reverse left encoder direction
 * @param right_inverted        Reverse right encoder direction
 */
odom_status_t odom_init(odometry_t *odom,
                        double wheel_radius_m,
                        double wheel_base_m,
                        double ticks_per_revolution,
                        int64_t encoder_min,
                        int64_t encoder_max,
                        bool left_inverted,
                        bool right_inverted);

/**
 * Reset everything:
 * - pose = (0, 0, 0)
 * - velocity = 0
 * - tick baseline cleared
 */
odom_status_t odom_reset(odometry_t *odom);

/**
 * Reset robot pose only.
 */
odom_status_t odom_reset_pose(odometry_t *odom,
                              double x_m,
                              double y_m,
                              double theta_rad);

/**
 * Reset internal tick reference.
 * After calling this, the next odom_update() will only store baseline ticks.
 */
odom_status_t odom_reset_ticks(odometry_t *odom);

/**
 * Update odometry using absolute encoder ticks and dt.
 *
 * Notes:
 * - left_tick/right_tick are raw encoder counts, not delta.
 * - first call only stores tick baseline and returns ODOM_OK.
 * - first tick may be non-zero and still works correctly.
 */
odom_status_t odom_update(odometry_t *odom,
                          int32_t left_tick,
                          int32_t right_tick,
                          double dt_sec);

/**
 * Get current pose.
 * Any output pointer may be NULL if not needed.
 */
odom_status_t odom_get_pose(const odometry_t *odom,
                            double *x_m,
                            double *y_m,
                            double *theta_rad);

/**
 * Get current velocity.
 * Any output pointer may be NULL if not needed.
 */
odom_status_t odom_get_velocity(const odometry_t *odom,
                                double *linear_velocity_mps,
                                double *angular_velocity_rps);

/**
 * Normalize angle to [-pi, pi).
 */
double odom_normalize_angle(double angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* ODOMETRY_H */
