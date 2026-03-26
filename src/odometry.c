#include "odometry.h"

#include <math.h>
#include <stddef.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



/**
 * @brief Check whether a tick value is inside the configured encoder range.
 *
 * This helper validates that a raw encoder tick value is within the legal
 * inclusive range [encoder_min, encoder_max].
 *
 * @param tick The encoder tick value to validate.
 * @param encoder_min The minimum valid encoder tick value.
 * @param encoder_max The maximum valid encoder tick value.
 * @return true if the tick is within range, otherwise false.
 */
static bool odom_tick_in_range(int32_t tick, int64_t encoder_min, int64_t encoder_max)
{
    return ((int64_t)tick >= encoder_min && (int64_t)tick <= encoder_max);
}



/**
 * @brief Compute the signed tick difference while handling encoder wraparound.
 *
 * This helper calculates the shortest signed difference between the current
 * tick and the previous tick for an encoder that wraps within an arbitrary
 * integer range [encoder_min, encoder_max].
 *
 * For example, if the encoder range is [0, 65535] and the previous tick is
 * 65530 while the current tick is 5, the raw subtraction gives a large
 * negative value. This function corrects it to the smallest equivalent
 * positive difference across the wrap boundary.
 *
 * The same logic also works for signed encoder ranges such as
 * [-32768, 32767].
 *
 * @param current The current encoder tick value.
 * @param previous The previous encoder tick value.
 * @param encoder_min The minimum value of the encoder range.
 * @param encoder_max The maximum value of the encoder range.
 * @return The shortest signed tick difference after wraparound correction.
 */
static int32_t odom_tick_diff_with_wrap(int32_t current,
                                        int32_t previous,
                                        int64_t encoder_min,
                                        int64_t encoder_max)
{
    /*
     * Support arbitrary encoder range:
     *   [encoder_min, encoder_max]
     *
     * Example:
     *   encoder_min = -32768, encoder_max = 32767
     *   range = 65536
     */

    const int64_t range = encoder_max - encoder_min + 1;
    const int64_t half_range = range / 2;

    int32_t diff = current - previous;

    /*
     * Choose shortest signed difference across wraparound.
     *
     * Example for [0, 65535]:
     * previous = 65530, current = 5
     * raw diff = -65525 -> corrected to +11
     *
     * Example for [-32768, 32767]:
     * previous = 32760, current = -32760
     * raw diff = -65520 -> corrected to +16
     */
    if (diff > half_range)
    {
        diff -= range;
    }
    else if (diff < -half_range)
    {
        diff += range;
    }

    return diff;
}


/**
 * @brief Normalize an angle to the range [-pi, pi).
 *
 * This function keeps the heading angle bounded so that downstream users
 * always receive a consistent orientation representation.
 *
 * @param angle_rad Input angle in radians.
 * @return Normalized angle in radians within [-pi, pi).
 */
double odom_normalize_angle(double angle_rad)
{
    while (angle_rad >= M_PI)
    {
        angle_rad -= 2.0 * M_PI;
    }
    while (angle_rad < -M_PI)
    {
        angle_rad += 2.0 * M_PI;
    }
    return angle_rad;
}



/**
 * @brief Initialize an odometry object with robot and encoder parameters.
 *
 * This function configures the odometry model for a differential drive robot.
 * It stores wheel geometry, encoder settings, inversion flags, and resets
 * pose, velocity, and tick history to their initial state.
 *
 * @param odom Pointer to the odometry object to initialize.
 * @param wheel_radius_m Radius of each wheel in meters.
 * @param wheel_base_m Distance between the left and right wheels in meters.
 * @param ticks_per_revolution Encoder ticks per full wheel revolution.
 * @param encoder_min Minimum encoder tick value.
 * @param encoder_max Maximum encoder tick value.
 * @param left_inverted Set to true if the left encoder direction is inverted.
 * @param right_inverted Set to true if the right encoder direction is inverted.
 * @return ODOM_OK on success.
 * @return ODOM_ERR_NULL if odom is NULL.
 * @return ODOM_ERR_INVALID_PARAM if any physical parameter is invalid.
 */
odom_status_t odom_init(odometry_t *odom,
                        double wheel_radius_m,
                        double wheel_base_m,
                        double ticks_per_revolution,
                        int64_t encoder_min,
                        int64_t encoder_max,
                        bool left_inverted,
                        bool right_inverted)
{
    if (odom == NULL)
    {
        return ODOM_ERR_NULL;
    }

    if (wheel_radius_m <= 0.0 || wheel_base_m <= 0.0 || ticks_per_revolution <= 0.0)
    {
        return ODOM_ERR_INVALID_PARAM;
    }

    if (encoder_min >= encoder_max)
    {
        return ODOM_ERR_INVALID_PARAM;
    }

    odom->wheel_radius_m = wheel_radius_m;
    odom->wheel_base_m = wheel_base_m;
    odom->ticks_per_revolution = ticks_per_revolution;
    odom->encoder_min = encoder_min;
    odom->encoder_max = encoder_max;
    odom->left_inverted = left_inverted;
    odom->right_inverted = right_inverted;

    odom->x_m = 0.0;
    odom->y_m = 0.0;
    odom->theta_rad = 0.0;
    odom->linear_velocity_mps = 0.0;
    odom->angular_velocity_rps = 0.0;

    odom->prev_left_tick = 0;
    odom->prev_right_tick = 0;
    odom->has_prev_tick = false;

    return ODOM_OK;
}



/**
 * @brief Reset the odometry state to the default origin pose.
 *
 * This function clears the current position, heading, velocity, and previous
 * tick history. After reset, the next update call will only store the new
 * baseline tick values without generating motion.
 *
 * @param odom Pointer to the odometry object.
 * @return ODOM_OK on success.
 * @return ODOM_ERR_NULL if odom is NULL.
 */
odom_status_t odom_reset(odometry_t *odom)
{
    if (odom == NULL)
    {
        return ODOM_ERR_NULL;
    }

    odom->x_m = 0.0;
    odom->y_m = 0.0;
    odom->theta_rad = 0.0;
    odom->linear_velocity_mps = 0.0;
    odom->angular_velocity_rps = 0.0;

    odom->prev_left_tick = 0;
    odom->prev_right_tick = 0;
    odom->has_prev_tick = false;

    return ODOM_OK;
}



/**
 * @brief Reset only the robot pose to a specified position and heading.
 *
 * This function updates the stored pose while preserving the configured robot
 * parameters and encoder settings. The heading angle is normalized into the
 * range [-pi, pi). Linear and angular velocities are reset to zero.
 *
 * Note that this function does not modify the stored previous encoder ticks.
 *
 * @param odom Pointer to the odometry object.
 * @param x_m New x position in meters.
 * @param y_m New y position in meters.
 * @param theta_rad New heading angle in radians.
 * @return ODOM_OK on success.
 * @return ODOM_ERR_NULL if odom is NULL.
 */
odom_status_t odom_reset_pose(odometry_t *odom,
                              double x_m,
                              double y_m,
                              double theta_rad)
{
    if (odom == NULL)
    {
        return ODOM_ERR_NULL;
    }

    odom->x_m = x_m;
    odom->y_m = y_m;
    odom->theta_rad = odom_normalize_angle(theta_rad);
    odom->linear_velocity_mps = 0.0;
    odom->angular_velocity_rps = 0.0;

    return ODOM_OK;
}



/**
 * @brief Reset only the stored encoder tick history.
 *
 * This function clears the previous left/right tick values and marks the
 * odometry object as not yet having a baseline tick sample. The next call to
 * odom_update() will only record the new tick values and will not compute
 * motion from them. Velocities are also reset to zero.
 *
 * @param odom Pointer to the odometry object.
 * @return ODOM_OK on success.
 * @return ODOM_ERR_NULL if odom is NULL.
 */
odom_status_t odom_reset_ticks(odometry_t *odom)
{
    if (odom == NULL)
    {
        return ODOM_ERR_NULL;
    }

    odom->prev_left_tick = 0;
    odom->prev_right_tick = 0;
    odom->has_prev_tick = false;
    odom->linear_velocity_mps = 0.0;
    odom->angular_velocity_rps = 0.0;

    return ODOM_OK;
}



/**
 * @brief Update odometry using the latest encoder ticks and elapsed time.
 *
 * This function performs a single odometry integration step for a differential
 * drive robot.
 *
 * The update process is:
 * 1. Validate input pointers, time step, and encoder tick ranges.
 * 2. On the first valid call, store the current ticks as the baseline only.
 * 3. Compute left/right tick differences with wraparound correction.
 * 4. Apply optional inversion for each wheel encoder.
 * 5. Convert tick differences into traveled distances.
 * 6. Compute the robot's linear displacement and heading change.
 * 7. Update x/y position using the midpoint heading approximation.
 * 8. Update linear and angular velocity estimates.
 *
 * @param odom Pointer to the odometry object.
 * @param left_tick Current left encoder tick.
 * @param right_tick Current right encoder tick.
 * @param dt_sec Elapsed time since the previous update in seconds.
 * @return ODOM_OK on success.
 * @return ODOM_ERR_NULL if odom is NULL.
 * @return ODOM_ERR_INVALID_DT if dt_sec is less than or equal to zero.
 * @return ODOM_ERR_TICK_OUT_OF_RANGE if either tick value is invalid.
 */
odom_status_t odom_update(odometry_t *odom,
                          int32_t left_tick,
                          int32_t right_tick,
                          double dt_sec)
{
    if (odom == NULL)
    {
        return ODOM_ERR_NULL;
    }

    if (dt_sec <= 0.0)
    {
        return ODOM_ERR_INVALID_DT;
    }

    if (!odom_tick_in_range(left_tick, odom->encoder_min, odom->encoder_max) ||
        !odom_tick_in_range(right_tick, odom->encoder_min, odom->encoder_max))
    {
        return ODOM_ERR_TICK_OUT_OF_RANGE;
    }

    /* First update: only store baseline tick
     * This means first input may be non-zero and still works correctly.
     */
    if (!odom->has_prev_tick)
    {
        odom->prev_left_tick = left_tick;
        odom->prev_right_tick = right_tick;
        odom->has_prev_tick = true;
        odom->linear_velocity_mps = 0.0;
        odom->angular_velocity_rps = 0.0;
        return ODOM_OK;
    }

    int32_t dleft_tick = odom_tick_diff_with_wrap(left_tick,
                                                  odom->prev_left_tick,
                                                  odom->encoder_min,
                                                  odom->encoder_max);

    int32_t dright_tick = odom_tick_diff_with_wrap(right_tick,
                                                   odom->prev_right_tick,
                                                   odom->encoder_min,
                                                   odom->encoder_max);

    odom->prev_left_tick = left_tick;
    odom->prev_right_tick = right_tick;

    if (odom->left_inverted)
    {
        dleft_tick = -dleft_tick;
    }

    if (odom->right_inverted)
    {
        dright_tick = -dright_tick;
    }

    const double meter_per_tick = (2.0 * M_PI * odom->wheel_radius_m) / odom->ticks_per_revolution;

    const double dl_m = (double)dleft_tick * meter_per_tick;
    const double dr_m = (double)dright_tick * meter_per_tick;

    const double ds_m = 0.5 * (dl_m + dr_m);
    const double dtheta_rad = (dr_m - dl_m) / odom->wheel_base_m;

    const double theta_mid = odom->theta_rad + 0.5 * dtheta_rad;

    odom->x_m += ds_m * cos(theta_mid);
    odom->y_m += ds_m * sin(theta_mid);
    odom->theta_rad = odom_normalize_angle(odom->theta_rad + dtheta_rad);

    odom->linear_velocity_mps = ds_m / dt_sec;
    odom->angular_velocity_rps = dtheta_rad / dt_sec;

    return ODOM_OK;
}


/**
 * @brief Get the current estimated robot pose.
 *
 * Any output pointer may be NULL if that field is not needed.
 *
 * @param odom Pointer to the odometry object.
 * @param x_m Output pointer for x position in meters.
 * @param y_m Output pointer for y position in meters.
 * @param theta_rad Output pointer for heading in radians.
 * @return ODOM_OK on success.
 * @return ODOM_ERR_NULL if odom is NULL.
 */
odom_status_t odom_get_pose(const odometry_t *odom,
                            double *x_m,
                            double *y_m,
                            double *theta_rad)
{
    if (odom == NULL)
    {
        return ODOM_ERR_NULL;
    }

    if (x_m != NULL)
    {
        *x_m = odom->x_m;
    }

    if (y_m != NULL)
    {
        *y_m = odom->y_m;
    }

    if (theta_rad != NULL)
    {
        *theta_rad = odom->theta_rad;
    }

    return ODOM_OK;
}



/**
 * @brief Get the current estimated robot velocities.
 *
 * Any output pointer may be NULL if that field is not needed.
 *
 * @param odom Pointer to the odometry object.
 * @param linear_velocity_mps Output pointer for linear velocity in meters per second.
 * @param angular_velocity_rps Output pointer for angular velocity in radians per second.
 * @return ODOM_OK on success.
 * @return ODOM_ERR_NULL if odom is NULL.
 */
odom_status_t odom_get_velocity(const odometry_t *odom,
                                double *linear_velocity_mps,
                                double *angular_velocity_rps)
{
    if (odom == NULL)
    {
        return ODOM_ERR_NULL;
    }

    if (linear_velocity_mps != NULL)
    {
        *linear_velocity_mps = odom->linear_velocity_mps;
    }

    if (angular_velocity_rps != NULL)
    {
        *angular_velocity_rps = odom->angular_velocity_rps;
    }

    return ODOM_OK;
}
