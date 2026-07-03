#include "chassis_odometry.h"

#include <math.h>
#include <string.h>

static chassis_odometry_t g_odometry;

static float chassis_odometry_counts_per_output_rev(const chassis_profile_t *profile)
{
    if (profile == NULL || profile->encoder_ppr == 0U || profile->encoder_quadrature == 0U) {
        return 0.0f;
    }

    return (float)profile->encoder_ppr *
           (float)profile->encoder_quadrature *
           profile->reduction_ratio;
}

static float chassis_odometry_normalize_angle(float angle_rad)
{
    const float pi = 3.14159265358979323846f;
    const float two_pi = 6.28318530717958647692f;

    while (angle_rad > pi) {
        angle_rad -= two_pi;
    }
    while (angle_rad < -pi) {
        angle_rad += two_pi;
    }

    return angle_rad;
}

static bool chassis_odometry_profile_valid(const chassis_profile_t *profile)
{
    if (profile == NULL || !chassis_config_validate_profile(profile)) {
        return false;
    }

    if (profile->wheel_radius_m <= 0.0f || profile->reduction_ratio <= 0.0f) {
        return false;
    }

    if (profile->drive_mode == CHASSIS_DRIVE_MECANUM) {
        return (profile->track_width_m > 0.0f && profile->wheel_base_m > 0.0f);
    }

    return (profile->track_width_m > 0.0f);
}

static void chassis_odometry_integrate_body_delta(float body_dx_m, float body_dy_m, float body_dyaw_rad)
{
    float yaw_mid;
    float cos_yaw;
    float sin_yaw;

    yaw_mid = g_odometry.yaw_rad + (0.5f * body_dyaw_rad);
    cos_yaw = cosf(yaw_mid);
    sin_yaw = sinf(yaw_mid);

    g_odometry.x_m += (body_dx_m * cos_yaw) - (body_dy_m * sin_yaw);
    g_odometry.y_m += (body_dx_m * sin_yaw) + (body_dy_m * cos_yaw);
    g_odometry.yaw_rad = chassis_odometry_normalize_angle(g_odometry.yaw_rad + body_dyaw_rad);
}

void chassis_odometry_init(void)
{
    chassis_odometry_reset();
}

void chassis_odometry_reset(void)
{
    memset(&g_odometry, 0, sizeof(g_odometry));
    g_odometry.profile_id = CHASSIS_PROFILE_MECANUM;
    g_odometry.drive_mode = CHASSIS_DRIVE_MECANUM;
}

bool chassis_odometry_update(const chassis_profile_t *profile,
                             const Motor_Status_t motor_status[CHASSIS_WHEEL_COUNT],
                             float dt_s)
{
    const float two_pi = 6.28318530717958647692f;
    float counts_per_rev;
    float meters_per_count;
    float wheel_delta_m[CHASSIS_WHEEL_COUNT];
    float body_dx_m;
    float body_dy_m;
    float body_dyaw_rad;
    float rotation_sum;
    uint8_t i;

    if (motor_status == NULL || dt_s <= 0.0f || !chassis_odometry_profile_valid(profile)) {
        g_odometry.valid = 0U;
        return false;
    }

    counts_per_rev = chassis_odometry_counts_per_output_rev(profile);
    if (counts_per_rev <= 0.0f) {
        g_odometry.valid = 0U;
        return false;
    }

    meters_per_count = (two_pi * profile->wheel_radius_m) / counts_per_rev;
    for (i = 0U; i < CHASSIS_WHEEL_COUNT; ++i) {
        wheel_delta_m[i] = (float)motor_status[i].encode_delta *
                           meters_per_count *
                           (float)profile->direction_sign[i];
        g_odometry.wheel_distance_m[i] += wheel_delta_m[i];
        g_odometry.wheel_total_count[i] = motor_status[i].total_count;
    }

    if (profile->drive_mode == CHASSIS_DRIVE_MECANUM) {
        rotation_sum = profile->wheel_base_m + profile->track_width_m;
        if (rotation_sum <= 0.0f) {
            g_odometry.valid = 0U;
            return false;
        }

        body_dx_m = (wheel_delta_m[0] + wheel_delta_m[1] + wheel_delta_m[2] + wheel_delta_m[3]) * 0.25f;
        body_dy_m = (-wheel_delta_m[0] + wheel_delta_m[1] - wheel_delta_m[2] + wheel_delta_m[3]) * 0.25f;
        body_dyaw_rad = (-wheel_delta_m[0] - wheel_delta_m[1] + wheel_delta_m[2] + wheel_delta_m[3]) /
                        (2.0f * rotation_sum);
    } else {
        float left_delta_m;
        float right_delta_m;

        left_delta_m = (wheel_delta_m[0] + wheel_delta_m[1]) * 0.5f;
        right_delta_m = (wheel_delta_m[2] + wheel_delta_m[3]) * 0.5f;
        body_dx_m = (left_delta_m + right_delta_m) * 0.5f;
        body_dy_m = 0.0f;
        body_dyaw_rad = (right_delta_m - left_delta_m) / profile->track_width_m;
    }

    chassis_odometry_integrate_body_delta(body_dx_m, body_dy_m, body_dyaw_rad);

    g_odometry.profile_id = profile->profile_id;
    g_odometry.drive_mode = profile->drive_mode;
    g_odometry.vx_mps = body_dx_m / dt_s;
    g_odometry.vy_mps = body_dy_m / dt_s;
    g_odometry.wz_radps = body_dyaw_rad / dt_s;
    g_odometry.valid = 1U;
    ++g_odometry.update_count;

    return true;
}

chassis_odometry_t chassis_odometry_get(void)
{
    return g_odometry;
}
