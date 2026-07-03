#include "chassis_config.h"

#include <stddef.h>

static const chassis_profile_t g_default_profiles[CHASSIS_PROFILE_COUNT] = {
    {
        .profile_id = CHASSIS_PROFILE_DIFF,
        .drive_mode = CHASSIS_DRIVE_DIFF,
        .wheel_radius_m = CHASSIS_DEFAULT_WHEEL_RADIUS_M,
        .track_width_m = CHASSIS_DEFAULT_DIFF_TRACK_WIDTH_M,
        .wheel_base_m = 0.0f,
        .reduction_ratio = CHASSIS_DEFAULT_REDUCTION_RATIO,
        .encoder_ppr = CHASSIS_DEFAULT_ENCODER_PPR,
        .encoder_quadrature = CHASSIS_DEFAULT_ENCODER_QUADRATURE,
        .direction_sign = { 1, 1, 1, 1 },
        .max_linear_speed_mps = 1.335f,
        .max_angular_speed_radps = 3.00f,
        .max_linear_accel_mps2 = 2.00f,
        .max_angular_accel_radps2 = 4.00f,
        .pwm_limit = CHASSIS_DEFAULT_PWM_LIMIT,
    },
    {
        .profile_id = CHASSIS_PROFILE_MECANUM,
        .drive_mode = CHASSIS_DRIVE_MECANUM,
        .wheel_radius_m = CHASSIS_DEFAULT_WHEEL_RADIUS_M,
        .track_width_m = CHASSIS_DEFAULT_MECHANUM_TRACK_WIDTH_M,
        .wheel_base_m = CHASSIS_DEFAULT_MECHANUM_WHEEL_BASE_M,
        .reduction_ratio = CHASSIS_DEFAULT_REDUCTION_RATIO,
        .encoder_ppr = CHASSIS_DEFAULT_ENCODER_PPR,
        .encoder_quadrature = CHASSIS_DEFAULT_ENCODER_QUADRATURE,
        .direction_sign = { 1, 1, 1, 1 },
        .max_linear_speed_mps = 1.335f,
        .max_angular_speed_radps = 2.50f,
        .max_linear_accel_mps2 = 2.50f,
        .max_angular_accel_radps2 = 5.00f,
        .pwm_limit = CHASSIS_DEFAULT_PWM_LIMIT,
    },
};

static bool chassis_config_sign_valid(int8_t sign)
{
    return (sign == 1) || (sign == -1);
}

static bool chassis_config_profile_limits_valid(const chassis_profile_t *profile)
{
    if (profile == NULL) {
        return false;
    }

    if (profile->wheel_radius_m < CHASSIS_WHEEL_RADIUS_MIN_M ||
        profile->wheel_radius_m > CHASSIS_WHEEL_RADIUS_MAX_M) {
        return false;
    }

    if (profile->track_width_m < CHASSIS_TRACK_WIDTH_MIN_M ||
        profile->track_width_m > CHASSIS_TRACK_WIDTH_MAX_M) {
        return false;
    }

    if (profile->drive_mode == CHASSIS_DRIVE_MECANUM) {
        if (profile->wheel_base_m < CHASSIS_WHEEL_BASE_MIN_M ||
            profile->wheel_base_m > CHASSIS_WHEEL_BASE_MAX_M ||
            profile->wheel_base_m == 0.0f) {
            return false;
        }
    } else {
        if (profile->wheel_base_m < 0.0f || profile->wheel_base_m > CHASSIS_WHEEL_BASE_MAX_M) {
            return false;
        }
    }

    if (profile->reduction_ratio < CHASSIS_REDUCTION_RATIO_MIN ||
        profile->reduction_ratio > CHASSIS_REDUCTION_RATIO_MAX) {
        return false;
    }

    if (profile->encoder_ppr < CHASSIS_ENCODER_PPR_MIN ||
        profile->encoder_ppr > CHASSIS_ENCODER_PPR_MAX) {
        return false;
    }

    if (profile->encoder_quadrature < CHASSIS_ENCODER_QUADRATURE_MIN ||
        profile->encoder_quadrature > CHASSIS_ENCODER_QUADRATURE_MAX) {
        return false;
    }

    if (profile->max_linear_speed_mps < CHASSIS_LINEAR_SPEED_MIN_MPS ||
        profile->max_linear_speed_mps > CHASSIS_LINEAR_SPEED_MAX_MPS) {
        return false;
    }

    if (profile->max_angular_speed_radps < CHASSIS_ANGULAR_SPEED_MIN_RADPS ||
        profile->max_angular_speed_radps > CHASSIS_ANGULAR_SPEED_MAX_RADPS) {
        return false;
    }

    if (profile->max_linear_accel_mps2 < CHASSIS_LINEAR_ACCEL_MIN_MPS2 ||
        profile->max_linear_accel_mps2 > CHASSIS_LINEAR_ACCEL_MAX_MPS2) {
        return false;
    }

    if (profile->max_angular_accel_radps2 < CHASSIS_ANGULAR_ACCEL_MIN_RADPS2 ||
        profile->max_angular_accel_radps2 > CHASSIS_ANGULAR_ACCEL_MAX_RADPS2) {
        return false;
    }

    if (profile->pwm_limit < CHASSIS_PWM_LIMIT_MIN ||
        profile->pwm_limit > CHASSIS_PWM_LIMIT_MAX) {
        return false;
    }

    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i) {
        if (!chassis_config_sign_valid(profile->direction_sign[i])) {
            return false;
        }
    }

    return true;
}

const chassis_profile_t *chassis_config_get_profile(chassis_profile_id_t profile_id)
{
    if (profile_id >= CHASSIS_PROFILE_COUNT) {
        return NULL;
    }

    return &g_default_profiles[profile_id];
}

chassis_profile_id_t chassis_config_default_profile_id(void)
{
    return CHASSIS_PROFILE_MECANUM;
}

bool chassis_config_validate_profile(const chassis_profile_t *profile)
{
    if (profile == NULL) {
        return false;
    }

    if (profile->profile_id >= CHASSIS_PROFILE_COUNT) {
        return false;
    }

    if (profile->profile_id == CHASSIS_PROFILE_DIFF &&
        profile->drive_mode != CHASSIS_DRIVE_DIFF) {
        return false;
    }

    if (profile->profile_id == CHASSIS_PROFILE_MECANUM &&
        profile->drive_mode != CHASSIS_DRIVE_MECANUM) {
        return false;
    }

    return chassis_config_profile_limits_valid(profile);
}
