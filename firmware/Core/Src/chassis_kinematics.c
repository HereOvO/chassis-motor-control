#include "chassis_kinematics.h"

#include <stddef.h>

#include "velocity_units.h"

static void chassis_kinematics_zero_output(float wheel_speed_rpm[CHASSIS_WHEEL_COUNT])
{
    if (wheel_speed_rpm == NULL) {
        return;
    }

    for (uint8_t i = 0U; i < CHASSIS_WHEEL_COUNT; ++i) {
        wheel_speed_rpm[i] = 0.0f;
    }
}

void chassis_kinematics_inverse_diff(const chassis_profile_t *profile,
                                     float vx_mps,
                                     float wz_radps,
                                     float wheel_speed_rpm[CHASSIS_WHEEL_COUNT])
{
    float left_mps;
    float right_mps;
    float half_track;
    float left_rpm;
    float right_rpm;

    if (wheel_speed_rpm == NULL || profile == NULL) {
        return;
    }

    half_track = profile->track_width_m * 0.5f;
    left_mps = vx_mps - (wz_radps * half_track);
    right_mps = vx_mps + (wz_radps * half_track);

    left_rpm = velocity_mps_to_rpm(left_mps, profile->wheel_radius_m);
    right_rpm = velocity_mps_to_rpm(right_mps, profile->wheel_radius_m);

    wheel_speed_rpm[0] = left_rpm * (float)profile->direction_sign[0];
    wheel_speed_rpm[1] = left_rpm * (float)profile->direction_sign[1];
    wheel_speed_rpm[2] = right_rpm * (float)profile->direction_sign[2];
    wheel_speed_rpm[3] = right_rpm * (float)profile->direction_sign[3];
}

void chassis_kinematics_inverse_mecanum(const chassis_profile_t *profile,
                                        float vx_mps,
                                        float vy_mps,
                                        float wz_radps,
                                        float wheel_speed_rpm[CHASSIS_WHEEL_COUNT])
{
    float wheel_radius;
    float half_track;
    float half_base;
    float kinematic_sum;
    float lf_mps;
    float lr_mps;
    float rr_mps;
    float rf_mps;

    if (wheel_speed_rpm == NULL || profile == NULL) {
        return;
    }

    wheel_radius = profile->wheel_radius_m;
    if (wheel_radius <= 0.0f) {
        chassis_kinematics_zero_output(wheel_speed_rpm);
        return;
    }

    half_track = profile->track_width_m * 0.5f;
    half_base = profile->wheel_base_m * 0.5f;
    kinematic_sum = half_track + half_base;

    lf_mps = vx_mps - vy_mps - (kinematic_sum * wz_radps);
    lr_mps = vx_mps + vy_mps - (kinematic_sum * wz_radps);
    rr_mps = vx_mps - vy_mps + (kinematic_sum * wz_radps);
    rf_mps = vx_mps + vy_mps + (kinematic_sum * wz_radps);

    wheel_speed_rpm[0] = velocity_mps_to_rpm(lf_mps, wheel_radius) * (float)profile->direction_sign[0];
    wheel_speed_rpm[1] = velocity_mps_to_rpm(lr_mps, wheel_radius) * (float)profile->direction_sign[1];
    wheel_speed_rpm[2] = velocity_mps_to_rpm(rr_mps, wheel_radius) * (float)profile->direction_sign[2];
    wheel_speed_rpm[3] = velocity_mps_to_rpm(rf_mps, wheel_radius) * (float)profile->direction_sign[3];
}

void chassis_kinematics_inverse(const chassis_profile_t *profile,
                                const chassis_cmd_t *cmd,
                                float wheel_speed_rpm[CHASSIS_WHEEL_COUNT])
{
    if (wheel_speed_rpm == NULL) {
        return;
    }

    chassis_kinematics_zero_output(wheel_speed_rpm);

    if (profile == NULL || cmd == NULL) {
        return;
    }

    if (profile->drive_mode == CHASSIS_DRIVE_DIFF) {
        chassis_kinematics_inverse_diff(profile, cmd->vx_mps, cmd->wz_radps, wheel_speed_rpm);
    } else {
        chassis_kinematics_inverse_mecanum(profile, cmd->vx_mps, cmd->vy_mps, cmd->wz_radps, wheel_speed_rpm);
    }
}
