#include "velocity_units.h"

#define CHASSIS_TWO_PI_F 6.28318530717958647692f

float velocity_radps_to_rpm(float radps)
{
    return radps * 60.0f / CHASSIS_TWO_PI_F;
}

float velocity_rpm_to_radps(float rpm)
{
    return rpm * CHASSIS_TWO_PI_F / 60.0f;
}

float velocity_mps_to_rpm(float linear_mps, float wheel_radius_m)
{
    float circumference_m;

    if (wheel_radius_m <= 0.0f) {
        return 0.0f;
    }

    circumference_m = CHASSIS_TWO_PI_F * wheel_radius_m;
    return (linear_mps / circumference_m) * 60.0f;
}

float velocity_rpm_to_mps(float wheel_rpm, float wheel_radius_m)
{
    float circumference_m;

    if (wheel_radius_m <= 0.0f) {
        return 0.0f;
    }

    circumference_m = CHASSIS_TWO_PI_F * wheel_radius_m;
    return (wheel_rpm * circumference_m) / 60.0f;
}
