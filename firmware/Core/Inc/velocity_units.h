#ifndef VELOCITY_UNITS_H
#define VELOCITY_UNITS_H

#ifdef __cplusplus
extern "C" {
#endif

float velocity_radps_to_rpm(float radps);
float velocity_rpm_to_radps(float rpm);
float velocity_mps_to_rpm(float linear_mps, float wheel_radius_m);
float velocity_rpm_to_mps(float wheel_rpm, float wheel_radius_m);

#ifdef __cplusplus
}
#endif

#endif /* VELOCITY_UNITS_H */
