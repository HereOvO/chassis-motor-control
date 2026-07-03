#ifndef CHASSIS_KINEMATICS_H
#define CHASSIS_KINEMATICS_H

#include "chassis_config.h"

#ifdef __cplusplus
extern "C" {
#endif

void chassis_kinematics_inverse_diff(const chassis_profile_t *profile,
                                     float vx_mps,
                                     float wz_radps,
                                     float wheel_speed_rpm[CHASSIS_WHEEL_COUNT]);
void chassis_kinematics_inverse_mecanum(const chassis_profile_t *profile,
                                        float vx_mps,
                                        float vy_mps,
                                        float wz_radps,
                                        float wheel_speed_rpm[CHASSIS_WHEEL_COUNT]);
void chassis_kinematics_inverse(const chassis_profile_t *profile,
                                const chassis_cmd_t *cmd,
                                float wheel_speed_rpm[CHASSIS_WHEEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_KINEMATICS_H */
