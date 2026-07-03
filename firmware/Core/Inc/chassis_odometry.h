#ifndef CHASSIS_ODOMETRY_H
#define CHASSIS_ODOMETRY_H

#include "chassis_config.h"
#include "motor_runtime.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    chassis_profile_id_t profile_id;
    chassis_drive_mode_t drive_mode;
    float x_m;
    float y_m;
    float yaw_rad;
    float vx_mps;
    float vy_mps;
    float wz_radps;
    float wheel_distance_m[CHASSIS_WHEEL_COUNT];
    int32_t wheel_total_count[CHASSIS_WHEEL_COUNT];
    uint32_t update_count;
    uint8_t valid;
} chassis_odometry_t;

void chassis_odometry_init(void);
void chassis_odometry_reset(void);
bool chassis_odometry_update(const chassis_profile_t *profile,
                             const Motor_Status_t motor_status[CHASSIS_WHEEL_COUNT],
                             float dt_s);
chassis_odometry_t chassis_odometry_get(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_ODOMETRY_H */
