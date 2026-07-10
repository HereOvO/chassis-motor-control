#ifndef CHASSIS_MODE_H
#define CHASSIS_MODE_H

#include "chassis_config.h"
#include "Motor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_MOTION_COMMAND_TIMEOUT_MS 200U

void chassis_mode_init(void);
bool chassis_mode_apply_profile(chassis_profile_id_t profile_id);
bool chassis_mode_apply_cmd(const chassis_cmd_t *cmd);
void chassis_mode_watchdog_update(void);
void chassis_mode_request_integrator_reset(void);
void chassis_mode_request_output_zero(void);
chassis_mode_t chassis_mode_get_active_mode(void);
chassis_profile_id_t chassis_mode_get_active_profile(void);
bool chassis_mode_raw_pwm_active(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_MODE_H */
