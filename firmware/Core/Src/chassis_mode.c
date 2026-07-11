#include "chassis_mode.h"

#include "main.h"
#include "chassis_kinematics.h"
#include "chassis_odometry.h"
#include "motor_driver.h"
#include "motor_control.h"
#include "runtime_tune.h"

static chassis_mode_t g_active_mode = CHASSIS_MODE_STOP;
static chassis_profile_id_t g_active_profile = CHASSIS_PROFILE_MECANUM;
static uint8_t g_raw_pwm_active = 0U;
static uint8_t g_motion_watchdog_armed = 0U;
static uint32_t g_motion_watchdog_last_refresh_ms = 0U;
static uint32_t g_motion_watchdog_trip_count = 0U;
static chassis_cmd_t g_last_velocity_cmd;
static uint8_t g_last_velocity_cmd_valid = 0U;

static bool chassis_mode_motion_enabled(void);

static MotorDirection_t chassis_mode_raw_direction_to_motor(int8_t direction)
{
    if (direction > 0) {
        return MOTOR_DIRECTION_FORWARD;
    }
    if (direction < 0) {
        return MOTOR_DIRECTION_REVERSE;
    }
    return MOTOR_DIRECTION_STOP;
}

static float chassis_mode_clamp_duty(float duty_norm)
{
    if (duty_norm < 0.0f) {
        return 0.0f;
    }
    if (duty_norm > 1.0f) {
        return 1.0f;
    }
    return duty_norm;
}

static void chassis_mode_stop_motion(void)
{
    g_raw_pwm_active = 0U;
    g_last_velocity_cmd_valid = 0U;
    MotorControlCore_ResetAll();
}

static void chassis_mode_reset_integrators(void)
{
    MotorControlCore_ResetIntegratorAll();
}

static void chassis_mode_clear_targets(void)
{
    float zero_targets[CHASSIS_WHEEL_COUNT] = { 0.0f, 0.0f, 0.0f, 0.0f };

    MotorControlCore_SetTargets(zero_targets);
}

static void chassis_mode_clamp_velocity(const chassis_profile_t *profile, chassis_cmd_t *cmd)
{
    if (profile == NULL || cmd == NULL) {
        return;
    }

    if (cmd->vx_mps > profile->max_linear_speed_mps) {
        cmd->vx_mps = profile->max_linear_speed_mps;
    } else if (cmd->vx_mps < -profile->max_linear_speed_mps) {
        cmd->vx_mps = -profile->max_linear_speed_mps;
    }

    if (cmd->vy_mps > profile->max_linear_speed_mps) {
        cmd->vy_mps = profile->max_linear_speed_mps;
    } else if (cmd->vy_mps < -profile->max_linear_speed_mps) {
        cmd->vy_mps = -profile->max_linear_speed_mps;
    }

    if (cmd->wz_radps > profile->max_angular_speed_radps) {
        cmd->wz_radps = profile->max_angular_speed_radps;
    } else if (cmd->wz_radps < -profile->max_angular_speed_radps) {
        cmd->wz_radps = -profile->max_angular_speed_radps;
    }
}

static void chassis_mode_reapply_last_velocity(void)
{
    const chassis_profile_t *profile;
    chassis_cmd_t cmd;
    float wheel_speed_rpm[CHASSIS_WHEEL_COUNT];

    if (g_last_velocity_cmd_valid == 0U || !chassis_mode_motion_enabled()) {
        return;
    }

    profile = runtime_tune_get_active_profile();
    if (profile == NULL) {
        return;
    }

    cmd = g_last_velocity_cmd;
    chassis_mode_clamp_velocity(profile, &cmd);
    chassis_kinematics_inverse(profile, &cmd, wheel_speed_rpm);
    MotorControlCore_SetTargets(wheel_speed_rpm);
}

static bool chassis_mode_motion_enabled(void)
{
    return (g_active_mode == CHASSIS_MODE_VELOCITY) || (g_active_mode == CHASSIS_MODE_TUNE);
}

static void chassis_mode_watchdog_refresh(void)
{
    g_motion_watchdog_last_refresh_ms = HAL_GetTick();
    g_motion_watchdog_armed = 1U;
}

static void chassis_mode_watchdog_disarm(void)
{
    g_motion_watchdog_last_refresh_ms = HAL_GetTick();
    g_motion_watchdog_armed = 0U;
}

void chassis_mode_watchdog_update(void)
{
    uint32_t now_ms;

    if (g_motion_watchdog_armed == 0U) {
        return;
    }

    if (!chassis_mode_motion_enabled()) {
        chassis_mode_watchdog_disarm();
        return;
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - g_motion_watchdog_last_refresh_ms) < CHASSIS_MOTION_COMMAND_TIMEOUT_MS) {
        return;
    }

    ++g_motion_watchdog_trip_count;
    g_active_mode = CHASSIS_MODE_STOP;
    g_raw_pwm_active = 0U;
    chassis_mode_watchdog_disarm();
    chassis_mode_stop_motion();
}

void chassis_mode_init(void)
{
    MotorControlCore_Init();
    runtime_tune_init();
    g_active_profile = runtime_tune_get_state()->active_profile_id;
    g_active_mode = CHASSIS_MODE_STOP;
    g_motion_watchdog_trip_count = 0U;
    chassis_mode_watchdog_disarm();
    MotorDriver_ResetProfile();
    chassis_odometry_reset();
    chassis_mode_stop_motion();
}

bool chassis_mode_apply_profile(chassis_profile_id_t profile_id)
{
    if (profile_id >= CHASSIS_PROFILE_COUNT) {
        return false;
    }

    if (!runtime_tune_switch_profile(profile_id)) {
        return false;
    }

    g_active_profile = profile_id;
    g_active_mode = CHASSIS_MODE_STOP;
    MotorDriver_ResetProfile();
    chassis_odometry_reset();
    chassis_mode_watchdog_disarm();
    chassis_mode_stop_motion();
    return true;
}

void chassis_mode_request_integrator_reset(void)
{
    chassis_mode_reset_integrators();
}

void chassis_mode_request_output_zero(void)
{
    g_active_mode = CHASSIS_MODE_STOP;
    chassis_mode_watchdog_disarm();
    chassis_mode_stop_motion();
}

static bool chassis_mode_apply_raw_pwm(const chassis_cmd_t *cmd)
{
    MotorDirection_t direction;
    float duty_norm;
    uint8_t motor_id;

    if (cmd == NULL || cmd->motor_id >= CHASSIS_WHEEL_COUNT) {
        return false;
    }

    motor_id = cmd->motor_id;
    direction = chassis_mode_raw_direction_to_motor(cmd->raw_direction);
    duty_norm = chassis_mode_clamp_duty(cmd->duty_norm);

    MotorControlCore_ResetAll();
    g_raw_pwm_active = 1U;
    g_active_mode = CHASSIS_MODE_TUNE;

    if (direction == MOTOR_DIRECTION_STOP || duty_norm <= 0.0f) {
        g_active_mode = CHASSIS_MODE_STOP;
        g_raw_pwm_active = 0U;
        chassis_mode_watchdog_disarm();
        MotorDriver_Stop(motor_id);
        return true;
    }

    MotorDriver_SetOutput(motor_id, direction, duty_norm);
    chassis_mode_watchdog_refresh();
    return true;
}

bool chassis_mode_apply_cmd(const chassis_cmd_t *cmd)
{
    const chassis_profile_t *profile;
    chassis_cmd_t local_cmd;
    float wheel_speed_rpm[CHASSIS_WHEEL_COUNT];

    if (cmd == NULL) {
        return false;
    }

    local_cmd = *cmd;
    profile = runtime_tune_get_active_profile();
    if (profile == NULL) {
        return false;
    }

    switch (local_cmd.type) {
    case CHASSIS_CMD_MODE:
        if (local_cmd.mode > CHASSIS_MODE_FAULT) {
            return false;
        }

        g_active_mode = local_cmd.mode;
        g_raw_pwm_active = 0U;
        if (g_active_mode == CHASSIS_MODE_STOP || g_active_mode == CHASSIS_MODE_FAULT) {
            chassis_mode_watchdog_disarm();
            chassis_mode_stop_motion();
        } else {
            chassis_mode_watchdog_refresh();
            chassis_mode_reset_integrators();
        }
        return true;

    case CHASSIS_CMD_PROFILE:
        return chassis_mode_apply_profile(local_cmd.profile_id);

    case CHASSIS_CMD_MOTOR_PARAM_SET:
        if (!MotorControlCore_SetParam(local_cmd.motor_id, local_cmd.motor_param_id, local_cmd.value)) {
            return false;
        }
        chassis_mode_reset_integrators();
        return true;
    case CHASSIS_CMD_PARAM_SET:
        if (!runtime_tune_set_param(local_cmd.param_id, local_cmd.value)) {
            return false;
        }
        chassis_mode_reset_integrators();
        chassis_mode_reapply_last_velocity();
        return true;

    case CHASSIS_CMD_PARAM_COMMIT:
        return runtime_tune_commit();

    case CHASSIS_CMD_PARAM_RESTORE:
        if (!runtime_tune_restore_defaults()) {
            return false;
        }
        g_active_mode = CHASSIS_MODE_STOP;
        chassis_odometry_reset();
        chassis_mode_watchdog_disarm();
        chassis_mode_stop_motion();
        return true;

    case CHASSIS_CMD_ENABLE:
        if (local_cmd.enable == 0U) {
            g_active_mode = CHASSIS_MODE_STOP;
            g_raw_pwm_active = 0U;
            chassis_mode_watchdog_disarm();
            chassis_mode_stop_motion();
        } else {
            g_active_mode = CHASSIS_MODE_VELOCITY;
            g_raw_pwm_active = 0U;
            chassis_mode_watchdog_refresh();
            chassis_mode_reset_integrators();
        }
        return true;

    case CHASSIS_CMD_INIT:
        g_active_mode = CHASSIS_MODE_STOP;
        g_raw_pwm_active = 0U;
        chassis_odometry_reset();
        chassis_mode_watchdog_disarm();
        chassis_mode_stop_motion();
        return true;

    case CHASSIS_CMD_ZERO:
        g_active_mode = CHASSIS_MODE_STOP;
        g_raw_pwm_active = 0U;
        chassis_odometry_reset();
        chassis_mode_watchdog_disarm();
        chassis_mode_stop_motion();
        return true;

    case CHASSIS_CMD_RAW_PWM:
        return chassis_mode_apply_raw_pwm(&local_cmd);

    case CHASSIS_CMD_VELOCITY:
        g_raw_pwm_active = 0U;
        if ((local_cmd.flags & CHASSIS_CMD_FLAG_AUTO_ENABLE) != 0UL) {
            g_active_mode = CHASSIS_MODE_VELOCITY;
        }
        if (!chassis_mode_motion_enabled()) {
            chassis_mode_clear_targets();
            chassis_mode_watchdog_disarm();
            chassis_mode_stop_motion();
            chassis_mode_reset_integrators();
            return true;
        }

        chassis_mode_clamp_velocity(profile, &local_cmd);
        chassis_kinematics_inverse(profile, &local_cmd, wheel_speed_rpm);
        MotorControlCore_SetTargets(wheel_speed_rpm);
        g_last_velocity_cmd = local_cmd;
        g_last_velocity_cmd_valid = 1U;
        chassis_mode_watchdog_refresh();
        return true;

    case CHASSIS_CMD_NONE:
    default:
        return false;
    }
}

chassis_mode_t chassis_mode_get_active_mode(void)
{
    return g_active_mode;
}

chassis_profile_id_t chassis_mode_get_active_profile(void)
{
    return g_active_profile;
}

bool chassis_mode_raw_pwm_active(void)
{
    return g_raw_pwm_active != 0U;
}



