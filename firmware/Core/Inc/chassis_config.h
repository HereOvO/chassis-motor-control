#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CHASSIS_WHEEL_COUNT 4U

/*
 * Protocol build switch:
 *   1: ASCII debug protocol, keeps VEL/PWMTEST/SET commands for bring-up.
 *   0: MOWEN production binary protocol from MOWEN car sensor/chassis spec.
 */
#ifndef CHASSIS_USE_DEBUG_PROTOCOL
#define CHASSIS_USE_DEBUG_PROTOCOL 0U
#endif

#define CHASSIS_WHEEL_RADIUS_MIN_M 0.01f
#define CHASSIS_WHEEL_RADIUS_MAX_M 0.30f
#define CHASSIS_TRACK_WIDTH_MIN_M 0.05f
#define CHASSIS_TRACK_WIDTH_MAX_M 1.20f
#define CHASSIS_WHEEL_BASE_MIN_M 0.00f
#define CHASSIS_WHEEL_BASE_MAX_M 1.20f
#define CHASSIS_REDUCTION_RATIO_MIN 0.10f
#define CHASSIS_REDUCTION_RATIO_MAX 500.0f
#define CHASSIS_ENCODER_PPR_MIN 1U
#define CHASSIS_ENCODER_PPR_MAX 65535U
#define CHASSIS_ENCODER_QUADRATURE_MIN 1U
#define CHASSIS_ENCODER_QUADRATURE_MAX 16U
#define CHASSIS_LINEAR_SPEED_MIN_MPS 0.0f
#define CHASSIS_LINEAR_SPEED_MAX_MPS 5.0f
#define CHASSIS_ANGULAR_SPEED_MIN_RADPS 0.0f
#define CHASSIS_ANGULAR_SPEED_MAX_RADPS 20.0f
#define CHASSIS_LINEAR_ACCEL_MIN_MPS2 0.0f
#define CHASSIS_LINEAR_ACCEL_MAX_MPS2 20.0f
#define CHASSIS_ANGULAR_ACCEL_MIN_RADPS2 0.0f
#define CHASSIS_ANGULAR_ACCEL_MAX_RADPS2 50.0f
#define CHASSIS_PWM_LIMIT_MIN 1U
#define CHASSIS_PWM_LIMIT_MAX 65535U

#define CHASSIS_DEFAULT_WHEEL_RADIUS_M 0.0255f
#define CHASSIS_DEFAULT_DIFF_TRACK_WIDTH_M 0.250f
#define CHASSIS_DEFAULT_MECHANUM_TRACK_WIDTH_M 0.250f
#define CHASSIS_DEFAULT_MECHANUM_WHEEL_BASE_M 0.220f
#define CHASSIS_DEFAULT_REDUCTION_RATIO 30.0f
#define CHASSIS_DEFAULT_ENCODER_PPR 500U
#define CHASSIS_DEFAULT_ENCODER_QUADRATURE 4U
#define CHASSIS_DEFAULT_PWM_LIMIT 13999U
#define CHASSIS_DEFAULT_MAX_WHEEL_SPEED_RPM 500.0f

typedef enum
{
    CHASSIS_DRIVE_DIFF = 0U,
    CHASSIS_DRIVE_MECANUM = 1U
} chassis_drive_mode_t;

typedef enum
{
    CHASSIS_PROFILE_DIFF = 0U,
    CHASSIS_PROFILE_MECANUM = 1U,
    CHASSIS_PROFILE_COUNT = 2U
} chassis_profile_id_t;

typedef enum
{
    CHASSIS_MODE_STOP = 0U,
    CHASSIS_MODE_VELOCITY = 1U,
    CHASSIS_MODE_TUNE = 2U,
    CHASSIS_MODE_FAULT = 3U
} chassis_mode_t;

typedef enum
{
    CHASSIS_PROTOCOL_MODE_ASCII = 0U,
    CHASSIS_PROTOCOL_MODE_MOWEN = 1U
} chassis_protocol_mode_t;

#if CHASSIS_USE_DEBUG_PROTOCOL
#define CHASSIS_DEFAULT_PROTOCOL_MODE CHASSIS_PROTOCOL_MODE_ASCII
#else
#define CHASSIS_DEFAULT_PROTOCOL_MODE CHASSIS_PROTOCOL_MODE_MOWEN
#endif

typedef enum
{
    RUNTIME_PARAM_WHEEL_RADIUS = 0U,
    RUNTIME_PARAM_TRACK_WIDTH = 1U,
    RUNTIME_PARAM_WHEEL_BASE = 2U,
    RUNTIME_PARAM_REDUCTION_RATIO = 3U,
    RUNTIME_PARAM_ENCODER_PPR = 4U,
    RUNTIME_PARAM_DIRECTION_SIGN_0 = 5U,
    RUNTIME_PARAM_DIRECTION_SIGN_1 = 6U,
    RUNTIME_PARAM_DIRECTION_SIGN_2 = 7U,
    RUNTIME_PARAM_DIRECTION_SIGN_3 = 8U,
    RUNTIME_PARAM_MAX_LINEAR_SPEED = 9U,
    RUNTIME_PARAM_MAX_ANGULAR_SPEED = 10U,
    RUNTIME_PARAM_MAX_LINEAR_ACCEL = 11U,
    RUNTIME_PARAM_MAX_ANGULAR_ACCEL = 12U,
    RUNTIME_PARAM_PWM_LIMIT = 13U,
    RUNTIME_PARAM_COUNT = 14U
} runtime_param_id_t;


typedef enum
{
    MOTOR_PARAM_KP = 0U,
    MOTOR_PARAM_KI = 1U,
    MOTOR_PARAM_KD = 2U,
    MOTOR_PARAM_KV = 3U,
    MOTOR_PARAM_K_STATIC = 4U,
    MOTOR_PARAM_OUTPUT_LIMIT = 5U,
    MOTOR_PARAM_INTEGRAL_LIMIT = 6U,
    MOTOR_PARAM_DEADBAND_RPM = 7U,
    MOTOR_PARAM_COUNT = 8U
} motor_control_param_id_t;

typedef enum
{
    CHASSIS_CMD_NONE = 0U,
    CHASSIS_CMD_VELOCITY = 1U,
    CHASSIS_CMD_ENABLE = 2U,
    CHASSIS_CMD_MODE = 3U,
    CHASSIS_CMD_PROFILE = 4U,
    CHASSIS_CMD_PARAM_SET = 5U,
    CHASSIS_CMD_PARAM_COMMIT = 6U,
    CHASSIS_CMD_PARAM_RESTORE = 7U,
    CHASSIS_CMD_ZERO = 8U,
    CHASSIS_CMD_RAW_PWM = 9U,
    CHASSIS_CMD_MOTOR_PARAM_SET = 10U,
    CHASSIS_CMD_INIT = 11U
} chassis_cmd_type_t;

#define CHASSIS_CMD_FLAG_ZERO_OUTPUT (1UL << 0)
#define CHASSIS_CMD_FLAG_RESET_INTEGRATOR (1UL << 1)
#define CHASSIS_CMD_FLAG_PROFILE_SWITCH (1UL << 2)
#define CHASSIS_CMD_FLAG_AUTO_ENABLE (1UL << 3)

typedef struct
{
    chassis_profile_id_t profile_id;
    chassis_drive_mode_t drive_mode;
    float wheel_radius_m;
    float track_width_m;
    float wheel_base_m;
    float reduction_ratio;
    uint16_t encoder_ppr;
    uint8_t encoder_quadrature;
    int8_t direction_sign[CHASSIS_WHEEL_COUNT];
    float max_linear_speed_mps;
    float max_angular_speed_radps;
    float max_linear_accel_mps2;
    float max_angular_accel_radps2;
    uint16_t pwm_limit;
} chassis_profile_t;

typedef struct
{
    chassis_cmd_type_t type;
    chassis_mode_t mode;
    chassis_profile_id_t profile_id;
    runtime_param_id_t param_id;
    motor_control_param_id_t motor_param_id;
    float vx_mps;
    float vy_mps;
    float wz_radps;
    float value;
    float duty_norm;
    uint8_t motor_id;
    int8_t raw_direction;
    uint8_t enable;
    uint32_t flags;
} chassis_cmd_t;

const chassis_profile_t *chassis_config_get_profile(chassis_profile_id_t profile_id);
chassis_profile_id_t chassis_config_default_profile_id(void);
bool chassis_config_validate_profile(const chassis_profile_t *profile);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_CONFIG_H */








