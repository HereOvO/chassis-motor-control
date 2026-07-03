#ifndef __MOTOR_RUNTIME_H__
#define __MOTOR_RUNTIME_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

#define MOTOR_COUNT 4U

#define MOTOR_BASE_PPR 500.0f
#define MOTOR_ENCODER_MULTIPLIER 4.0f
#define MOTOR_GEAR_RATIO 30.0f
#define MOTOR_PULSES_PER_REV_OUTPUT \
    (MOTOR_BASE_PPR * MOTOR_ENCODER_MULTIPLIER * MOTOR_GEAR_RATIO)

#define MOTOR_SPEED_SAMPLE_TIME_MS 20U
#define MOTOR_SPEED_SAMPLE_FREQUENCY_HZ (1000U / MOTOR_SPEED_SAMPLE_TIME_MS)
#define MOTOR_SPEED_SAMPLE_PERIOD_S (0.001f * (float)MOTOR_SPEED_SAMPLE_TIME_MS)

#define MOTOR_PID_CONTROL_PERIOD_MS 10U
#define MOTOR_PID_CONTROL_FREQUENCY_HZ (1000U / MOTOR_PID_CONTROL_PERIOD_MS)
#define MOTOR_PID_CONTROL_PERIOD_S (0.001f * (float)MOTOR_PID_CONTROL_PERIOD_MS)

#define MOTOR_PWM_ARR 13999U
#define MOTOR_PWM_MAX_TIM5_12 MOTOR_PWM_ARR
#define MOTOR_PWM_MAX_TIM9 MOTOR_PWM_ARR
#define MOTOR_DUTY_NORM_MIN 0.0f
#define MOTOR_DUTY_NORM_MAX 1.0f

typedef enum
{
    MOTOR_DIRECTION_STOP = 0,
    MOTOR_DIRECTION_FORWARD = 1,
    MOTOR_DIRECTION_REVERSE = -1,
    MOTOR_DIRECTION_BRAKE = 2,
    MOTOR_DIRECTION_STANDBY = 3
} MotorDirection_t;

typedef struct
{
    uint8_t dev;
    float speed_rpm;
    MotorDirection_t direction;
    int32_t encode_delta;
    int32_t total_count;
    uint32_t last_counter;
    uint8_t is_status_changed;
} Motor_Status_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float kv;
    float k_static;
    float output_limit;
    float integral_limit;
    float deadband_rpm;
} MotorPidProfile_t;

typedef struct
{
    TIM_HandleTypeDef *pwm_forward_timer;
    uint32_t pwm_forward_channel;
    TIM_HandleTypeDef *pwm_reverse_timer;
    uint32_t pwm_reverse_channel;
    TIM_HandleTypeDef *encoder_timer;
    uint32_t pwm_limit;
    int8_t direction_invert;
} MotorHardwareBinding_t;

extern Motor_Status_t all_motors[MOTOR_COUNT];
extern const MotorHardwareBinding_t g_motor_hw_binding[MOTOR_COUNT];
extern const MotorPidProfile_t g_motor_default_pid_profile[MOTOR_COUNT];

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_RUNTIME_H__ */
