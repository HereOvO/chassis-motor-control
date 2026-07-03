#include "motor_control.h"
#include "motor_driver.h"
#include "motor_feedback.h"

#include <math.h>

typedef struct
{
    float prev_error;
    float integral;
} MotorPidState_t;

const MotorPidProfile_t g_motor_default_pid_profile[MOTOR_COUNT] = {
    { .kp = 60.0f, .ki = 8.0f, .kd = 0.5f, .kv = 36.74f, .k_static = 100.0f, .output_limit = (float)MOTOR_PWM_ARR, .integral_limit = 300.0f, .deadband_rpm = 1.0f },
    { .kp = 50.0f, .ki = 10.0f, .kd = 0.5f, .kv = 36.74f, .k_static = 100.0f, .output_limit = (float)MOTOR_PWM_ARR, .integral_limit = 300.0f, .deadband_rpm = 1.0f },
    { .kp = 230.0f, .ki = 15.0f, .kd = 0.03f, .kv = 36.74f, .k_static = 100.0f, .output_limit = (float)MOTOR_PWM_ARR, .integral_limit = 300.0f, .deadband_rpm = 1.0f },
    { .kp = 230.0f, .ki = 15.0f, .kd = 0.03f, .kv = 172.0f, .k_static = 100.0f, .output_limit = (float)MOTOR_PWM_ARR, .integral_limit = 300.0f, .deadband_rpm = 1.0f }
};

static MotorPidProfile_t s_profile[MOTOR_COUNT];
static MotorPidState_t s_pid_state[MOTOR_COUNT];
static float s_target_rpm[MOTOR_COUNT];
static uint8_t s_core_initialized;

static float motor_control_clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

static void motor_control_reset_state(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) {
        return;
    }

    s_pid_state[motor_id].prev_error = 0.0f;
    s_pid_state[motor_id].integral = 0.0f;
}

static float motor_control_output_limit(uint8_t motor_id)
{
    float limit;

    if (motor_id >= MOTOR_COUNT) {
        return 0.0f;
    }

    limit = s_profile[motor_id].output_limit;
    if (limit <= 0.0f) {
        limit = 1.0f;
    }

    return limit;
}

void MotorControlCore_Init(void)
{
    if (s_core_initialized != 0U) {
        return;
    }

    MotorDriver_Init();
    MotorFeedback_Init();
    for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
        s_profile[motor_id] = g_motor_default_pid_profile[motor_id];
        s_target_rpm[motor_id] = 0.0f;
        motor_control_reset_state(motor_id);
    }
    MotorDriver_StopAll();
    s_core_initialized = 1U;
}

void MotorControlCore_ResetAll(void)
{
    for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
        MotorControlCore_Reset(motor_id);
    }
}

void MotorControlCore_Reset(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) {
        return;
    }

    s_target_rpm[motor_id] = 0.0f;
    motor_control_reset_state(motor_id);
    MotorDriver_Stop(motor_id);
}

void MotorControlCore_ResetIntegrator(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) {
        return;
    }

    motor_control_reset_state(motor_id);
}

void MotorControlCore_ResetIntegratorAll(void)
{
    for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
        MotorControlCore_ResetIntegrator(motor_id);
    }
}

void MotorControlCore_SetProfile(uint8_t motor_id, const MotorPidProfile_t *profile)
{
    if (motor_id >= MOTOR_COUNT || profile == NULL) {
        return;
    }

    s_profile[motor_id] = *profile;
    if (s_profile[motor_id].output_limit <= 0.0f) {
        s_profile[motor_id].output_limit = (float)MOTOR_PWM_ARR;
    }
    if (s_profile[motor_id].integral_limit < 0.0f) {
        s_profile[motor_id].integral_limit = -s_profile[motor_id].integral_limit;
    }
}

const MotorPidProfile_t *MotorControlCore_GetProfile(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) {
        return NULL;
    }

    return &s_profile[motor_id];
}

void MotorControlCore_SetTargets(const float target_rpm[MOTOR_COUNT])
{
    if (target_rpm == NULL) {
        return;
    }

    for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
        s_target_rpm[motor_id] = target_rpm[motor_id];
    }
}

void MotorControlCore_SetTarget(uint8_t motor_id, float target_rpm)
{
    if (motor_id >= MOTOR_COUNT) {
        return;
    }

    s_target_rpm[motor_id] = target_rpm;
}

float MotorControlCore_GetTarget(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) {
        return 0.0f;
    }

    return s_target_rpm[motor_id];
}

void MotorControlCore_Compute(float out_cmd[MOTOR_COUNT])
{
    const float dt = MOTOR_PID_CONTROL_PERIOD_S;

    if (out_cmd == NULL) {
        return;
    }

    for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
        Motor_Status_t status;
        MotorPidProfile_t profile;
        float target_speed;
        float actual_speed;
        float error;
        float p_term;
        float i_term;
        float d_term;
        float ff_term;
        float static_term;
        float output_signed;
        float limit;
        float output_norm;

        profile = s_profile[motor_id];
        status = MotorFeedback_GetStatus(motor_id);
        target_speed = s_target_rpm[motor_id];
        actual_speed = status.speed_rpm;

        error = target_speed - actual_speed;
        s_pid_state[motor_id].integral += error * dt;
        if (profile.integral_limit > 0.0f) {
            s_pid_state[motor_id].integral = motor_control_clamp_float(
                s_pid_state[motor_id].integral,
                -profile.integral_limit,
                profile.integral_limit);
        }

        if ((target_speed == 0.0f) && (fabsf(error) <= profile.deadband_rpm)) {
            s_pid_state[motor_id].integral = 0.0f;
        }

        p_term = profile.kp * error;
        i_term = profile.ki * s_pid_state[motor_id].integral;
        d_term = profile.kd * ((error - s_pid_state[motor_id].prev_error) / dt);
        s_pid_state[motor_id].prev_error = error;

        ff_term = profile.kv * target_speed;
        static_term = 0.0f;
        if (target_speed > profile.deadband_rpm) {
            static_term = profile.k_static;
        } else if (target_speed < -profile.deadband_rpm) {
            static_term = -profile.k_static;
        }

        output_signed = p_term + i_term + d_term + ff_term + static_term;
        limit = motor_control_output_limit(motor_id);
        if (limit <= 0.0f) {
            out_cmd[motor_id] = 0.0f;
            continue;
        }

        output_signed = motor_control_clamp_float(output_signed, -limit, limit);
        output_norm = output_signed / limit;
        if (fabsf(output_norm) <= 0.0001f) {
            output_norm = 0.0f;
        }

        out_cmd[motor_id] = output_norm;
    }
}
