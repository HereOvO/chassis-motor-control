#include "Motor.h"

#include <math.h>

void MotorControl_Init(void)
{
    MotorControlCore_Init();
}

void MotorControl_ResetAll(void)
{
    MotorControlCore_ResetAll();
}

void MotorControl_ApplyOutput(uint8_t motor_id, int8_t direction, uint32_t pulse)
{
    float duty_norm;
    MotorDirection_t motor_direction;
    const MotorHardwareBinding_t *binding;
    uint32_t limit;

    if (motor_id >= MOTOR_COUNT) {
        return;
    }

    binding = &g_motor_hw_binding[motor_id];
    limit = binding->pwm_limit;
    if (limit == 0U) {
        limit = MOTOR_PWM_ARR;
    }

    duty_norm = (float)pulse / (float)limit;
    if (duty_norm < 0.0f) {
        duty_norm = 0.0f;
    } else if (duty_norm > 1.0f) {
        duty_norm = 1.0f;
    }

    if (direction > 0) {
        motor_direction = MOTOR_DIRECTION_FORWARD;
    } else if (direction < 0) {
        motor_direction = MOTOR_DIRECTION_REVERSE;
    } else if (direction == MOTOR_DIRECTION_BRAKE) {
        motor_direction = MOTOR_DIRECTION_BRAKE;
    } else {
        motor_direction = MOTOR_DIRECTION_STOP;
    }

    MotorDriver_SetOutput(motor_id, motor_direction, duty_norm);
}

void StopMotorPWM(uint8_t motor_id)
{
    MotorDriver_Stop(motor_id);
}

void SetMotorPWM(uint8_t motor_id, int8_t direction, uint32_t pulse)
{
    MotorControl_ApplyOutput(motor_id, direction, pulse);
}

uint32_t RpmToPulse(float rpm_value, uint8_t motor_id)
{
    const MotorPidProfile_t *profile;
    float normalized;
    float limit;

    if (motor_id >= MOTOR_COUNT) {
        return 0U;
    }

    profile = MotorControlCore_GetProfile(motor_id);
    if (profile == NULL) {
        return 0U;
    }

    limit = profile->output_limit;
    if (limit <= 0.0f) {
        limit = 1.0f;
    }

    normalized = fabsf(rpm_value) / 100.0f;
    if (profile->kv > 0.0001f) {
        normalized = fabsf(rpm_value) * profile->kv / (float)MOTOR_PWM_ARR;
    }

    if (normalized > limit) {
        normalized = limit;
    }

    return MotorDriver_DutyToCcr(motor_id, normalized);
}

void MotorSpeedTask(void *argument)
{
    MotorFeedback_Task(argument);
}
