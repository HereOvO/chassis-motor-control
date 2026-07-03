#include "motor_driver.h"

const MotorHardwareBinding_t g_motor_hw_binding[MOTOR_COUNT] = {
    { &htim1, TIM_CHANNEL_1, &htim9, TIM_CHANNEL_1, &htim2, MOTOR_PWM_ARR, 1 },
    { &htim1, TIM_CHANNEL_2, &htim9, TIM_CHANNEL_2, &htim3, MOTOR_PWM_ARR, 1 },
    { &htim1, TIM_CHANNEL_3, &htim12, TIM_CHANNEL_1, &htim4, MOTOR_PWM_ARR, -1 },
    { &htim1, TIM_CHANNEL_4, &htim12, TIM_CHANNEL_2, &htim5, MOTOR_PWM_ARR, -1 }
};

static MotorDirection_t s_profile_stop_mode[MOTOR_COUNT];
static MotorDirection_t s_profile_brake_mode[MOTOR_COUNT];
static MotorDriverOutput_t s_last_output[MOTOR_COUNT];

static int8_t motor_resolve_direction(uint8_t motor_id, MotorDirection_t direction)
{
    const MotorHardwareBinding_t *binding;

    if (motor_id >= MOTOR_COUNT) {
        return MOTOR_DIRECTION_STOP;
    }

    binding = &g_motor_hw_binding[motor_id];
    if (direction == MOTOR_DIRECTION_FORWARD ||
        direction == MOTOR_DIRECTION_REVERSE ||
        direction == MOTOR_DIRECTION_STOP ||
        direction == MOTOR_DIRECTION_BRAKE ||
        direction == MOTOR_DIRECTION_STANDBY) {
        if (binding->direction_invert < 0) {
            if (direction == MOTOR_DIRECTION_FORWARD) {
                return MOTOR_DIRECTION_REVERSE;
            }
            if (direction == MOTOR_DIRECTION_REVERSE) {
                return MOTOR_DIRECTION_FORWARD;
            }
        }
        return direction;
    }

    return MOTOR_DIRECTION_STOP;
}

static void motor_write_low(const MotorHardwareBinding_t *binding)
{
    if (binding == NULL) {
        return;
    }

    (void)__HAL_TIM_SET_COMPARE(binding->pwm_forward_timer, binding->pwm_forward_channel, 0U);
    (void)__HAL_TIM_SET_COMPARE(binding->pwm_reverse_timer, binding->pwm_reverse_channel, 0U);
}

static void motor_driver_store_output(uint8_t motor_id,
                                      MotorDirection_t direction,
                                      float duty_norm,
                                      uint32_t forward_ccr,
                                      uint32_t reverse_ccr)
{
    if (motor_id >= MOTOR_COUNT) {
        return;
    }

    s_last_output[motor_id].direction = direction;
    s_last_output[motor_id].duty_norm = duty_norm;
    s_last_output[motor_id].forward_ccr = forward_ccr;
    s_last_output[motor_id].reverse_ccr = reverse_ccr;
}

void MotorDriver_Init(void)
{
    for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
        const MotorHardwareBinding_t *binding = &g_motor_hw_binding[motor_id];

        s_profile_stop_mode[motor_id] = MOTOR_DIRECTION_STOP;
        s_profile_brake_mode[motor_id] = MOTOR_DIRECTION_BRAKE;

        if (binding->pwm_forward_timer != NULL && binding->pwm_reverse_timer != NULL) {
            (void)HAL_TIM_PWM_Start(binding->pwm_forward_timer, binding->pwm_forward_channel);
            (void)HAL_TIM_PWM_Start(binding->pwm_reverse_timer, binding->pwm_reverse_channel);
            motor_write_low(binding);
            motor_driver_store_output(motor_id, MOTOR_DIRECTION_STOP, 0.0f, 0U, 0U);
        }
    }
}

void MotorDriver_ResetProfile(void)
{
    for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
        s_profile_stop_mode[motor_id] = MOTOR_DIRECTION_STOP;
        s_profile_brake_mode[motor_id] = MOTOR_DIRECTION_BRAKE;
    }
}

uint32_t MotorDriver_DutyToCcr(uint8_t motor_id, float duty_norm)
{
    uint32_t limit;
    float duty;
    float ccr;

    if (motor_id >= MOTOR_COUNT) {
        return 0U;
    }

    limit = g_motor_hw_binding[motor_id].pwm_limit;
    if (limit == 0U) {
        limit = MOTOR_PWM_ARR;
    }

    duty = duty_norm;
    if (duty < MOTOR_DUTY_NORM_MIN) {
        duty = MOTOR_DUTY_NORM_MIN;
    } else if (duty > MOTOR_DUTY_NORM_MAX) {
        duty = MOTOR_DUTY_NORM_MAX;
    }

    ccr = duty * (float)limit;
    if (ccr < 0.0f) {
        ccr = 0.0f;
    }

    return (uint32_t)(ccr + 0.5f);
}

void MotorDriver_SetOutput(uint8_t motor_id, MotorDirection_t direction, float duty_norm)
{
    const MotorHardwareBinding_t *binding;
    uint32_t ccr;
    int8_t resolved_direction;

    if (motor_id >= MOTOR_COUNT) {
        return;
    }

    binding = &g_motor_hw_binding[motor_id];
    if (binding->pwm_forward_timer == NULL || binding->pwm_reverse_timer == NULL) {
        return;
    }
    resolved_direction = motor_resolve_direction(motor_id, direction);
    ccr = MotorDriver_DutyToCcr(motor_id, duty_norm);
    motor_write_low(binding);

    if (resolved_direction == MOTOR_DIRECTION_FORWARD) {
        (void)__HAL_TIM_SET_COMPARE(binding->pwm_forward_timer, binding->pwm_forward_channel, ccr);
        motor_driver_store_output(motor_id, MOTOR_DIRECTION_FORWARD, duty_norm, ccr, 0U);
        return;
    }

    if (resolved_direction == MOTOR_DIRECTION_REVERSE) {
        (void)__HAL_TIM_SET_COMPARE(binding->pwm_reverse_timer, binding->pwm_reverse_channel, ccr);
        motor_driver_store_output(motor_id, MOTOR_DIRECTION_REVERSE, duty_norm, 0U, ccr);
        return;
    }

    if (resolved_direction == MOTOR_DIRECTION_BRAKE) {
        if (s_profile_brake_mode[motor_id] == MOTOR_DIRECTION_BRAKE) {
            (void)__HAL_TIM_SET_COMPARE(binding->pwm_forward_timer, binding->pwm_forward_channel, binding->pwm_limit);
            (void)__HAL_TIM_SET_COMPARE(binding->pwm_reverse_timer, binding->pwm_reverse_channel, binding->pwm_limit);
            motor_driver_store_output(motor_id,
                                      MOTOR_DIRECTION_BRAKE,
                                      1.0f,
                                      binding->pwm_limit,
                                      binding->pwm_limit);
        }
        return;
    }

    if (resolved_direction == MOTOR_DIRECTION_STANDBY) {
        if (s_profile_stop_mode[motor_id] == MOTOR_DIRECTION_STANDBY) {
            (void)__HAL_TIM_SET_COMPARE(binding->pwm_forward_timer, binding->pwm_forward_channel, 0U);
            (void)__HAL_TIM_SET_COMPARE(binding->pwm_reverse_timer, binding->pwm_reverse_channel, 0U);
        }
        motor_driver_store_output(motor_id, MOTOR_DIRECTION_STANDBY, 0.0f, 0U, 0U);
        return;
    }

    motor_driver_store_output(motor_id, MOTOR_DIRECTION_STOP, 0.0f, 0U, 0U);
}

MotorDriverOutput_t MotorDriver_GetOutput(uint8_t motor_id)
{
    MotorDriverOutput_t empty = { MOTOR_DIRECTION_STOP, 0.0f, 0U, 0U };

    if (motor_id >= MOTOR_COUNT) {
        return empty;
    }

    return s_last_output[motor_id];
}

void MotorDriver_Stop(uint8_t motor_id)
{
    MotorDriver_SetOutput(motor_id, MOTOR_DIRECTION_STOP, 0.0f);
}

void MotorDriver_Brake(uint8_t motor_id)
{
    MotorDriver_SetOutput(motor_id, MOTOR_DIRECTION_BRAKE, 0.0f);
}

void MotorDriver_Standby(uint8_t motor_id)
{
    MotorDriver_SetOutput(motor_id, MOTOR_DIRECTION_STANDBY, 0.0f);
}

void MotorDriver_StopAll(void)
{
    for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
        MotorDriver_Stop(motor_id);
    }
}
