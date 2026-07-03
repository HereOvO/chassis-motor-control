#include "motor_feedback.h"

#include "cmsis_os.h"

#include <string.h>

extern osMutexId_t MotorStatusMutexHandle;

Motor_Status_t all_motors[MOTOR_COUNT];
static Motor_Status_t s_motor_status_cache[MOTOR_COUNT];

static void motor_feedback_store_locked(uint8_t motor_id, const Motor_Status_t *state)
{
    if (motor_id >= MOTOR_COUNT || state == NULL) {
        return;
    }

    s_motor_status_cache[motor_id] = *state;
    all_motors[motor_id] = *state;
}

static Motor_Status_t *motor_feedback_slot(uint8_t motor_id)
{
    if (motor_id >= MOTOR_COUNT) {
        return NULL;
    }

    return &s_motor_status_cache[motor_id];
}

static Motor_Status_t motor_feedback_snapshot(uint8_t motor_id)
{
    Motor_Status_t empty = {0};
    Motor_Status_t *slot = motor_feedback_slot(motor_id);

    if (slot == NULL) {
        return empty;
    }

    return *slot;
}

static void motor_feedback_sample_all(void)
{
    for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
        MotorFeedback_Sample(motor_id);
    }
}

void MotorFeedback_Init(void)
{
    memset(s_motor_status_cache, 0, sizeof(s_motor_status_cache));

    for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
        s_motor_status_cache[motor_id].dev = motor_id;
        s_motor_status_cache[motor_id].direction = MOTOR_DIRECTION_STOP;
        s_motor_status_cache[motor_id].last_counter = 0U;
        all_motors[motor_id] = s_motor_status_cache[motor_id];
    }
}

void MotorFeedback_Reset(uint8_t motor_id)
{
    Motor_Status_t *slot = motor_feedback_slot(motor_id);

    if (slot == NULL) {
        return;
    }

    slot->dev = motor_id;
    slot->speed_rpm = 0.0f;
    slot->direction = MOTOR_DIRECTION_STOP;
    slot->encode_delta = 0;
    slot->total_count = 0;
    slot->last_counter = (uint32_t)__HAL_TIM_GET_COUNTER(g_motor_hw_binding[motor_id].encoder_timer);
    slot->is_status_changed = 0U;
    motor_feedback_store_locked(motor_id, slot);
}

void MotorFeedback_Sample(uint8_t motor_id)
{
    const MotorHardwareBinding_t *binding;
    Motor_Status_t *slot;
    uint32_t current_counter;
    int32_t delta;
    int32_t signed_delta;
    uint32_t previous_counter;
    uint32_t period;

    if (motor_id >= MOTOR_COUNT) {
        return;
    }

    binding = &g_motor_hw_binding[motor_id];
    if (binding->encoder_timer == NULL) {
        return;
    }

    slot = motor_feedback_slot(motor_id);
    if (slot == NULL) {
        return;
    }

    current_counter = (uint32_t)__HAL_TIM_GET_COUNTER(binding->encoder_timer);
    previous_counter = slot->last_counter;
    period = (uint32_t)__HAL_TIM_GET_AUTORELOAD(binding->encoder_timer) + 1U;
    if (period == 0U) {
        period = 65536U;
    }

    delta = (int32_t)current_counter - (int32_t)previous_counter;
    if (delta > (int32_t)(period / 2U)) {
        delta -= (int32_t)period;
    } else if (delta < -(int32_t)(period / 2U)) {
        delta += (int32_t)period;
    }

    signed_delta = delta;
    if (binding->direction_invert < 0) {
        signed_delta = -signed_delta;
    }

    slot->encode_delta = signed_delta;
    slot->last_counter = current_counter;
    slot->total_count += signed_delta;
    slot->speed_rpm = ((float)signed_delta * (60000.0f / (float)MOTOR_SPEED_SAMPLE_TIME_MS)) /
                      MOTOR_PULSES_PER_REV_OUTPUT;
    if (signed_delta > 0) {
        slot->direction = MOTOR_DIRECTION_FORWARD;
    } else if (signed_delta < 0) {
        slot->direction = MOTOR_DIRECTION_REVERSE;
    } else {
        slot->direction = MOTOR_DIRECTION_STOP;
    }
    slot->is_status_changed = (signed_delta != 0) ? 1U : 0U;

    if (MotorStatusMutexHandle != NULL) {
        (void)osMutexAcquire(MotorStatusMutexHandle, osWaitForever);
        motor_feedback_store_locked(motor_id, slot);
        (void)osMutexRelease(MotorStatusMutexHandle);
    } else {
        motor_feedback_store_locked(motor_id, slot);
    }
}

void MotorFeedback_Task(void *argument)
{
    Motor_Status_t *external_state = (Motor_Status_t *)argument;
    uint8_t motor_id;

    if (external_state == NULL) {
        for (motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
            MotorFeedback_Reset(motor_id);
        }
        for (;;) {
            motor_feedback_sample_all();
            osDelay(MOTOR_SPEED_SAMPLE_TIME_MS);
        }
    }

    motor_id = external_state->dev;
    if (motor_id >= MOTOR_COUNT) {
        return;
    }

    MotorFeedback_Reset(motor_id);
    *external_state = motor_feedback_snapshot(motor_id);

    for (;;) {
        MotorFeedback_Sample(motor_id);
        *external_state = motor_feedback_snapshot(motor_id);
        osDelay(MOTOR_SPEED_SAMPLE_TIME_MS);
    }
}

Motor_Status_t MotorFeedback_GetStatus(uint8_t motor_id)
{
    Motor_Status_t snapshot;

    if (motor_id >= MOTOR_COUNT) {
        Motor_Status_t empty = {0};
        return empty;
    }

    if (MotorStatusMutexHandle != NULL) {
        (void)osMutexAcquire(MotorStatusMutexHandle, osWaitForever);
        snapshot = s_motor_status_cache[motor_id];
        (void)osMutexRelease(MotorStatusMutexHandle);
        return snapshot;
    }

    return motor_feedback_snapshot(motor_id);
}
