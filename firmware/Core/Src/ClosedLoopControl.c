#include "ClosedLoopControl.h"

#include "motor_control.h"
#include "motor_driver.h"

#include <math.h>

extern void osDelay(uint32_t millisec);

void MotorControlProfiles_Init(void)
{
    MotorControlCore_Init();
}

void MotorControlProfiles_SetRuntimeTarget(uint8_t motor_id, int8_t direction, float speed_rpm)
{
    float target;

    if (motor_id >= MOTOR_COUNT) {
        return;
    }

    if (direction > 0) {
        target = fabsf(speed_rpm);
    } else if (direction < 0) {
        target = -fabsf(speed_rpm);
    } else {
        target = 0.0f;
    }

    MotorControlCore_SetTarget(motor_id, target);
}

void MotorControlProfiles_SetRuntimeTargets(const int8_t direction[MOTOR_COUNT],
                                            const float speed_rpm[MOTOR_COUNT])
{
    float target_rpm[MOTOR_COUNT];

    if (direction == NULL || speed_rpm == NULL) {
        return;
    }

    for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
        if (direction[motor_id] > 0) {
            target_rpm[motor_id] = fabsf(speed_rpm[motor_id]);
        } else if (direction[motor_id] < 0) {
            target_rpm[motor_id] = -fabsf(speed_rpm[motor_id]);
        } else {
            target_rpm[motor_id] = 0.0f;
        }
    }

    MotorControlCore_SetTargets(target_rpm);
}

void MotorControlProfiles_UseDefaultProfile(uint8_t motor_id, const MotorPidProfile_t *profile)
{
    MotorControlCore_SetProfile(motor_id, profile);
}

void Motor_PID_TaskFunc(void *argument)
{
    (void)argument;

    float output_cmd[MOTOR_COUNT];

    for (;;) {
        MotorControlCore_Compute(output_cmd);

        for (uint8_t motor_id = 0U; motor_id < MOTOR_COUNT; ++motor_id) {
            float cmd = output_cmd[motor_id];
            MotorDirection_t direction;
            float duty_norm;

            if (cmd > 0.0001f) {
                direction = MOTOR_DIRECTION_FORWARD;
                duty_norm = cmd;
            } else if (cmd < -0.0001f) {
                direction = MOTOR_DIRECTION_REVERSE;
                duty_norm = -cmd;
            } else {
                direction = MOTOR_DIRECTION_STOP;
                duty_norm = 0.0f;
            }

            MotorDriver_SetOutput(motor_id, direction, duty_norm);
        }

        osDelay(MOTOR_PID_CONTROL_PERIOD_MS);
    }
}
