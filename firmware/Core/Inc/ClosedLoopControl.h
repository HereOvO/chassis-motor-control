#ifndef __CLOSED_LOOP_CONTROL_H__
#define __CLOSED_LOOP_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_runtime.h"
#include "motor_driver.h"
#include "motor_control.h"
#include <stdint.h>

void MotorControlProfiles_Init(void);
void MotorControlProfiles_SetRuntimeTarget(uint8_t motor_id, int8_t direction, float speed_rpm);
void MotorControlProfiles_SetRuntimeTargets(const int8_t direction[MOTOR_COUNT],
                                            const float speed_rpm[MOTOR_COUNT]);
void MotorControlProfiles_UseDefaultProfile(uint8_t motor_id, const MotorPidProfile_t *profile);
void Motor_PID_TaskFunc(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __CLOSED_LOOP_CONTROL_H__ */
