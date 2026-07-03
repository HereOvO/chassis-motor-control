#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "chassis_config.h"
#include "motor_runtime.h"
#include <stdint.h>

void MotorControlCore_Init(void);
void MotorControlCore_ResetAll(void);
void MotorControlCore_Reset(uint8_t motor_id);
void MotorControlCore_ResetIntegrator(uint8_t motor_id);
void MotorControlCore_ResetIntegratorAll(void);
void MotorControlCore_SetProfile(uint8_t motor_id, const MotorPidProfile_t *profile);
void MotorControlCore_SetTargets(const float target_rpm[MOTOR_COUNT]);
void MotorControlCore_SetTarget(uint8_t motor_id, float target_rpm);
float MotorControlCore_GetTarget(uint8_t motor_id);
void MotorControlCore_Compute(float out_cmd[MOTOR_COUNT]);
const MotorPidProfile_t *MotorControlCore_GetProfile(uint8_t motor_id);
bool MotorControlCore_SetParam(uint8_t motor_id, motor_control_param_id_t param_id, float value);
bool MotorControlCore_GetParam(uint8_t motor_id, motor_control_param_id_t param_id, float *value);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H__ */

