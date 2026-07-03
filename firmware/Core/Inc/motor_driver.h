#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_runtime.h"
#include "tim.h"
#include <stdint.h>

typedef struct
{
    MotorDirection_t direction;
    float duty_norm;
    uint32_t forward_ccr;
    uint32_t reverse_ccr;
} MotorDriverOutput_t;

void MotorDriver_Init(void);
void MotorDriver_ResetProfile(void);
void MotorDriver_StopAll(void);
void MotorDriver_Stop(uint8_t motor_id);
void MotorDriver_Brake(uint8_t motor_id);
void MotorDriver_Standby(uint8_t motor_id);
void MotorDriver_SetOutput(uint8_t motor_id, MotorDirection_t direction, float duty_norm);
uint32_t MotorDriver_DutyToCcr(uint8_t motor_id, float duty_norm);
MotorDriverOutput_t MotorDriver_GetOutput(uint8_t motor_id);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_DRIVER_H__ */
