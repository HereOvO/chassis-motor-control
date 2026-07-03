#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_runtime.h"
#include "motor_driver.h"
#include "motor_feedback.h"
#include "motor_control.h"
#include "tim.h"
#include <stdint.h>

void MotorControl_Init(void);
void MotorControl_ResetAll(void);
void MotorControl_ApplyOutput(uint8_t motor_id, int8_t direction, uint32_t pulse);
void StopMotorPWM(uint8_t motor_id);
void SetMotorPWM(uint8_t motor_id, int8_t direction, uint32_t pulse);
uint32_t RpmToPulse(float rpm_value, uint8_t motor_id);
void MotorSpeedTask(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
