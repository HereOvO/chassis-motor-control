#ifndef __MOTOR_FEEDBACK_H__
#define __MOTOR_FEEDBACK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_runtime.h"
#include "tim.h"
#include <stdint.h>

void MotorFeedback_Init(void);
void MotorFeedback_Reset(uint8_t motor_id);
void MotorFeedback_Sample(uint8_t motor_id);
void MotorFeedback_Task(void *argument);
Motor_Status_t MotorFeedback_GetStatus(uint8_t motor_id);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_FEEDBACK_H__ */
