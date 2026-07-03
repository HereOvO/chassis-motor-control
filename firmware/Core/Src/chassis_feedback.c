#include "chassis_feedback.h"

#include "chassis_mode.h"
#include "chassis_odometry.h"
#include "motor_control.h"
#include "motor_driver.h"
#include "motor_feedback.h"
#include "usart.h"

#include <stdio.h>
#include <string.h>

typedef struct
{
    const char *forward_pin;
    const char *reverse_pin;
} chassis_feedback_motor_pin_t;

static const chassis_feedback_motor_pin_t g_feedback_motor_pin[MOTOR_COUNT] = {
    { "PE9/TIM1_CH1", "PE5/TIM9_CH1" },
    { "PE11/TIM1_CH2", "PE6/TIM9_CH2" },
    { "PE13/TIM1_CH3", "PB14/TIM12_CH1" },
    { "PE14/TIM1_CH4", "PB15/TIM12_CH2" }
};

static void chassis_feedback_send_text(const char *text)
{
    if (text == NULL) {
        return;
    }

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 20U);
}

void chassis_feedback_report(void)
{
    char line[192];
    chassis_odometry_t odom;
    Motor_Status_t status[MOTOR_COUNT];
    MotorDriverOutput_t output[MOTOR_COUNT];
    float target[MOTOR_COUNT];
    int len;
    uint8_t i;

    odom = chassis_odometry_get();
    for (i = 0U; i < MOTOR_COUNT; ++i) {
        status[i] = MotorFeedback_GetStatus(i);
        output[i] = MotorDriver_GetOutput(i);
        target[i] = MotorControlCore_GetTarget(i);
    }

    len = snprintf(line,
                   sizeof(line),
                   "STAT,mode=%u,profile=%u,valid=%u,x=%.4f,y=%.4f,yaw=%.4f,vx=%.4f,vy=%.4f,wz=%.4f,upd=%lu\r\n",
                   (unsigned int)chassis_mode_get_active_mode(),
                   (unsigned int)chassis_mode_get_active_profile(),
                   (unsigned int)odom.valid,
                   odom.x_m,
                   odom.y_m,
                   odom.yaw_rad,
                   odom.vx_mps,
                   odom.vy_mps,
                   odom.wz_radps,
                   (unsigned long)odom.update_count);
    if (len > 0) {
        chassis_feedback_send_text(line);
    }

    len = snprintf(line,
                   sizeof(line),
                   "IK_RPM,m0=%.1f,m1=%.1f,m2=%.1f,m3=%.1f\r\n",
                   target[0],
                   target[1],
                   target[2],
                   target[3]);
    if (len > 0) {
        chassis_feedback_send_text(line);
    }

    for (i = 0U; i < MOTOR_COUNT; ++i) {
        const char *forward_state;
        const char *reverse_state;

        forward_state = (output[i].forward_ccr > 0U) ? "PWM" : "LOW";
        reverse_state = (output[i].reverse_ccr > 0U) ? "PWM" : "LOW";
        len = snprintf(line,
                       sizeof(line),
                       "M%u,target=%.1f,rpm=%.1f,delta=%ld,total=%ld,dir=%d,duty=%.3f,%s=%s(%lu),%s=%s(%lu)\r\n",
                       (unsigned int)i,
                       target[i],
                       status[i].speed_rpm,
                       (long)status[i].encode_delta,
                       (long)status[i].total_count,
                       (int)output[i].direction,
                       output[i].duty_norm,
                       g_feedback_motor_pin[i].forward_pin,
                       forward_state,
                       (unsigned long)output[i].forward_ccr,
                       g_feedback_motor_pin[i].reverse_pin,
                       reverse_state,
                       (unsigned long)output[i].reverse_ccr);
        if (len > 0) {
            chassis_feedback_send_text(line);
        }
    }

    chassis_feedback_send_text("\r\n");
}
