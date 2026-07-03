#include "chassis_feedback.h"

#include "chassis_config.h"
#include "chassis_mode.h"
#include "chassis_odometry.h"
#include "usart.h"

#if CHASSIS_USE_DEBUG_PROTOCOL
#include "motor_control.h"
#include "motor_driver.h"
#include "motor_feedback.h"

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
#else
static int16_t chassis_feedback_scale_i16(float value)
{
    float scaled;

    scaled = value * 1000.0f;
    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32768.0f) {
        return -32768;
    }
    if (scaled >= 0.0f) {
        return (int16_t)(scaled + 0.5f);
    }
    return (int16_t)(scaled - 0.5f);
}

static void chassis_feedback_put_i16_le(uint8_t *dst, int16_t value)
{
    uint16_t raw;

    if (dst == NULL) {
        return;
    }

    raw = (uint16_t)value;
    dst[0] = (uint8_t)(raw & 0xFFU);
    dst[1] = (uint8_t)((raw >> 8) & 0xFFU);
}
#endif

void chassis_feedback_report(void)
{
#if CHASSIS_USE_DEBUG_PROTOCOL
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
        const MotorPidProfile_t *pid;

        forward_state = (output[i].forward_ccr > 0U) ? "PWM" : "LOW";
        reverse_state = (output[i].reverse_ccr > 0U) ? "PWM" : "LOW";
        pid = MotorControlCore_GetProfile(i);
        len = snprintf(line,
                       sizeof(line),
                       "M%u,target=%.1f,rpm=%.1f,delta=%ld,total=%ld,dir=%d,duty=%.3f,kp=%.3f,ki=%.3f,kd=%.3f,kv=%.3f,ks=%.3f,olim=%.1f,ilim=%.1f,db=%.2f,%s=%s(%lu),%s=%s(%lu)\r\n",
                       (unsigned int)i,
                       target[i],
                       status[i].speed_rpm,
                       (long)status[i].encode_delta,
                       (long)status[i].total_count,
                       (int)output[i].direction,
                       output[i].duty_norm,
                       (pid != NULL) ? pid->kp : 0.0f,
                       (pid != NULL) ? pid->ki : 0.0f,
                       (pid != NULL) ? pid->kd : 0.0f,
                       (pid != NULL) ? pid->kv : 0.0f,
                       (pid != NULL) ? pid->k_static : 0.0f,
                       (pid != NULL) ? pid->output_limit : 0.0f,
                       (pid != NULL) ? pid->integral_limit : 0.0f,
                       (pid != NULL) ? pid->deadband_rpm : 0.0f,
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
#else
    uint8_t frame[12];
    uint8_t checksum;
    uint8_t i;
    chassis_odometry_t odom;

    odom = chassis_odometry_get();

    frame[0] = 0xAAU;
    frame[1] = 0xBBU;
    frame[2] = 0x0AU;
    frame[3] = 0x12U;
    chassis_feedback_put_i16_le(&frame[4], chassis_feedback_scale_i16(odom.vx_mps));
    chassis_feedback_put_i16_le(&frame[6], chassis_feedback_scale_i16(odom.vy_mps));
    chassis_feedback_put_i16_le(&frame[8], chassis_feedback_scale_i16(odom.wz_radps));
    frame[10] = 0x00U;

    checksum = 0U;
    for (i = 2U; i <= 9U; ++i) {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    frame[11] = checksum;

    (void)HAL_UART_Transmit(&huart1, frame, sizeof(frame), 20U);
#endif
}



