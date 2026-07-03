#include <stdint.h>

// ==========================================
// 宏定义
// ==========================================
#define MAX_PWM_PULSE  13999  // PWM最大值
#define MOTOR_COUNT    4      // 电机总数

// ==========================================
// PID 结构体
// ==========================================
typedef struct {
    float Kp;           // 比例
    float Ki;           // 积分
    float Kd;           // 微分
    
    float prevError;    // 上次误差
    float integral;     // 积分累计
} PID_Controller;

// 定义4个电机的PID数据保存区
PID_Controller motors[MOTOR_COUNT];

/**
 * @brief 初始化电机PID参数
 */
void PID_Init(uint8_t id, float kp, float ki, float kd) {
    if (id >= MOTOR_COUNT) return;
    motors[id].Kp = kp;
    motors[id].Ki = ki;
    motors[id].Kd = kd;
    motors[id].prevError = 0.0f;
    motors[id].integral = 0.0f;
}

/**
 * @brief PID计算函数
 * 
 * @param id           输入: 电机ID (0 - 3)
 * @param target_dir   输入: 目标方向 (-1:反转, 0:停止, 1:正转)
 * @param target_speed 输入: 目标速度绝对值 (RPM)
 * @param current_dir  输入: 当前实际方向 (-1:反转, 1:正转, 0:静止)
 * @param current_speed 输入: 当前实际速度绝对值 (RPM)
 * @param result_array 输出: 必须是一个长度为3的 int32_t 数组
 *                     result_array[0] -> 电机ID
 *                     result_array[1] -> 输出方向 (-1或1)
 *                     result_array[2] -> 输出PWM Pulse (0-13999)
 */
void Motor_PID_Calculate(uint8_t id, 
                         int8_t target_dir, float target_speed, 
                         int8_t current_dir, float current_speed, 
                         int32_t *result_array) 
{
    // 安全检查
    if (id >= MOTOR_COUNT) return;
    PID_Controller *pid = &motors[id];

    // 1. 将“方向+绝对值”转换为“带符号的数值”进行运算
    //    如果 target_dir 为 0，则目标真实速度为 0
    float real_target = 0.0f;
    if (target_dir == 1)       real_target = target_speed;
    else if (target_dir == -1) real_target = -target_speed;
    else                       real_target = 0.0f; // 停止模式

    //    处理当前速度符号
    float real_current = (current_dir == -1) ? -current_speed : current_speed;

    // 2. 计算误差
    float error = real_target - real_current;

    // 3. 积分项 (累计误差)
    pid->integral += error;

    //    积分限幅 (防止积分饱和)
    //    限制积分项最大贡献为 PWM最大值的 80%
    float integral_limit = (pid->Ki > 0.0001f) ? (MAX_PWM_PULSE * 0.8f / pid->Ki) : 0.0f;
    if (pid->integral > integral_limit) pid->integral = integral_limit;
    else if (pid->integral < -integral_limit) pid->integral = -integral_limit;

    // 4. 微分项
    float derivative = error - pid->prevError;

    // 5. 计算PID输出 (带符号)
    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

    // 6. 更新误差
    pid->prevError = error;

    // 7. 格式化输出结果 (拆分为 方向 + Pulse绝对值)
    int32_t out_dir = 1;
    int32_t out_pulse = 0;

    if (output >= 0) {
        out_dir = 1;      // 正向力矩
        out_pulse = (int32_t)output;
    } else {
        out_dir = -1;     // 反向力矩
        out_pulse = (int32_t)(-output); // 取绝对值
    }

    //    输出限幅
    if (out_pulse > MAX_PWM_PULSE) out_pulse = MAX_PWM_PULSE;

    // 8. 填入结果数组
    result_array[0] = (int32_t)id;       // 索引0: ID
    result_array[1] = out_dir;           // 索引1: 方向
    result_array[2] = out_pulse;         // 索引2: Pulse
}

