/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : jetson_uart.c
  * @brief          : UART communication implementation for Jetson Nano
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "jetson_uart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"  // 需要访问huart4和huart5
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <motor.h>
#include "cmsis_os.h"  // 添加FreeRTOS头文件以使用互斥锁
#include <stm32f4xx_hal_gpio.h>
#include <uart.h>
/* USER CODE END Includes */

/*extern PV*/
extern Motor_Status_t all_motors[4];

//Mutex句柄
extern osMutexId_t MotorStatusMutexHandle;

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief 舵机控制消息结构体
 */
typedef struct {
    uint8_t servo_id;      /*!< 舵机ID (0-15) */
    uint16_t angle;        /*!< 目标角度 (0-180度) */
    uint16_t duration;     /*!< 执行时间 (ms) */
} ServoControlMsg_t;

/**
 * @brief 多舵机姿态结构体
 */
typedef struct {
    uint16_t angles[16];   /*!< 16个舵机的角度值 */
    uint32_t timestamp;    /*!< 时间戳 */
} MultiServoPose_t;

/**
 * @brief IMU数据结构体
 */
typedef struct {
    float accel_x_g;       /*!< X轴加速度 (g) */
    float accel_y_g;       /*!< Y轴加速度 (g) */
    float accel_z_g;       /*!< Z轴加速度 (g) */
    float gyro_x_dps;      /*!< X轴角速度 (°/s) */
    float gyro_y_dps;      /*!< Y轴角速度 (°/s) */
    float gyro_z_dps;      /*!< Z轴角速度 (°/s) */
    float temperature_c;   /*!< 温度 (°C) */
} IMUData_t;

/**
 * @brief 电机数据结构体
 */
typedef struct {
    int8_t direction[4];          /*!< 4个电机的转向 (1:正转, -1:反转, 0:停止) */
    float speed_rpm[4];           /*!< 4个电机的转速 (RPM) */
    uint32_t timestamp;           /*!< 时间戳 */
} MotorData_t;

/**
 * @brief 综合姿态信息结构体（舵机+IMU+电机）
 */
typedef struct {
    MultiServoPose_t servo_pose;  /*!< 舵机姿态 */
    IMUData_t imu_data;           /*!< IMU数据 */
    MotorData_t motor_data;       /*!< 电机数据 */
    uint32_t timestamp;           /*!< 综合时间戳 */
} ComprehensivePose_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

static uint8_t uart5_rx_buffer[1024];  /*!< UART5接收缓冲区，增大缓冲区以应对高频数据 */
static uint8_t uart5_rx_index = 0;      /*!< UART5接收索引 */
static uint8_t uart5_frame_complete = 0; /*!< 帧完成标志 */
static uint8_t uart5_rx_data;           /*!< UART5接收数据 */

static JetsonMessage_t received_msg;     /*!< 用于接收消息的临时缓冲区 */

static uint32_t last_receive_tick = 0;  /*!< 最后接收时间戳 */
static const uint32_t RECEIVE_TIMEOUT_MS = 200;  /*!< 接收超时时间 (200ms) */

static MultiServoPose_t current_pose = {0};     /*!< 当前舵机姿态 */
static uint32_t last_pose_update = 0;           /*!< 上次姿态更新时间 */

static ComprehensivePose_t comprehensive_pose = {0};  /*!< 综合姿态信息（舵机+IMU+电机） */

// 存储上位机发来的电机控制命令
int8_t motor_cmd_direction[4] = {0};  /*!< 上位机发来的电机转向命令 */
float motor_cmd_speed[4] = {0.0f};   /*!< 上位机发来的电机转速命令 */
static uint8_t motor_cmd_type[4] = {0};     /*!< 上位机发来的电机命令类型 (0=慢刹车, 1=快刹车, 2=反转, 3=正转) */





/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

// 解析舵机控制消息
HAL_StatusTypeDef ParseServoControlMessage(uint8_t *buffer, uint8_t length);

// 发送当前姿态消息
HAL_StatusTypeDef SendCurrentPose(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// 串口接收单字节缓冲区
static uint8_t uart_rx_byte;

// 启动UART5中断接收
HAL_StatusTypeDef JetsonUart_StartReceiveIT(void)
{
    // 确保huart5句柄有效
    if(&huart5 == NULL) {
        return HAL_ERROR;
    }

    // 启动单字节中断接收
    return HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1);
}

// UART接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART5) {
        // 更新最后接收时间戳
        last_receive_tick = HAL_GetTick();

        // 处理接收到的数据
        Jetson_ProcessReceivedData(&uart_rx_byte, 1);

        // 重新启动接收下一个字节
        if(HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1) != HAL_OK) {
            // 如果重新启动接收失败，可以发送错误信息
            char error_msg[] = "UART5 Re-start receive error\r\n";
            HAL_UART_Transmit(&huart4, (uint8_t*)error_msg, strlen(error_msg), 0);
        }
    }
}

/* USER CODE END 0 */

/* Exported functions --------------------------------------------------------*/

/* USER CODE BEGIN 1 */
// UART错误回调函数
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART5) {
        // 检查是否真的是错误（错误码非0）
        if(huart->ErrorCode != HAL_UART_ERROR_NONE) {
            // 清除错误状态
            huart->ErrorCode = HAL_UART_ERROR_NONE;
        }
        // 无论是否有实际错误，都重新启动接收
        HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1);
    }
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
// 检查接收超时的函数
void Jetson_CheckReceiveTimeout(void)
{
    // 检查是否超过接收超时时间
    if(uart5_rx_index > 0 && (HAL_GetTick() - last_receive_tick) > RECEIVE_TIMEOUT_MS) {
        // 超时，重置接收索引，丢弃未完成的消息
        uart5_rx_index = 0;
    }
}


/**
 * @brief 解析舵机控制消息
 * @param buffer 消息缓冲区
 * @param length 消息长度
 * @return HAL_StatusTypeDef HAL_OK 成功，HAL_ERROR 失败
 *
 * 消息格式: "SERVO,<servo_id>,<angle>,<duration>\n"
 * 例如: "SERVO,0,90,1000\n" - 控制0号舵机转到90度，用时1000ms
 */
HAL_StatusTypeDef ParseServoControlMessage(uint8_t *buffer, uint8_t length)
{
    if(buffer == NULL || length == 0) {
        return HAL_ERROR;
    }

    // 简单解析消息（实际应用中应使用更健壮的解析方法）
    char msg_str[64];
    if(length >= sizeof(msg_str)) {
        return HAL_ERROR; // 消息太长
    }

    // 复制到临时字符串
    for(int i = 0; i < length && i < sizeof(msg_str)-1; i++) {
        msg_str[i] = (char)buffer[i];
    }
    msg_str[length] = '\0';

    // 检查消息头
    if(strncmp(msg_str, "SERVO,", 6) != 0) {
        return HAL_ERROR; // 不是舵机控制消息
    }

    // 解析参数
    char *token = strtok(msg_str, ",");
    if(token == NULL || strcmp(token, "SERVO") != 0) {
        return HAL_ERROR;
    }

    // 解析舵机ID
    token = strtok(NULL, ",");
    if(token == NULL) {
        return HAL_ERROR;
    }
    uint8_t servo_id = atoi(token);

    // 解析角度
    token = strtok(NULL, ",");
    if(token == NULL) {
        return HAL_ERROR;
    }
    uint16_t angle = atoi(token);

    // 解析持续时间
    token = strtok(NULL, ",");
    if(token == NULL) {
        return HAL_ERROR;
    }
    uint16_t duration = atoi(token);

    // 限制范围
    if(servo_id >= 16 || angle > 180) {
        return HAL_ERROR;
    }

    // 更新当前姿态
    current_pose.angles[servo_id] = angle;
    current_pose.timestamp = HAL_GetTick();
    last_pose_update = HAL_GetTick();

    // 同时更新综合姿态信息
    comprehensive_pose.servo_pose.angles[servo_id] = angle;
    comprehensive_pose.servo_pose.timestamp = HAL_GetTick();

    // 调用PCA9685控制函数（需要外部提供）
    extern void PCA9685_SetAngle(I2C_HandleTypeDef *hi2c, uint8_t num, uint8_t angle);
    extern I2C_HandleTypeDef hi2c2;  /*!< 外部I2C句柄 */

    // 直接设置舵机角度，不使用缓慢移动
    PCA9685_SetAngle(&hi2c2, servo_id, angle);

    return HAL_OK;
}


/**
 * @brief 解析电机控制消息
 * @param buffer 消息缓冲区
 * @param length 消息长度
 * @return HAL_StatusTypeDef HAL_OK 成功，HAL_ERROR 失败
 *
 * 消息格式: "MOTOR,<motor_id>,<signed_speed_rpm>\n"
 * signed_speed_rpm: 正值=正转, 负值=反转, 0=停止
 * 例如: "MOTOR,0,50.5\n" - 控制0号电机正转，转速50.5 RPM
 *       "MOTOR,0,-30.0\n" - 控制0号电机反转，转速30.0 RPM
 *       "MOTOR,0,0.0\n" - 控制0号电机停止
 */
HAL_StatusTypeDef ParseMotorControlMessage(uint8_t *buffer, uint8_t length)
{
    if(buffer == NULL || length == 0) {
        return HAL_ERROR;
    }

    // 简单解析消息（实际应用中应使用更健壮的解析方法）
    char msg_str[128];
    if(length >= sizeof(msg_str)) {
        return HAL_ERROR; // 消息太长
    }

    // 复制到临时字符串
    for(int i = 0; i < length && i < sizeof(msg_str)-1; i++) {
        msg_str[i] = (char)buffer[i];
    }
    msg_str[length] = '\0';

    // 检查消息头
    if(strncmp(msg_str, "MOTOR,", 6) != 0) {
        return HAL_ERROR; // 不是电机控制消息
    }

    // 解析参数
    char *token = strtok(msg_str, ",");
    if(token == NULL || strcmp(token, "MOTOR") != 0) {
        return HAL_ERROR;
    }

    // 解析电机ID
    token = strtok(NULL, ",");
    if(token == NULL) {
        return HAL_ERROR;
    }
    uint8_t motor_id = atoi(token);

    // 解析带符号的转速
    token = strtok(NULL, ",");
    if(token == NULL) {
        return HAL_ERROR;
    }
    float signed_speed_rpm = atof(token);

    // 限制范围
    if(motor_id >= 4) {
        return HAL_ERROR;
    }

    // 根据速度值确定方向和绝对速度
    int8_t direction = 0;
    float speed_rpm = 0.0f;

    if(signed_speed_rpm > 0.0f) {
        direction = 1;      // 正转
        speed_rpm = signed_speed_rpm;
    } else if(signed_speed_rpm < 0.0f) {
        direction = -1;     // 反转
        speed_rpm = -signed_speed_rpm;  // 取绝对值
    } else {
        direction = 0;      // 停止
        speed_rpm = 0.0f;
    }

    // 存储上位机发来的电机控制命令（使用原子操作保护）
    if(motor_id < 4) {
        // 在中断中禁用中断以保护数据一致性
        uint32_t primask = __get_PRIMASK();
        __disable_irq();

        motor_cmd_direction[motor_id] = direction;
        motor_cmd_speed[motor_id] = speed_rpm;
        // 将方向转换为命令类型（-1=反转，0=停止，1=正转）
        int8_t command_type = 0;  // 默认停止
        if(direction == 1) {
            command_type = 1;  // 正转
        } else if(direction == -1) {
            command_type = -1;  // 反转
        } else {  // direction == 0 (停止/刹车)
            command_type = 0;  // 停止
        }
        motor_cmd_type[motor_id] = (uint8_t)command_type;

        // 恢复中断状态
        __set_PRIMASK(primask);
    }

    // 这里可以添加对电机的控制逻辑（目前只更新内部状态）
    // extern void ControlMotor(uint8_t motor_id, int8_t direction, float speed_rpm);
    // ControlMotor(motor_id, direction, speed_rpm);

    return HAL_OK;
}

/**
 * @brief 发送当前姿态消息
 * @return HAL_StatusTypeDef HAL_OK 成功，其他值 失败
 *
 * 消息格式: "POSE,<servo0>,<servo1>,...,<servo15>,<timestamp>,<accel_x>,<accel_y>,<accel_z>,<gyro_x>,<gyro_y>,<gyro_z>,<temperature>,<dir1>,<dir2>,<dir3>,<dir4>,<rpm1>,<rpm2>,<rpm3>,<rpm4>\n"
 */
HAL_StatusTypeDef SendCurrentPose(void)
{
    // 进入临界区
    taskENTER_CRITICAL() ;

    char pose_msg[512]; // 增大缓冲区以容纳更多数据
    sprintf(pose_msg, "POSE,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f\n",
            comprehensive_pose.servo_pose.angles[0], comprehensive_pose.servo_pose.angles[1],
            comprehensive_pose.servo_pose.angles[2], comprehensive_pose.servo_pose.angles[3],
            comprehensive_pose.servo_pose.angles[4], comprehensive_pose.servo_pose.angles[5],
            comprehensive_pose.servo_pose.angles[6], comprehensive_pose.servo_pose.angles[7],
            comprehensive_pose.servo_pose.angles[8], comprehensive_pose.servo_pose.angles[9],
            comprehensive_pose.servo_pose.angles[10], comprehensive_pose.servo_pose.angles[11],
            comprehensive_pose.servo_pose.angles[12], comprehensive_pose.servo_pose.angles[13],
            comprehensive_pose.servo_pose.angles[14], comprehensive_pose.servo_pose.angles[15],
            comprehensive_pose.servo_pose.timestamp,
            comprehensive_pose.imu_data.accel_x_g, comprehensive_pose.imu_data.accel_y_g, comprehensive_pose.imu_data.accel_z_g,
            comprehensive_pose.imu_data.gyro_x_dps, comprehensive_pose.imu_data.gyro_y_dps, comprehensive_pose.imu_data.gyro_z_dps,
            comprehensive_pose.imu_data.temperature_c,
            comprehensive_pose.motor_data.direction[0], comprehensive_pose.motor_data.direction[1],
            comprehensive_pose.motor_data.direction[2], comprehensive_pose.motor_data.direction[3],
            comprehensive_pose.motor_data.speed_rpm[0], comprehensive_pose.motor_data.speed_rpm[1],
            comprehensive_pose.motor_data.speed_rpm[2], comprehensive_pose.motor_data.speed_rpm[3]);



//           UART_SendInt16_WithLabel(&huart1,".direction", comprehensive_pose.motor_data.direction[3]);
//           UART_SendInt16_WithLabel(&huart1,"speed_rpm",comprehensive_pose.motor_data.speed_rpm[3] );
   

    //出临界区
     taskEXIT_CRITICAL();

    return HAL_UART_Transmit(&huart5, (uint8_t*)pose_msg, strlen(pose_msg), 0xFFFF);
}

/**
 * @brief 检查姿态是否发生变化
 * @return uint8_t 1-发生变化，0-未发生变化
 *
 * 检查舵机角度是否发生变化，可根据需要扩展检查IMU数据变化
 */
uint8_t CheckPoseChanged(void)
{
    static ComprehensivePose_t last_pose = {0};
    uint8_t changed = 0;

    // 进入临界区
    taskENTER_CRITICAL() ;

    // 检查舵机角度是否发生变化
    for(int i = 0; i < 16; i++) {
        if(comprehensive_pose.servo_pose.angles[i] != last_pose.servo_pose.angles[i]) {
            changed = 1;
            break;
        }
    }

    // 如果舵机角度没变，检查电机数据是否发生变化
    if(!changed) {
        for(int i = 0; i < 4; i++) {
            if(comprehensive_pose.motor_data.direction[i] != last_pose.motor_data.direction[i] ||
               comprehensive_pose.motor_data.speed_rpm[i] != last_pose.motor_data.speed_rpm[i]) {
                changed = 1;
                break;
            }
        }
    }

    //出临界区
     taskEXIT_CRITICAL();

    // 如果舵机角度或电机数据没变，检查IMU数据是否发生变化（可选）
    if(!changed) {
        // 可以根据需要决定是否检查IMU数据变化
        // 这里暂时只检查舵机角度和电机数据变化
    }

    if(changed) {
        // 更新last_pose
        for(int i = 0; i < 16; i++) {
            last_pose.servo_pose.angles[i] = comprehensive_pose.servo_pose.angles[i];
        }
        last_pose.servo_pose.timestamp = comprehensive_pose.servo_pose.timestamp;

        // 同时更新电机数据
        for(int i = 0; i < 4; i++) {
            last_pose.motor_data.direction[i] = comprehensive_pose.motor_data.direction[i];
            last_pose.motor_data.speed_rpm[i] = comprehensive_pose.motor_data.speed_rpm[i];
        }
        last_pose.motor_data.timestamp = comprehensive_pose.motor_data.timestamp;
    }

    return changed;
}

/* USER CODE END 2 */

/* USER CODE BEGIN 3 */

// 定期清理接收缓冲区的函数，防止长时间累积数据
void Jetson_ClearStaleData(void)
{
    // 如果接收索引长时间未被重置且数值较大，则强制清空
    if(uart5_rx_index > 50) {  // 调整阈值以平衡响应速度和稳定性
        // 重置接收索引
        uart5_rx_index = 0;
    }
}
/* USER CODE END 3 */

/* USER CODE BEGIN 4 */

// 更新IMU数据
void UpdateIMUData(float accel_x, float accel_y, float accel_z,
                   float gyro_x, float gyro_y, float gyro_z,
                   float temperature)
{
    comprehensive_pose.imu_data.accel_x_g = accel_x;
    comprehensive_pose.imu_data.accel_y_g = accel_y;
    comprehensive_pose.imu_data.accel_z_g = accel_z;
    comprehensive_pose.imu_data.gyro_x_dps = gyro_x;
    comprehensive_pose.imu_data.gyro_y_dps = gyro_y;
    comprehensive_pose.imu_data.gyro_z_dps = gyro_z;
    comprehensive_pose.imu_data.temperature_c = temperature;
    comprehensive_pose.timestamp = HAL_GetTick();
}

// 更新电机数据（使用分离的方向和速度）
void UpdateMotorData(int8_t dir1, int8_t dir2, int8_t dir3, int8_t dir4,
                     float rpm1, float rpm2, float rpm3, float rpm4)
{
    // 一次性更新所有电机数据
    comprehensive_pose.motor_data.direction[0] = dir1;
    comprehensive_pose.motor_data.direction[1] = dir2;
    comprehensive_pose.motor_data.direction[2] = dir3;
    comprehensive_pose.motor_data.direction[3] = dir4;

    comprehensive_pose.motor_data.speed_rpm[0] = rpm1;
    comprehensive_pose.motor_data.speed_rpm[1] = rpm2;
    comprehensive_pose.motor_data.speed_rpm[2] = rpm3;
    comprehensive_pose.motor_data.speed_rpm[3] = rpm4;

    comprehensive_pose.motor_data.timestamp = HAL_GetTick();
    comprehensive_pose.timestamp = HAL_GetTick(); // 更新整体时间戳

    //出临界区
     taskEXIT_CRITICAL();
    
}



// 更新电机数据（使用带符号速度）
void UpdateMotorDataWithSignedSpeed(float signed_rpm1, float signed_rpm2, float signed_rpm3, float signed_rpm4)
{

    // 处理第一个电机
    int8_t dir1 = 0;
    float rpm1 = 0.0f;
    // 添加小的阈值来避免在0附近的抖动
    const float DIRECTION_THRESHOLD = 0.1f;  // 方向判断阈值，可根据需要调整
    if(signed_rpm1 > DIRECTION_THRESHOLD) {
        dir1 = 1;
        rpm1 = signed_rpm1;
    } else if(signed_rpm1 < -DIRECTION_THRESHOLD) {
        dir1 = -1;
        rpm1 = -signed_rpm1;  // 取绝对值
    } else {
        dir1 = 0;  // 当速度接近0时，方向为0
        rpm1 = 0.0f;
    }

    // 处理第二个电机
    int8_t dir2 = 0;
    float rpm2 = 0.0f;
    if(signed_rpm2 > 0.0f) {
        dir2 = 1;
        rpm2 = signed_rpm2;
    } else if(signed_rpm2 < 0.0f) {
        dir2 = -1;
        rpm2 = -signed_rpm2;  // 取绝对值
    } else {
        dir2 = 0;
        rpm2 = 0.0f;
    }

    // 处理第三个电机
    int8_t dir3 = 0;
    float rpm3 = 0.0f;
    if(signed_rpm3 > 0.0f) {
        dir3 = 1;
        rpm3 = signed_rpm3;
    } else if(signed_rpm3 < 0.0f) {
        dir3 = -1;
        rpm3 = -signed_rpm3;  // 取绝对值
    } else {
        dir3 = 0;
        rpm3 = 0.0f;
    }
    
  



    // 处理第四个电机
    int8_t dir4 = 0;
    float rpm4 = 0.0f;
    if(signed_rpm4 > 0.0f) {
        dir4 = 1;
        rpm4 = signed_rpm4;
    } else if(signed_rpm4 < 0.0f) {
        dir4 = -1;
        rpm4 = -signed_rpm4;  // 取绝对值
    } else {
        dir4 = 0;
        rpm4 = 0.0f;
    }


//    if(dir4>0)//调试输出false
//    {
//        UART_SendInt16_WithLabel(&huart1,"inside_direction", dir4);
//    }
    
    // 一次性更新所有电机数据
    //进入临界区
    taskENTER_CRITICAL();
    comprehensive_pose.motor_data.direction[0] = dir1;
    comprehensive_pose.motor_data.direction[1] = dir2;
    comprehensive_pose.motor_data.direction[2] = dir3;
    comprehensive_pose.motor_data.direction[3] = dir4;

    comprehensive_pose.motor_data.speed_rpm[0] = rpm1;
    comprehensive_pose.motor_data.speed_rpm[1] = rpm2;
    comprehensive_pose.motor_data.speed_rpm[2] = rpm3;
    comprehensive_pose.motor_data.speed_rpm[3] = rpm4;

    comprehensive_pose.motor_data.timestamp = HAL_GetTick();
    comprehensive_pose.timestamp = HAL_GetTick(); // 更新整体时间戳

    // 出临界区
    taskEXIT_CRITICAL() ;

}

// 更新单个电机数据
void UpdateSingleMotorData(uint8_t motor_id, int8_t direction, float speed_rpm)
{
    // 进入临界区
    taskENTER_CRITICAL();
    if(motor_id < 4) {
        comprehensive_pose.motor_data.direction[motor_id] = direction;
        comprehensive_pose.motor_data.speed_rpm[motor_id] = speed_rpm;

        comprehensive_pose.motor_data.timestamp = HAL_GetTick();
        comprehensive_pose.timestamp = HAL_GetTick(); // 更新整体时间戳
    }

    // 出临界区
    taskEXIT_CRITICAL() ;
}

// 更新单个电机数据（使用带符号速度）
void UpdateSingleMotorDataWithSignedSpeed(uint8_t motor_id, float signed_speed_rpm)
{

      



    if(motor_id < 4) {
        int8_t direction = 0;
        float abs_speed = 0.0f;
        if(signed_speed_rpm > 0.0f) {
            direction = 1;      // 正转
            abs_speed = signed_speed_rpm;
        } else if(signed_speed_rpm < 0.0f) {
            direction = -1;     // 反转
            abs_speed = -signed_speed_rpm;  // 取绝对值
        } else {
            direction = 0;      // 停止
            abs_speed = 0.0f;
        }
        // 进入临界区
        taskENTER_CRITICAL();
        
        comprehensive_pose.motor_data.direction[motor_id] = direction;
        comprehensive_pose.motor_data.speed_rpm[motor_id] = abs_speed;

//    if(motor_id==3)//调试输出pass
//    {
//        UART_SendInt16_WithLabel(&huart1,"direction", direction);
//        UART_SendInt16_WithLabel(&huart1,"abs_speed",abs_speed );
//    }
            
        comprehensive_pose.motor_data.timestamp = HAL_GetTick();
        comprehensive_pose.timestamp = HAL_GetTick(); // 更新整体时间戳
        
        // 出临界区
        taskEXIT_CRITICAL();
    }


    
}

// 获取上位机发来的电机控制命令
void GetMotorCommand(uint8_t motor_id, int8_t *direction, float *speed_rpm)
{
    if(motor_id < 4 && direction != NULL && speed_rpm != NULL) {
        // 进入临界区
        taskENTER_CRITICAL() ;

        *direction = motor_cmd_direction[motor_id];
        *speed_rpm = motor_cmd_speed[motor_id];

        //出临界区
         taskEXIT_CRITICAL();
    }
}

/**
  * @brief  Initialize UART communication with Jetson
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef JetsonUart_Init(void)
{
    // 初始化接收缓冲区
    uart5_rx_index = 0;
    uart5_frame_complete = 0;

    return HAL_OK;
}

// 获取上位机发来的电机命令类型
int8_t GetMotorCommandType(uint8_t motor_id)
{
    int8_t result = 0; // 默认返回停止

    if(motor_id < 4) {
        // 进入临界区
        taskENTER_CRITICAL() ;

        result = (int8_t)motor_cmd_type[motor_id];

        //出临界区
         taskEXIT_CRITICAL(); }
    

    return result;
}

/* USER CODE END 4 */




/**
  * @brief  Send heartbeat message to Jetson
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Jetson_SendHeartbeat(void)
{
    char heartbeat_msg[] = "HB\r\n";
    return HAL_UART_Transmit(&huart5, (uint8_t*)heartbeat_msg, strlen(heartbeat_msg), 0xFFFF);
}

/**
  * @brief  Send data message to Jetson
  * @param  data: Data to send
  * @param  length: Length of data
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Jetson_SendData(uint8_t *data, uint8_t length)
{
    if(length > 64) {
        return HAL_ERROR; // 数据长度超出限制
    }

    // 简单的数据发送，实际应用中可能需要添加协议头等
    return HAL_UART_Transmit(&huart5, data, length, 0xFFFF);
}

/**
  * @brief  Send acknowledgment message to Jetson
  * @param  msg_id: Message ID to acknowledge
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Jetson_SendAck(uint8_t msg_id)
{
    char ack_msg[20];
    sprintf(ack_msg, "ACK:%d\r\n", msg_id);
    return HAL_UART_Transmit(&huart5, (uint8_t*)ack_msg, strlen(ack_msg), 0xFFFF);
}

/**
  * @brief  Send error message to Jetson
  * @param  error_code: Error code
  * @param  error_msg: Error message string
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Jetson_SendError(uint8_t error_code, const char* error_msg)
{
    char err_msg[50];
    sprintf(err_msg, "ERR:%d:%s\r\n", error_code, error_msg);
    return HAL_UART_Transmit(&huart5, (uint8_t*)err_msg, strlen(err_msg), 0xFFFF);
}

/**
  * @brief  Process received data from Jetson
  * @param  data: Received data
  * @param  length: Length of received data
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef Jetson_ProcessReceivedData(uint8_t *data, uint8_t length)
{
    // 处理接收到的数据
    // 这里可以实现具体的协议解析逻辑
    for(uint8_t i = 0; i < length; i++) {
        // 检查是否是换行符，表示消息结束
        if(data[i] == '\n' && uart5_rx_index > 0 && uart5_rx_buffer[uart5_rx_index-1] == '\r') {
            // 完整的消息到达 (\r\n)
            uart5_rx_buffer[uart5_rx_index] = '\0'; // 添加字符串结束符
            uart5_frame_complete = 1;

            // 尝试解析为舵机控制消息
            if(ParseServoControlMessage(uart5_rx_buffer, uart5_rx_index-2) != HAL_OK) { // -2 to exclude \r\n
                // 如果不是舵机控制消息，尝试解析为电机控制消息
                if(ParseMotorControlMessage(uart5_rx_buffer, uart5_rx_index-2) != HAL_OK) {
                    // 如果都不是，可以选择输出或忽略
                    // 对于调试，可以取消下面的注释
                    /*
                    char debug_msg[260];  // 调整缓冲区大小以适应可能的完整消息
                    sprintf(debug_msg, "Unknown command: %s", uart5_rx_buffer);
                    HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 0xFFFF);
                    */
                }
            }

            // 这里可以添加消息处理逻辑
            // 例如：解析命令、执行相应操作等

            // 重置接收索引
            uart5_rx_index = 0;
        }
        // 检查是否是换行符但前面没有回车符的情况（兼容性处理）
        else if(data[i] == '\n' && uart5_rx_index > 0) {
            // 部分兼容：单独的\n也被认为是消息结束
            uart5_rx_buffer[uart5_rx_index] = '\0';
            uart5_frame_complete = 1;

            // 尝试解析为舵机控制消息
            if(ParseServoControlMessage(uart5_rx_buffer, uart5_rx_index-1) != HAL_OK) { // -1 to exclude \n
                // 如果不是舵机控制消息，尝试解析为电机控制消息
                if(ParseMotorControlMessage(uart5_rx_buffer, uart5_rx_index-1) != HAL_OK) {
                    // 如果都不是，可以选择输出或忽略
                    // 对于调试，可以取消下面的注释
                    /*
                    char debug_msg[260];
                    sprintf(debug_msg, "Unknown command: %s", uart5_rx_buffer);
                    HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 0xFFFF);
                    */
                }
            }

            // 重置接收索引
            uart5_rx_index = 0;
        }
        // 检查是否是回车符，但后面可能没有换行符
        else if(data[i] == '\r' && uart5_rx_index > 0) {
            // 检查是否已经有内容，如果是则视为消息结束
            uart5_rx_buffer[uart5_rx_index] = '\0';
            uart5_frame_complete = 1;

            // 通过UART4输出接收到的数据（调试用）- 只在完整消息时输出
            char debug_msg[260];
            sprintf(debug_msg, "Message: %s", uart5_rx_buffer);
            HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 0xFFFF);

            // 重置接收索引
            uart5_rx_index = 0;
        }
        else {
            // 检查缓冲区空间是否足够
            if(uart5_rx_index < sizeof(uart5_rx_buffer)-1) {
                // 将数据添加到接收缓冲区，但不输出
                uart5_rx_buffer[uart5_rx_index++] = data[i];
            }
            else {
                // 缓冲区溢出，重置
                uart5_rx_index = 0;
                // 添加错误信息输出
                char error_msg[] = "UART5 Buffer Overflow\r\n";
                HAL_UART_Transmit(&huart4, (uint8_t*)error_msg, strlen(error_msg), 0xFFFF);
            }
        }
    }

    return HAL_OK;
}

/**
  * @brief  Get pointer to receive buffer
  * @retval uint8_t*: Pointer to receive buffer
  */
uint8_t* Jetson_GetRxBuffer(void)
{
    return uart5_rx_buffer;
}

/**
  * @brief  Get length of received data
  * @retval uint8_t: Length of received data
  */
uint8_t Jetson_GetRxLength(void)
{
    return uart5_rx_index;
}

/**
  * @brief  Reset receive state
  * @retval None
  */
void Jetson_ResetRxState(void)
{
    uart5_rx_index = 0;
    uart5_frame_complete = 0;
}

/* USER CODE BEGIN 1 */
/*jetson_uart的任务函数*/
void Jetson_Task(void *argument)
{
  /* USER CODE BEGIN Jetson_Task */
  char debug_msg[100];

  // 用于100Hz姿态回传的时间跟踪
  uint32_t last_pose_transmit = 0;
  const uint32_t POSE_TRANSMIT_INTERVAL = 10; // 10ms = 100Hz

  // 初始化UART通信
  JetsonUart_Init();

  // 等待UART完全初始化
  osDelay(100);

  // 启动UART5中断接收
  if(JetsonUart_StartReceiveIT() != HAL_OK) {
      sprintf(debug_msg, "Failed to start UART5 IT receive\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 0xFFFF);
  } else {
      sprintf(debug_msg, "UART5 IT receive started successfully\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 0xFFFF);
  }

  // 等待系统稳定
  osDelay(1000);

  // 发送初始化完成消息
  sprintf(debug_msg, "Jetson Comm Task Started at 100Hz\r\n");
  HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 0xFFFF);

  /* Infinite loop */
  for(;;)
  {
    // 100Hz通信速率，即每10ms执行一次循环
    // 数据接收由中断处理，这里可以做其他事情

    // 更新所有电机数据到综合姿态信息
    extern void UpdateMotorData(int8_t dir1, int8_t dir2, int8_t dir3, int8_t dir4,
                               float rpm1, float rpm2, float rpm3, float rpm4);
   

    // 使用带符号速度更新电机数据
    osMutexAcquire(MotorStatusMutexHandle,portMAX_DELAY);
    float signed_speed_0 = all_motors[0].speed_rpm;
    float signed_speed_1 = all_motors[1].speed_rpm;
    float signed_speed_2 = all_motors[2].speed_rpm;
    float signed_speed_3 = all_motors[3].speed_rpm;

    
//    UART_SendInt16_WithLabel(&huart1,"before_direction", all_motors[3].direction );
//    UART_SendFloat_WithLabel_Manual(&huart1,"before_speed",  all_motors[3].speed_rpm);
    
//    if(all_motors[0].direction == -1) signed_speed_0 = -signed_speed_0;//多处理了一步
//    if(all_motors[1].direction == -1) signed_speed_1 = -signed_speed_1;
//    if(all_motors[2].direction == -1) signed_speed_2 = -signed_speed_2;
//    if(all_motors[3].direction == -1) signed_speed_3 = -signed_speed_3;
    osMutexRelease(MotorStatusMutexHandle);
    
    //调试false
    //UART_SendInt16_WithLabel(&huart1,"before_direction", signed_speed_3);
    // UART_SendFloat_WithLabel_Manual(&huart1,"after_speed", signed_speed_3);

    
    UpdateMotorDataWithSignedSpeed(signed_speed_0, signed_speed_1, signed_speed_2, signed_speed_3);

    osDelay(10);

    // 检查接收超时并处理
    Jetson_CheckReceiveTimeout();
    // 定期清理可能的陈旧数据
    Jetson_ClearStaleData();

    // 检查姿态是否发生变化，如果变化则发送
    if(CheckPoseChanged()) {
        // 发送当前姿态
        SendCurrentPose();
        last_pose_transmit = HAL_GetTick();
    }
    // 同时保持最小发送间隔，避免过于频繁发送
    else if((HAL_GetTick() - last_pose_transmit) >= POSE_TRANSMIT_INTERVAL) {
        // 即使没有变化，也定期发送以维持通信
        SendCurrentPose();
        last_pose_transmit = HAL_GetTick();
    }
  }
  /* USER CODE END Jetson_Task */
}
/* USER CODE END 1 */