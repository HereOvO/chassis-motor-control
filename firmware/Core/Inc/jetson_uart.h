/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : jetson_uart.h
  * @brief          : Header for jetson_uart.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __JETSON_UART_H
#define __JETSON_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"  /*!< 包含HAL库定义 */
#include "cmsis_os.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
// 通信消息类型定义
typedef enum {
    MSG_TYPE_HEARTBEAT = 0,    // 心跳包
    MSG_TYPE_DATA = 1,         // 数据传输
    MSG_TYPE_CMD = 2,          // 命令传输
    MSG_TYPE_ACK = 3,          // 确认消息
    MSG_TYPE_ERROR = 4         // 错误消息
} MessageType_t;

// 通信消息结构体
typedef struct {
    MessageType_t type;        // 消息类型
    uint8_t id;               // 消息ID
    uint8_t length;           // 数据长度
    uint8_t data[64];         // 数据载荷
} JetsonMessage_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
// 初始化UART通信
HAL_StatusTypeDef JetsonUart_Init(void);

// 启动UART中断接收
HAL_StatusTypeDef JetsonUart_StartReceiveIT(void);

// 发送心跳消息
HAL_StatusTypeDef Jetson_SendHeartbeat(void);

// 发送数据消息
HAL_StatusTypeDef Jetson_SendData(uint8_t *data, uint8_t length);

// 发送确认消息
HAL_StatusTypeDef Jetson_SendAck(uint8_t msg_id);

// 发送错误消息
HAL_StatusTypeDef Jetson_SendError(uint8_t error_code, const char* error_msg);

// 接收消息处理
HAL_StatusTypeDef Jetson_ProcessReceivedData(uint8_t *data, uint8_t length);

// 获取接收缓冲区指针
uint8_t* Jetson_GetRxBuffer(void);

// 获取接收数据长度
uint8_t Jetson_GetRxLength(void);

// 重置接收状态
void Jetson_ResetRxState(void);

// UART接收完成回调函数（供HAL调用）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

// 检查接收超时的函数
void Jetson_CheckReceiveTimeout(void);

// 定期清理接收缓冲区的函数
void Jetson_ClearStaleData(void);

// 发送当前姿态消息
HAL_StatusTypeDef SendCurrentPose(void);

// 检查姿态是否发生变化
uint8_t CheckPoseChanged(void);

// 更新IMU数据
void UpdateIMUData(float accel_x, float accel_y, float accel_z,
                   float gyro_x, float gyro_y, float gyro_z,
                   float temperature);

// 初始化电机数据互斥锁
void InitMotorDataMutex(void);

// 更新电机数据
void UpdateMotorData(int8_t dir1, int8_t dir2, int8_t dir3, int8_t dir4,
                     float rpm1, float rpm2, float rpm3, float rpm4);

// 更新电机数据（使用带符号速度）
void UpdateMotorDataWithSignedSpeed(float signed_rpm1, float signed_rpm2, float signed_rpm3, float signed_rpm4);

// 更新单个电机数据
void UpdateSingleMotorData(uint8_t motor_id, int8_t direction, float speed_rpm);

// 更新单个电机数据（使用带符号速度）
void UpdateSingleMotorDataWithSignedSpeed(uint8_t motor_id, float signed_speed_rpm);

// 获取上位机发来的电机控制命令
void GetMotorCommand(uint8_t motor_id, int8_t *direction, float *speed_rpm);

// 获取上位机发来的电机命令类型
int8_t GetMotorCommandType(uint8_t motor_id);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __JETSON_UART_H */
