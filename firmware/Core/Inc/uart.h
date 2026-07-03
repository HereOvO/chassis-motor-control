#ifndef __UART_H__
#define __UART_H__



//=================include===========================
#include "Motor.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"  //定义了TickType_t
#include "queue.h"  //定义了TickType_t
#include "main.h"
#include "cmsis_os.h"
#include "math.h"

//=================extern PV===========================
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim12;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;




// ==================== 数据结构定义 =================




//=======================函数===========================
/* UART任务函数 */ 
extern void UART4_Task(void *argument);

HAL_StatusTypeDef UART1_SendUint16_Text(UART_HandleTypeDef* uarthandle,uint16_t data);
HAL_StatusTypeDef UART_SendFloat_WithLabel_Manual(UART_HandleTypeDef* uarthandle, const char* label, float data);

#endif