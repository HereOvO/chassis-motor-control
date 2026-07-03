//=================include===========================
#include "Motor.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"  //定义了TickType_t
#include "queue.h"  //定义了TickType_t
#include "main.h"
#include "cmsis_os.h"
#include "math.h"
#include "uart.h"

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
//=================self PV===========================
osMutexId_t UART4mutex_id;
//=====================其它函数==================================


//=====================任务函数==================================
void UART_Task(void *argument)
{
  /* USER CODE BEGIN UART_Task */
  unsigned long cnt = 0;
  char uart_send_buf[50];

  // 等待系统稳定
  osDelay(1000);

  while(1)
  {
      cnt++;
      sprintf(uart_send_buf, "UART Message Count: %lu\r\n", cnt);

      // 发送数据到串口
      HAL_UART_Transmit(&huart4, (uint8_t*)uart_send_buf, strlen(uart_send_buf), 0xFFFF);

      // 每隔1秒发送一次
      osDelay(1000);
  }
  /* USER CODE END UART_Task */
}


//以文本形式发送int16_t
HAL_StatusTypeDef UART_SendInt16_WithLabel(UART_HandleTypeDef* uarthandle, const char* label, int16_t data)
{
    char buffer[32];
    int length;
    
    // 格式化为文本：例如 "Delta: -120\r\n"
    length = snprintf(buffer, sizeof(buffer), "%s: %d\r\n", label, data);
    
    if (length < 0 || length >= sizeof(buffer)) {
        return HAL_ERROR; // 格式化错误
    }
    
    return HAL_UART_Transmit(uarthandle, (uint8_t*)buffer, length, HAL_MAX_DELAY);
}

/* 以文本形式发送float */
HAL_StatusTypeDef UART_SendFloat_WithLabel_Manual(UART_HandleTypeDef* uarthandle, const char* label, float data)
{
    char buffer[64];
    int length;
    
    // 手动拆分整数和小数部分 (处理保留3位小数的情况)
    int32_t int_part = (int32_t)data;
    int32_t dec_part = (int32_t)((data - int_part) * 1000); // 乘1000保留3位
    
    // 处理负数小数部分的绝对值 (例如 -0.123，int=0, dec=-123 -> 显示 -0.123)
    if(dec_part < 0) dec_part = -dec_part;
    
    // 如果整体是负数且整数部分为0 (例如 -0.5)，需要手动加负号
    if (data < 0 && int_part == 0) {
        length = snprintf(buffer, sizeof(buffer), "%s: -%d.%03d\r\n", label, int_part, dec_part);
    } else {
        length = snprintf(buffer, sizeof(buffer), "%s: %d.%03d\r\n", label, int_part, dec_part);
    }
    
    if (length < 0 || length >= sizeof(buffer)) return HAL_ERROR;
    
    return HAL_UART_Transmit(uarthandle, (uint8_t*)buffer, length, HAL_MAX_DELAY);
}