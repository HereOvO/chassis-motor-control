/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_mpu6050.c
  * @brief          : MPU6050 driver implementation
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
#include "bsp_mpu6050.h"
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint8_t mpu6050_data_buffer[14];
static uint8_t mpu6050_wake_data = 0x00;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initialize MPU6050 sensor
  * @param  hi2c: I2C handle
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BSP_MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    
    // Wake up MPU6050
    status = HAL_I2C_Mem_Write(hi2c, 
                               MPU6050_ADDR_WRITE, 
                               MPU6050_REG_PWR_MGMT_1, 
                               I2C_MEMADD_SIZE_8BIT, 
                               &mpu6050_wake_data, 
                               1, 
                               MPU6050_I2C_TIMEOUT);
    
    if(status != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    HAL_Delay(100);  // Wait for MPU6050 to stabilize
    
    // Additional configuration can be added here
    // Configure accelerometer range (±2g)
    uint8_t accel_config = 0x00; // ±2g range
    status = HAL_I2C_Mem_Write(hi2c, 
                               MPU6050_ADDR_WRITE, 
                               MPU6050_REG_ACCEL_CONFIG, 
                               I2C_MEMADD_SIZE_8BIT, 
                               &accel_config, 
                               1, 
                               MPU6050_I2C_TIMEOUT);
    
    if(status != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    // Configure gyroscope range (±250°/s)
    uint8_t gyro_config = 0x00; // ±250°/s range
    status = HAL_I2C_Mem_Write(hi2c, 
                               MPU6050_ADDR_WRITE, 
                               MPU6050_REG_GYRO_CONFIG, 
                               I2C_MEMADD_SIZE_8BIT, 
                               &gyro_config, 
                               1, 
                               MPU6050_I2C_TIMEOUT);
    
    if(status != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
  * @brief  Read raw data from MPU6050
  * @param  hi2c: I2C handle
  * @param  data: Pointer to MPU6050 data structure
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BSP_MPU6050_Read_Raw_Data(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data)
{
    HAL_StatusTypeDef status;
    
    // Read 14 bytes of data starting from ACCEL_XOUT_H register
    status = HAL_I2C_Mem_Read(hi2c, 
                              MPU6050_ADDR_READ, 
                              MPU6050_REG_ACCEL_XOUT_H, 
                              I2C_MEMADD_SIZE_8BIT, 
                              mpu6050_data_buffer, 
                              14, 
                              MPU6050_I2C_TIMEOUT);
    
    if(status != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    // Parse raw data
    data->raw_accel_x = (int16_t)((mpu6050_data_buffer[0] << 8) | mpu6050_data_buffer[1]);
    data->raw_accel_y = (int16_t)((mpu6050_data_buffer[2] << 8) | mpu6050_data_buffer[3]);
    data->raw_accel_z = (int16_t)((mpu6050_data_buffer[4] << 8) | mpu6050_data_buffer[5]);
    data->raw_temp = (int16_t)((mpu6050_data_buffer[6] << 8) | mpu6050_data_buffer[7]);
    data->raw_gyro_x = (int16_t)((mpu6050_data_buffer[8] << 8) | mpu6050_data_buffer[9]);
    data->raw_gyro_y = (int16_t)((mpu6050_data_buffer[10] << 8) | mpu6050_data_buffer[11]);
    data->raw_gyro_z = (int16_t)((mpu6050_data_buffer[12] << 8) | mpu6050_data_buffer[13]);
    
    return HAL_OK;
}

/**
  * @brief  Read processed data from MPU6050
  * @param  hi2c: I2C handle
  * @param  data: Pointer to MPU6050 data structure
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BSP_MPU6050_Read_Data(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data)
{
    HAL_StatusTypeDef status;
    
    status = BSP_MPU6050_Read_Raw_Data(hi2c, data);
    if(status != HAL_OK)
    {
        return status;
    }
    
    // Convert raw data to physical values
    data->accel_x_g = data->raw_accel_x / 16384.0f;  // Assuming ±2g range
    data->accel_y_g = data->raw_accel_y / 16384.0f;
    data->accel_z_g = data->raw_accel_z / 16384.0f;
    
    data->gyro_x_dps = data->raw_gyro_x / 131.0f;   // Assuming ±250°/s range
    data->gyro_y_dps = data->raw_gyro_y / 131.0f;
    data->gyro_z_dps = data->raw_gyro_z / 131.0f;
    
    data->temperature_c = data->raw_temp / 340.0f + 36.53f;
    
    return HAL_OK;
}

/**
  * @brief  Print MPU6050 data via UART
  * @param  data: Pointer to MPU6050 data structure
  * @retval None
  */
void BSP_MPU6050_Print_Data(MPU6050_Data_t *data)
{
    uint8_t buffer[100];
    uint16_t len;
    
    // Print accelerometer data
    len = sprintf((char*)buffer, "accel_x_g: ");
    HAL_UART_Transmit(&huart4, buffer, len, 100);
    UART_Send_Float_Custom(&huart4, data->accel_x_g, 2);

    len = sprintf((char*)buffer, "accel_y_g: ");
    HAL_UART_Transmit(&huart4, buffer, len, 100);
    UART_Send_Float_Custom(&huart4, data->accel_y_g, 2);

    len = sprintf((char*)buffer, "accel_z_g: ");
    HAL_UART_Transmit(&huart4, buffer, len, 100);
    UART_Send_Float_Custom(&huart4, data->accel_z_g, 2);

    // Print temperature
    len = sprintf((char*)buffer, "temperature_c: ");
    HAL_UART_Transmit(&huart4, buffer, len, 100);
    UART_Send_Float_Custom(&huart4, data->temperature_c, 2);

    // Print gyroscope data
    len = sprintf((char*)buffer, "gyro_x_dps: ");
    HAL_UART_Transmit(&huart4, buffer, len, 100);
    UART_Send_Float_Custom(&huart4, data->gyro_x_dps, 2);

    len = sprintf((char*)buffer, "gyro_y_dps: ");
    HAL_UART_Transmit(&huart4, buffer, len, 100);
    UART_Send_Float_Custom(&huart4, data->gyro_y_dps, 2);

    len = sprintf((char*)buffer, "gyro_z_dps: ");
    HAL_UART_Transmit(&huart4, buffer, len, 100);
    UART_Send_Float_Custom(&huart4, data->gyro_z_dps, 2);
    
    // Send newline
    HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n", 2, 100);
}

/**
  * @brief  Send float value via UART with specified decimal places
  * @param  huart: UART handle
  * @param  num: Float number to send
  * @param  decimals: Number of decimal places
  * @retval None
  */
void UART_Send_Float_Custom(UART_HandleTypeDef *huart, float num, uint8_t decimals) 
{
    char buffer[15];
    int32_t integer_part;
    int32_t fractional_part;
    uint8_t i, idx = 0;
    int32_t multiplier = 1;

    // Calculate 10^decimals
    for(i = 0; i < decimals; i++) {
        multiplier *= 10;
    }

    // Handle negative numbers
    if(num < 0) {
        HAL_UART_Transmit(huart, (uint8_t*)"-", 1, 10);
        num = -num;
    }

    // Extract integer and fractional parts
    integer_part = (int32_t)num;
    fractional_part = (int32_t)((num - integer_part) * multiplier);

    // If fractional part is negative (due to floating point precision issues)
    if(fractional_part < 0) {
        fractional_part = -fractional_part;
    }

    // Convert integer part to string
    int32_t temp = integer_part;
    uint8_t int_digits = 0;

    // Count integer digits
    if(temp == 0) {
        int_digits = 1;
    } else {
        while(temp > 0) {
            int_digits++;
            temp /= 10;
        }
    }

    // Convert integer part to string
    temp = integer_part;
    idx = int_digits;
    buffer[idx] = '\0';  // String terminator

    if(temp == 0) {
        buffer[0] = '0';
    } else {
        while(temp > 0) {
            buffer[--idx] = (temp % 10) + '0';  // Fill from back
            temp /= 10;
        }
    }

    // Send integer part
    HAL_UART_Transmit(huart, (uint8_t*)buffer, int_digits, 10);

    // Send decimal point
    HAL_UART_Transmit(huart, (uint8_t*)".", 1, 10);

    // Convert fractional part
    temp = fractional_part;
    uint8_t frac_digits = 0;

    // Count fractional digits
    while(temp > 0) {
        frac_digits++;
        temp /= 10;
    }

    // Add leading zeros if needed
    for(i = frac_digits; i < decimals; i++) {
        HAL_UART_Transmit(huart, (uint8_t*)"0", 1, 10);
    }

    // Convert fractional part to string
    if(fractional_part > 0) {
        temp = fractional_part;
        idx = frac_digits;
        buffer[idx] = '\0';

        while(temp > 0) {
            buffer[--idx] = (temp % 10) + '0';
            temp /= 10;
        }

        // Send fractional part
        HAL_UART_Transmit(huart, (uint8_t*)buffer, frac_digits, 10);
    }

    HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 10);
}

/* USER CODE BEGIN 1 */
/*mpu6050任务函数*/
void Imu_Task(void *argument)
{
  /* USER CODE BEGIN Imu_Task */
  MPU6050_Data_t imu_data;
  char imu_send_buf[100];
  HAL_StatusTypeDef status;

  // 等待系统稳定
  osDelay(1000);

  // 初始化MPU6050传感器
  status = BSP_MPU6050_Init(&hi2c1);
  if(status != HAL_OK)
  {
      // 初始化失败，发送错误信息
      sprintf(imu_send_buf, "MPU6050 Initialization Failed!\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)imu_send_buf, strlen(imu_send_buf), 0xFFFF);
  }
  else
  {
      // 初始化成功，发送确认信息
      sprintf(imu_send_buf, "MPU6050 Initialization Success!\r\n");
      HAL_UART_Transmit(&huart4, (uint8_t*)imu_send_buf, strlen(imu_send_buf), 0xFFFF);
  }

  /* Infinite loop */
  for(;;)
  {
    // 读取MPU6050传感器数据
    status = BSP_MPU6050_Read_Data(&hi2c1, &imu_data);

    if(status == HAL_OK)
    {
        // 格式化并发送IMU数据
        sprintf(imu_send_buf,
                "IMU: Accel[X:%.2fg,Y:%.2fg,Z:%.2fg] Gyro[X:%.2fdps,Y:%.2fdps,Z:%.2fdps] Temp:%.2fc\r\n",
                imu_data.accel_x_g, imu_data.accel_y_g, imu_data.accel_z_g,
                imu_data.gyro_x_dps, imu_data.gyro_y_dps, imu_data.gyro_z_dps,
                imu_data.temperature_c);

        // 更新IMU数据到综合姿态信息
        extern void UpdateIMUData(float accel_x, float accel_y, float accel_z,
                                  float gyro_x, float gyro_y, float gyro_z,
                                  float temperature);
        UpdateIMUData(imu_data.accel_x_g, imu_data.accel_y_g, imu_data.accel_z_g,
                      imu_data.gyro_x_dps, imu_data.gyro_y_dps, imu_data.gyro_z_dps,
                      imu_data.temperature_c);
    }
    else
    {
        sprintf(imu_send_buf, "MPU6050 Read Error!\r\n");
        HAL_UART_Transmit(&huart4, (uint8_t*)imu_send_buf, strlen(imu_send_buf), 0xFFFF);
    }

    // 每隔100ms读取一次数据 (10Hz)
    osDelay(100);
  }
  /* USER CODE END Imu_Task */
}
/* USER CODE END 1 */
