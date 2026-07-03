/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bsp_mpu6050.h
  * @brief          : Header for bsp_mpu6050.c file.
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
#ifndef __BSP_MPU6050_H
#define __BSP_MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
    int16_t raw_temp;
    
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float temperature_c;
} MPU6050_Data_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define MPU6050_ADDR                    0x68
#define MPU6050_ADDR_WRITE              (MPU6050_ADDR << 1)
#define MPU6050_ADDR_READ               ((MPU6050_ADDR << 1) | 0x01)

#define MPU6050_REG_ACCEL_XOUT_H        0x3B
#define MPU6050_REG_PWR_MGMT_1          0x6B
#define MPU6050_REG_CONFIG              0x1A
#define MPU6050_REG_GYRO_CONFIG         0x1B
#define MPU6050_REG_ACCEL_CONFIG        0x1C
#define MPU6050_REG_FIFO_EN             0x23
#define MPU6050_REG_INT_ENABLE          0x38
#define MPU6050_REG_INT_PIN_CFG         0x37
#define MPU6050_REG_USER_CTRL           0x6A
#define MPU6050_REG_FIFO_COUNT_H        0x72
#define MPU6050_REG_FIFO_R_W            0x74
#define MPU6050_REG_WHO_AM_I            0x75

#define MPU6050_I2C_TIMEOUT             100
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
HAL_StatusTypeDef BSP_MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BSP_MPU6050_Read_Data(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data);
HAL_StatusTypeDef BSP_MPU6050_Read_Raw_Data(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data);
void BSP_MPU6050_Print_Data(MPU6050_Data_t *data);
extern UART_HandleTypeDef huart4;
void UART_Send_Float_Custom(UART_HandleTypeDef *huart, float num, uint8_t decimals);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __BSP_MPU6050_H */
