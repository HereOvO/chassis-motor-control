/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MotorDirectionControl_IO1_1_Pin GPIO_PIN_9
#define MotorDirectionControl_IO1_1_GPIO_Port GPIOE
#define MotorDirectionControl_IO1_2_Pin GPIO_PIN_5
#define MotorDirectionControl_IO1_2_GPIO_Port GPIOE
#define MotorDirectionControl_IO2_1_Pin GPIO_PIN_11
#define MotorDirectionControl_IO2_1_GPIO_Port GPIOE
#define MotorDirectionControl_IO2_2_Pin GPIO_PIN_6
#define MotorDirectionControl_IO2_2_GPIO_Port GPIOE
#define MotorDirectionControl_IO3_1_Pin GPIO_PIN_13
#define MotorDirectionControl_IO3_1_GPIO_Port GPIOE
#define MotorDirectionControl_IO3_2_Pin GPIO_PIN_14
#define MotorDirectionControl_IO3_2_GPIO_Port GPIOB
#define MotorDirectionControl_IO4_1_Pin GPIO_PIN_14
#define MotorDirectionControl_IO4_1_GPIO_Port GPIOE
#define MotorDirectionControl_IO4_2_Pin GPIO_PIN_15
#define MotorDirectionControl_IO4_2_GPIO_Port GPIOB
#define MotorEncoderSignal_1C1_Pin GPIO_PIN_15
#define MotorEncoderSignal_1C1_GPIO_Port GPIOA
#define MotorEncoderSignal_1C2_Pin GPIO_PIN_3
#define MotorEncoderSignal_1C2_GPIO_Port GPIOB
#define MotorEncoderSignal_2C1_Pin GPIO_PIN_6
#define MotorEncoderSignal_2C1_GPIO_Port GPIOC
#define MotorEncoderSignal_2C2_Pin GPIO_PIN_7
#define MotorEncoderSignal_2C2_GPIO_Port GPIOC
#define MotorEncoderSignal_3C1_Pin GPIO_PIN_12
#define MotorEncoderSignal_3C1_GPIO_Port GPIOD
#define MotorEncoderSignal_3C2_Pin GPIO_PIN_13
#define MotorEncoderSignal_3C2_GPIO_Port GPIOD
#define MotorEncoderSignal_4C1_Pin GPIO_PIN_0
#define MotorEncoderSignal_4C1_GPIO_Port GPIOA
#define MotorEncoderSignal_4C2_Pin GPIO_PIN_1
#define MotorEncoderSignal_4C2_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
