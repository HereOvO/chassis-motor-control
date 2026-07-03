#ifndef __PCA9685_H
#define __PCA9685_H

//=================include===========================
#include "Motor.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"  //定义了TickType_t
#include "queue.h"  //定义了TickType_t
#include "main.h"
#include "cmsis_os.h"
#include "math.h"
#include <string.h>
#include <stdio.h>

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

/* PCA9685 I2C Address */
#define PCA9685_ADDR              0x80  // 0x40 << 1
#define PCA9685_MODE1             0x00
#define PCA9685_PRESCALE          0xFE
#define PCA9685_LED0_ON_L         0x06
#define PCA9685_LED0_ON_H         0x07
#define PCA9685_LED0_OFF_L        0x08
#define PCA9685_LED0_OFF_H        0x09

/* Function declarations */
void PCA9685_Init(I2C_HandleTypeDef *hi2c, float freq);
void PCA9685_SetPWM(I2C_HandleTypeDef *hi2c, uint8_t num, uint32_t on, uint32_t off);
void PCA9685_SetDuty(I2C_HandleTypeDef *hi2c, uint8_t num, float duty);
void PCA9685_ServoControl(I2C_HandleTypeDef *hi2c, uint8_t num, uint8_t start_angle, uint8_t end_angle, uint8_t speed);
void PCA9685_SetAngle(I2C_HandleTypeDef *hi2c, uint8_t num, uint8_t angle);



#endif
