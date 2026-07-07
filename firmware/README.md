# STM32F407 底盘电机控制固件

## 当前定位

本工程当前维护的是基于 `STM32F407VET6` 的四轮底盘电机控制固件。活跃构建与运行链路已经聚焦于：

- `USART1` 命令接收与反馈输出
- 四路编码器采样
- 四路电机双向 PWM 输出
- `FreeRTOS` 主控制任务调度
- 底盘运动学、里程计、PID + 前馈控制

以下旧内容已从当前活跃工程中移除：

- `MPU6050 / IMU` 采集链路
- `PCA9685` 舵机驱动链路
- `I2C1 / I2C2` 外设初始化与对应驱动源码
- `UART5` 舵机/姿态回传实现链路

原版通用通信协议不再作为当前固件说明的一部分，但协议定义已原样保留在 `LEGACY_PROTOCOL_REFERENCE.md`，仅用于和当前底盘协议做对照，不参与当前活跃固件说明。

## 硬件资源

- **主控芯片**: `STM32F407VET6`
- **系统时钟**: `168MHz`
- **命令/反馈串口**: `USART1`，`PA9(TX)` / `PA10(RX)`
- **编码器输入**: `TIM2`、`TIM3`、`TIM4`、`TIM5`
- **电机 PWM 输出**: `TIM1`、`TIM9`、`TIM12`
- **HAL 时基**: `TIM14`
- **未纳入当前活跃构建**: `I2C1`、`I2C2`、`UART5`

电机索引顺序：

- `0 = left-front`
- `1 = left-rear`
- `2 = right-rear`
- `3 = right-front`

## 软件架构

- **开发环境**: `STM32CubeMX + Keil MDK-ARM`
- **实时操作系统**: `FreeRTOS`
- **主控制任务**: `ChassisControlTask`
- **主循环周期**: `1 ms`
- **编码器采样 / 里程计更新**: `50 Hz`
- **PID + 前馈计算**: `100 Hz`
- **串口反馈上报**: `100 ms`

当前运行链路：

```text
USART1 RX interrupt
-> chassis_protocol_poll()
-> chassis_mode_apply_cmd()
-> chassis_kinematics_inverse()
-> MotorControlCore_SetTargets()
-> MotorControlCore_Compute()
-> MotorDriver_SetOutput()
-> encoder feedback
-> chassis_odometry_update()
```

## 当前协议状态

当前默认构建使用正式二进制协议：

```c
#define CHASSIS_USE_DEBUG_PROTOCOL 0U
```

可在调试联调时切回保留的 ASCII 调试协议：

```c
#define CHASSIS_USE_DEBUG_PROTOCOL 1U
```

其中 `CHASSIS_USE_DEBUG_PROTOCOL` 现在只决定上电默认协议。运行中可通过独立 sideband 切换帧在 `ASCII` 与 `MOWEN` 之间切换，不会占用正常正式协议或调试协议命令空间：

```text
FE EF 50 MM CC FD
MM = 00 -> ASCII
MM = 01 -> MOWEN
CC = (0x50 + MM) & 0xFF
```

当前协议与上位机说明请优先阅读：

- `MOTOR_MIGRATION_PROJECT.md`
- `docx/formal_protocol_status.md`
- `upper_computer/python_programs/README.md`

原版通用 `SERVO` / `POSE` 协议定义保留在：

- `LEGACY_PROTOCOL_REFERENCE.md`

## 构建环境

- **IDE**: `Keil MDK-ARM uVision 5`
- **编译器**: `ARMCC 5.05 update 1`
- **工程文件**: `MDK-ARM/wheeled_leg_robot_f407.uvprojx`
- **目标名**: `wheeled_leg_robot_f407`

参考命令：

```powershell
D:\Keil_v5\UV4\uVision.com -r MDK-ARM\wheeled_leg_robot_f407.uvprojx -t wheeled_leg_robot_f407 -o MDK-ARM\keil_rebuild.log
```

## 文档索引

- **当前固件构建与运行说明**: `MOTOR_MIGRATION_PROJECT.md`
- **正式二进制协议状态记录**: `docx/formal_protocol_status.md`
- **上位机脚本与串口联调说明**: `upper_computer/python_programs/README.md`
- **原版通用协议对照保留**: `LEGACY_PROTOCOL_REFERENCE.md`
