# 轮式腿机器人上位机通信程序

本项目包含三个Python程序，用于与STM32单片机通过UART5进行通信。

## 架构说明

由于多个程序不能同时直接访问同一个物理串口，我们采用了中间服务器架构：

1. **serial_server.py** - 串口中间服务器（核心）
   - 负责与物理串口(COM11)通信
   - 使用ZeroMQ发布/订阅模式分发数据
   - 一个程序独占串口资源，避免冲突

2. **receiver_client.py** - 数据接收客户端
   - 订阅来自单片机的数据
   - 解析并显示各种消息类型

3. **sender_client.py** - 命令发送客户端  
   - 向单片机发送控制命令
   - 提供交互式界面

## 安装依赖

```bash
pip install pyzmq pyserial
```

## 使用方法

### 1. 启动串口服务器（必须首先运行）

```bash
python serial_server.py
```

### 2. 启动接收客户端（在另一个终端）

```bash
python receiver_client.py
```

### 3. 启动发送客户端（在另一个终端）

```bash
python sender_client.py
```

## 消息格式

### 接收的消息类型：
- **POSE**: 姿态数据 - `POSE,<servo0>,<servo1>,...,<servo15>,<timestamp>,<accel_x>,<accel_y>,<accel_z>,<gyro_x>,<gyro_y>,<gyro_z>,<temperature>,<dir1>,<dir2>,<dir3>,<dir4>,<rpm1>,<rpm2>,<rpm3>,<rpm4>`
- **HB**: 心跳包 - `HB`
- **ACK**: 确认消息 - `ACK:<msg_id>`
- **ERR**: 错误消息 - `ERR:<error_code>:<error_msg>`

### 发送的命令类型：
- **SERVO**: 舵机控制 - `SERVO,<servo_id>,<angle>,<duration>`
- **MOTOR**: 电机控制 - `MOTOR,<motor_id>,<signed_speed_rpm>`
  - `signed_speed_rpm`: 正值=正转, 负值=反转, 0=停止/刹车

## 配置说明

- 默认串口：COM11（可在serial_server.py中修改）
- 发布端口：TCP 5555（接收数据）
- 订阅端口：TCP 5556（发送数据）

## 注意事项

1. 必须先启动serial_server.py，再启动其他客户端
2. 确保指定的串口号在您的系统中存在且未被其他程序占用
3. 如果需要使用COM9或COM12，请修改serial_server.py中的端口号
4. 发送客户端提供交互式菜单，方便发送各种控制命令
5. 接收客户端会自动解析和格式化接收到的数据

## 示例命令

### 舵机控制
```
SERVO,0,90,1000    # 控制0号舵机转到90度，用时1000ms
```

### 电机控制
```
MOTOR,0,50.5     # 控制0号电机正转，转速50.5 RPM
MOTOR,0,-30.0    # 控制0号电机反转，转速30.0 RPM
MOTOR,0,0.0      # 控制0号电机停止
```