import zmq
import json
import time
from datetime import datetime
import threading
import sys

class DataReceiver:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect("tcp://localhost:5555")  # 连接到服务器的发布端口
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")  # 订阅所有消息
        
        # 解析结果存储
        self.parsed_data = {}
        self.running = True

    def parse_pose_message(self, data):
        """解析姿态数据 (POSE消息)"""
        try:
            parts = data.split(',')
            if len(parts) >= 25 and parts[0] == 'POSE':
                parsed = {
                    'type': 'POSE',
                    'servo_angles': [int(x) for x in parts[1:17]],  # 16个舵机角度
                    'timestamp': int(parts[17]),  # 时间戳
                    'imu_data': {
                        'accel_x': float(parts[18]),
                        'accel_y': float(parts[19]),
                        'accel_z': float(parts[20]),
                        'gyro_x': float(parts[21]),
                        'gyro_y': float(parts[22]),
                        'gyro_z': float(parts[23]),
                        'temperature': float(parts[24])
                    },
                    'motor_data': {
                        'directions': [int(x) for x in parts[25:29]],  # 4个电机方向
                        'speeds': [float(x) for x in parts[29:33]]  # 4个电机转速
                    }
                }
                return parsed
        except Exception as e:
            print(f"解析POSE消息失败: {e}")
        return None

    def parse_heartbeat_message(self, data):
        """解析心跳消息 (HB消息)"""
        if data == 'HB':
            return {'type': 'HEARTBEAT'}
        return None

    def parse_ack_message(self, data):
        """解析确认消息 (ACK消息)"""
        if data.startswith('ACK:'):
            try:
                msg_id = int(data.split(':')[1])
                return {'type': 'ACK', 'msg_id': msg_id}
            except:
                pass
        return None

    def parse_error_message(self, data):
        """解析错误消息 (ERR消息)"""
        if data.startswith('ERR:'):
            try:
                parts = data.split(':', 2)
                error_code = int(parts[1])
                error_msg = parts[2] if len(parts) > 2 else "Unknown error"
                return {'type': 'ERROR', 'error_code': error_code, 'error_msg': error_msg}
            except:
                pass
        return None

    def parse_servo_control_message(self, data):
        """解析舵机控制消息 (SERVO消息)"""
        if data.startswith('SERVO,'):
            try:
                parts = data.split(',')
                servo_id = int(parts[1])
                angle = int(parts[2])
                duration = int(parts[3])
                return {'type': 'SERVO_CONTROL', 'servo_id': servo_id, 'angle': angle, 'duration': duration}
            except:
                pass
        return None

    def parse_motor_control_message(self, data):
        """解析电机控制消息 (MOTOR消息)"""
        if data.startswith('MOTOR,'):
            try:
                parts = data.split(',')
                motor_id = int(parts[1])
                signed_speed_rpm = float(parts[2])
                return {'type': 'MOTOR_CONTROL', 'motor_id': motor_id, 'signed_speed_rpm': signed_speed_rpm}
            except:
                pass
        return None

    def parse_message(self, data):
        """解析不同类型的消息"""
        parsers = [
            self.parse_pose_message,
            self.parse_heartbeat_message,
            self.parse_ack_message,
            self.parse_error_message,
            self.parse_servo_control_message,
            self.parse_motor_control_message
        ]
        
        for parser in parsers:
            result = parser(data)
            if result:
                return result
        
        # 如果没有匹配任何预定义格式，返回原始数据
        return {'type': 'UNKNOWN', 'raw_data': data}

    def display_parsed_data(self, parsed_data):
        """显示解析后的数据"""
        print("\n" + "=" * 60)
        print(f"时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
        print(f"消息类型: {parsed_data['type']}")

        if parsed_data['type'] == 'POSE':
            print("姿态数据:")
            print(f"  舵机角度: {parsed_data['servo_angles']}")
            print(f"  时间戳: {parsed_data['timestamp']}")
            print(f"  IMU数据: 加速度({parsed_data['imu_data']['accel_x']:.2f}, "
                  f"{parsed_data['imu_data']['accel_y']:.2f}, {parsed_data['imu_data']['accel_z']:.2f}), "
                  f"陀螺仪({parsed_data['imu_data']['gyro_x']:.2f}, "
                  f"{parsed_data['imu_data']['gyro_y']:.2f}, {parsed_data['imu_data']['gyro_z']:.2f}), "
                  f"温度: {parsed_data['imu_data']['temperature']:.2f}°C")

            # 电机数据显示改进
            print("  电机数据:")
            if 'motor_data' in parsed_data and 'directions' in parsed_data['motor_data']:
                directions = parsed_data['motor_data']['directions']
                speeds = parsed_data['motor_data']['speeds']
                for i, (direction, speed) in enumerate(zip(directions, speeds)):
                    if direction == -1:
                        dir_str = "反转"
                    elif direction == 1:
                        dir_str = "正转"
                    else:
                        dir_str = "停止"
                    print(f"    电机{i}: {dir_str}, 转速: {speed:.1f} RPM")

        elif parsed_data['type'] == 'MOTOR_CONTROL':
            signed_speed = parsed_data['signed_speed_rpm']
            direction = parsed_data.get('direction', None)

            # 优先使用direction字段（如果有）
            if direction is not None:
                if direction == -1:
                    direction_str = "反转"
                elif direction == 1:
                    direction_str = "正转"
                else:
                    direction_str = "停止"
            else:
                # 根据signed_speed判断方向
                if signed_speed < 0:
                    direction_str = "反转"
                    speed_display = abs(signed_speed)
                elif signed_speed > 0:
                    direction_str = "正转"
                    speed_display = signed_speed
                else:
                    direction_str = "停止"
                    speed_display = 0

            print(f"  电机控制 - ID: {parsed_data['motor_id']}, "
                  f"方向: {direction_str}, 转速: {speed_display:.1f} RPM")

        # ... 其他类型保持不变
        print("=" * 60)

    def start_receiving(self):
        """开始接收数据"""
        print("开始接收数据... 按 Ctrl+C 退出")
        try:
            while self.running:
                try:
                    # 非阻塞接收
                    message_json = self.socket.recv_string(flags=zmq.NOBLOCK)
                    message = json.loads(message_json)
                    
                    if message['type'] == 'received_from_serial':
                        raw_data = message['data']
                        print(f"[{message['timestamp']}] 接收到原始数据: {raw_data}")
                        
                        # 解析数据
                        parsed_data = self.parse_message(raw_data)
                        if parsed_data:
                            self.display_parsed_data(parsed_data)
                
                except zmq.Again:
                    # 没有新消息，短暂休眠
                    time.sleep(0.01)
                except KeyboardInterrupt:
                    print("\n用户中断，停止接收...")
                    break
                    
        except Exception as e:
            print(f"接收数据时出错: {e}")
        finally:
            self.stop()

    def stop(self):
        """停止接收"""
        self.running = False
        self.socket.close()
        self.context.term()


if __name__ == "__main__":
    receiver = DataReceiver()
    receiver.start_receiving()