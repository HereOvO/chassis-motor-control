import zmq
import json
import time
import threading
from datetime import datetime
import sys

class CommandSender:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.connect("tcp://localhost:5556")  # 连接到服务器的订阅端口
        time.sleep(0.1)  # 等待连接建立

    def send_command(self, command):
        """发送命令到串口"""
        # 确保数据以换行符结尾
        if not command.endswith('\n'):
            command += '\n'
        message = f"send_to_serial {command}"
        self.socket.send_string(message)
        print(f"已发送命令: {command.strip()}")

    def send_servo_command(self, servo_id, angle, duration=1000):
        """发送舵机控制命令"""
        command = f"SERVO,{servo_id},{angle},{duration}"
        self.send_command(command)

    def send_motor_command(self, motor_id, signed_speed_rpm):
        """发送电机控制命令
        signed_speed_rpm: 正值=正转, 负值=反转, 0=停止
        """
        command = f"MOTOR,{motor_id},{signed_speed_rpm}"
        self.send_command(command)

    def send_custom_command(self, custom_command):
        """发送自定义命令"""
        self.send_command(custom_command)

    def interactive_menu(self):
        """交互式菜单"""
        print("\n" + "="*60)
        print("串口命令发送客户端")
        print("="*60)
        print("选择操作:")
        print("1. 发送舵机控制命令 (SERVO)")
        print("2. 发送电机控制命令 (MOTOR)")
        print("3. 发送自定义命令")
        print("4. 发送心跳包 (HB)")
        print("5. 发送测试数据")
        print("0. 退出")
        print("-"*60)

    def run_interactive(self):
        """运行交互式界面"""
        while True:
            try:
                self.interactive_menu()
                choice = input("请选择操作 (0-5): ").strip()
                
                if choice == '0':
                    print("退出程序...")
                    break
                elif choice == '1':
                    # 发送舵机控制命令
                    try:
                        servo_id = int(input("输入舵机ID (0-15): "))
                        angle = int(input("输入角度 (0-180): "))
                        duration = int(input("输入持续时间 (ms, 默认1000): ") or "1000")
                        
                        if 0 <= servo_id <= 15 and 0 <= angle <= 180:
                            self.send_servo_command(servo_id, angle, duration)
                        else:
                            print("输入值超出范围!")
                    except ValueError:
                        print("请输入有效的数字!")
                        
                elif choice == '2':
                    # 发送电机控制命令
                    try:
                        motor_id = int(input("输入电机ID (0-3): "))
                        print("转速: 正值=正转, 负值=反转, 0=停止")
                        signed_speed_rpm = float(input("输入带符号的转速 (RPM): "))

                        if 0 <= motor_id <= 3:
                            self.send_motor_command(motor_id, signed_speed_rpm)
                        else:
                            print("输入值超出范围!")
                    except ValueError:
                        print("请输入有效的数字!")
                        
                elif choice == '3':
                    # 发送自定义命令
                    custom_cmd = input("输入自定义命令: ")
                    if custom_cmd.strip():
                        self.send_custom_command(custom_cmd.strip())
                    else:
                        print("命令不能为空!")
                        
                elif choice == '4':
                    # 发送心跳包
                    self.send_custom_command("HB")
                    
                elif choice == '5':
                    # 发送测试数据
                    print("发送测试数据...")
                    # 测试舵机命令
                    self.send_servo_command(0, 90, 1000)
                    time.sleep(0.1)
                    # 测试电机命令
                    self.send_motor_command(0, 50.5)   # 正转
                    time.sleep(0.1)
                    self.send_motor_command(0, -30.0)   # 反转
                    time.sleep(0.1)
                    self.send_motor_command(0, 0.0)    # 停止
                    time.sleep(0.1)
                    # 测试混合命令
                    self.send_custom_command("TEST MESSAGE")
                    
                else:
                    print("无效选择，请重新输入!")
                    
                print()  # 空行
                
            except KeyboardInterrupt:
                print("\n用户中断，退出程序...")
                break
            except Exception as e:
                print(f"发生错误: {e}")

    def stop(self):
        """停止发送"""
        self.socket.close()
        self.context.term()


def demo_commands(sender):
    """演示各种命令"""
    print("开始演示命令发送...")
    
    # 发送一些测试命令
    print("\n1. 发送舵机控制命令:")
    sender.send_servo_command(0, 90, 1000)  # 控制0号舵机转到90度，用时1000ms
    time.sleep(1)
    sender.send_servo_command(1, 45, 500)   # 控制1号舵机转到45度，用时500ms
    time.sleep(1)
    sender.send_servo_command(0, 0, 1000)   # 控制0号舵机转到0度
    
    print("\n2. 发送电机控制命令:")
    sender.send_motor_command(0, 50.5)   # 控制0号电机正转，转速50.5 RPM
    time.sleep(1)
    sender.send_motor_command(0, -30.0)   # 控制0号电机反转，转速30.0 RPM
    time.sleep(1)
    sender.send_motor_command(0, 0.0)    # 控制0号电机停止
    
    print("\n3. 发送其他命令:")
    sender.send_custom_command("HB")         # 心跳包
    time.sleep(0.5)
    sender.send_custom_command("TEST")       # 测试命令
    
    print("\n演示完成!")


if __name__ == "__main__":
    sender = CommandSender()
    
    if len(sys.argv) > 1 and sys.argv[1] == "--demo":
        # 运行演示模式
        demo_commands(sender)
    else:
        # 运行交互模式
        print("等待串口服务器启动... (请先运行 serial_server.py)")
        time.sleep(2)
        sender.run_interactive()
    
    sender.stop()