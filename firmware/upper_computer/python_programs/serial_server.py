import serial
import threading
import time
import json
import zmq
from datetime import datetime
import logging

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class SerialServer:
    def __init__(self, port='COM11', baudrate=115200):
        """
        串口中间服务器
        :param port: 串口名称
        :param baudrate: 波特率
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn = None
        self.running = False
        
        # ZeroMQ上下文和套接字
        self.context = zmq.Context()
        self.publisher = self.context.socket(zmq.PUB)  # 用于发布串口接收的数据
        self.subscriber = self.context.socket(zmq.SUB)  # 用于订阅要发送到串口的数据
        self.publisher.bind("tcp://*:5555")  # 发布端口
        self.subscriber.bind("tcp://*:5556")  # 订阅端口
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, "send_to_serial")  # 订阅发送到串口的消息
        
        # 用于同步的锁
        self.lock = threading.Lock()

    def connect_serial(self):
        """连接串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,  # 8位数据位
                parity=serial.PARITY_EVEN,  # 偶校验，总共9位（8数据+1校验）
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            logger.info(f"成功连接到串口 {self.port}")
            return True
        except Exception as e:
            logger.error(f"连接串口失败: {e}")
            return False

    def disconnect_serial(self):
        """断开串口连接"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            logger.info("已断开串口连接")

    def read_from_serial(self):
        """从串口读取数据并发布到ZMQ"""
        buffer = ""
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    if self.serial_conn.in_waiting > 0:
                        data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                        buffer += data
                        
                        # 查找完整的行（以\n结尾）
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            if '\r' in line:
                                line = line.replace('\r', '')

                            if line.strip():  # 如果不是空行
                                # 发布接收到的数据
                                message = {
                                    'type': 'received_from_serial',
                                    'data': line.strip(),
                                    'timestamp': datetime.now().isoformat()
                                }
                                self.publisher.send_string(json.dumps(message))
                                logger.info(f"转发串口数据: {line.strip()}")
                                
                    time.sleep(0.01)  # 短暂休眠，避免CPU占用过高
                else:
                    time.sleep(1)  # 等待串口重连
            except Exception as e:
                logger.error(f"读取串口数据时出错: {e}")
                time.sleep(1)

    def send_to_serial(self, data):
        """向串口发送数据"""
        with self.lock:
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    # 确保数据以换行符结尾
                    if not data.endswith('\n'):
                        data += '\n'

                    # 发送数据，校验位由硬件自动处理
                    self.serial_conn.write(data.encode('utf-8'))
                    logger.info(f"发送到串口: {data.strip()}")
                    return True
                else:
                    logger.warning("串口未连接，无法发送数据")
                    return False
            except Exception as e:
                logger.error(f"发送数据到串口时出错: {e}")
                return False

    def handle_outgoing_messages(self):
        """处理要发送到串口的消息"""
        while self.running:
            try:
                # 非阻塞接收消息
                try:
                    message = self.subscriber.recv_string(flags=zmq.NOBLOCK)
                    if message.startswith("send_to_serial"):
                        # 提取要发送的数据
                        _, data = message.split(" ", 1)
                        self.send_to_serial(data)
                except zmq.Again:
                    # 没有消息，短暂休眠
                    time.sleep(0.01)
            except Exception as e:
                logger.error(f"处理发送消息时出错: {e}")
                time.sleep(0.01)

    def start(self):
        """启动服务器"""
        if not self.connect_serial():
            return False
            
        self.running = True
        
        # 启动读取线程
        read_thread = threading.Thread(target=self.read_from_serial, daemon=True)
        read_thread.start()
        
        # 启动处理发送消息的线程
        send_thread = threading.Thread(target=self.handle_outgoing_messages, daemon=True)
        send_thread.start()
        
        logger.info("串口服务器已启动")
        
        try:
            # 主循环
            while self.running:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("收到中断信号，正在关闭...")
        finally:
            self.stop()

    def stop(self):
        """停止服务器"""
        self.running = False
        self.disconnect_serial()
        self.publisher.close()
        self.subscriber.close()
        self.context.term()
        logger.info("串口服务器已停止")


if __name__ == "__main__":
    # 创建并启动串口服务器
    server = SerialServer(port='COM11', baudrate=115200)  # 根据实际情况修改端口
    server.start()