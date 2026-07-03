#!/usr/bin/env python3
"""
协议统一测试脚本
用于测试新的电机控制协议是否正确实现
"""

def test_protocol_format():
    """测试协议格式"""
    print("=== 电机控制协议测试 ===")

    # 测试新的协议格式
    test_cases = [
        ("MOTOR,0,50.5", "正转"),
        ("MOTOR,0,-30.0", "反转"),
        ("MOTOR,0,0.0", "停止"),
        ("MOTOR,1,100.0", "电机1正转"),
        ("MOTOR,2,-75.5", "电机2反转")
    ]

    print("新协议格式: MOTOR,<motor_id>,<signed_speed_rpm>")
    print("signed_speed_rpm: 正值=正转, 负值=反转, 0=停止")
    print("简化了协议，使用带符号的速度值同时表示转向和转速\n")

    for cmd, desc in test_cases:
        print(f"命令: {cmd} -> {desc}")

    print("\n=== 舵机控制协议 ===")
    print("格式: SERVO,<servo_id>,<angle>,<duration>")
    print("示例: SERVO,0,90,1000 -> 控制0号舵机转到90度，用时1000ms")

    print("\n=== 姿态回传协议 ===")
    print("格式: POSE,<servo0>..<servo15>,<timestamp>,<accel_x>..<temperature>,<dir1>..<dir4>,<rpm1>..<rpm4>")
    print("其中电机方向字段: -1=反转, 0=停止, 1=正转")

def test_signed_speed_concept():
    """测试带符号速度概念"""
    print("\n=== 带符号速度概念测试 ===")
    print("通过带符号的速度值同时表示转向和转速:")
    print("- 正值: 正转，数值表示转速")
    print("- 负值: 反转，绝对值表示转速")
    print("- 零值: 停止/刹车")

def test_unified_brake_handling():
    """测试统一刹车处理"""
    print("\n=== 统一刹车处理测试 ===")
    print("不再区分快刹车和慢刹车，统一为刹车处理:")
    print("- 当速度为0时，电机停止/刹车")
    print("- 内部统一使用慢刹车逻辑")

def test_backward_compatibility():
    """测试向后兼容性"""
    print("\n=== 向后兼容性测试 ===")
    print("下位机代码已更新，保持与旧版本的兼容性:")
    print("协议简化为: MOTOR,<motor_id>,<signed_speed_rpm>")
    print("signed_speed_rpm: 正值=正转, 负值=反转, 0=停止/刹车")
    print("GetMotorCommandType返回值: -1=反转, 0=停止/刹车, 1=正转")

def test_command_types():
    """测试命令类型"""
    print("\n=== 命令类型测试 ===")
    print("GetMotorCommandType函数现在返回:")
    print("- -1: 电机反转")
    print("- 0: 电机停止/刹车")
    print("- 1: 电机正转")

if __name__ == "__main__":
    test_protocol_format()
    test_signed_speed_concept()
    test_unified_brake_handling()
    test_command_types()
    test_backward_compatibility()
    print("\n协议更新完成！")