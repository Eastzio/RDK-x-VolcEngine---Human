#!/usr/bin/env python
#
# *********     舵机控制系统     *********
#
# 基于原有SCServo控制代码扩展实现多种功能模式
# 包括多舵机同步控制、示教模式、动作组执行和调试模式

import sys
import os
import time
import json

sys.path.append("..")
from scservo_sdk import *

# 全局变量
SERVO_IDS = [1, 2, 3, 4, 5, 6, 7]  # 7个舵机的ID
action_groups = {}  # 保存动作组的字典

def read_servo(servo_id):
    """读取单个舵机的当前位置和速度直到到达目标位置"""
    while 1:
        # 读取舵机当前位置
        scs_present_position, scs_present_speed, scs_comm_result, scs_error = packetHandler.ReadPosSpeed(servo_id)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))
        else:
            print("[ID:%03d] PresPos:%d PresSpd:%d" % (servo_id, scs_present_position, scs_present_speed))
        if scs_error != 0:
            print(packetHandler.getRxPacketError(scs_error))

        # 读取舵机运动状态
        moving, scs_comm_result, scs_error = packetHandler.ReadMoving(servo_id)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))

        if moving == 0:
            break
    return scs_present_position

def read_all_servos():
    """读取所有舵机的位置"""
    positions = []
    for servo_id in SERVO_IDS:
        pos, _, scs_comm_result, scs_error = packetHandler.ReadPosSpeed(servo_id)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))
            positions.append(0)  # 读取失败时使用默认值
        else:
            positions.append(pos)
        if scs_error != 0:
            print(packetHandler.getRxPacketError(scs_error))
    return positions

def wait_all_servos_stop():
    """等待所有舵机运动完成"""
    while True:
        all_stopped = True
        for servo_id in SERVO_IDS:
            moving, scs_comm_result, scs_error = packetHandler.ReadMoving(servo_id)
            if scs_comm_result != COMM_SUCCESS:
                print(packetHandler.getTxRxResult(scs_comm_result))
            if scs_error != 0:
                print(packetHandler.getRxPacketError(scs_error))
            if moving != 0:
                all_stopped = False
                break
        if all_stopped:
            break
        time.sleep(0.1)

def set_torque(enable):
    """设置所有舵机的扭矩状态"""
    for servo_id in SERVO_IDS:
        # 使用write1ByteTxRx方法写入扭矩使能寄存器
        # SMS_STS_TORQUE_ENABLE = 40 (从SDK可知)
        scs_comm_result, scs_error = packetHandler.write1ByteTxRx(servo_id, 40, 1 if enable else 0)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        if scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))
    
    if enable:
        print("已启用所有舵机扭矩")
    else:
        print("已禁用所有舵机扭矩")


def save_action_groups():
    """保存动作组到文件"""
    with open('action_groups.json', 'w', encoding='utf-8') as f:
        json.dump(action_groups, f, ensure_ascii=False, indent=4)
    print("动作组已保存到文件")

def load_action_groups():
    """从文件加载动作组"""
    global action_groups
    try:
        with open('action_groups.json', 'r', encoding='utf-8') as f:
            action_groups = json.load(f)
        print("动作组已从文件加载")
    except FileNotFoundError:
        print("未找到动作组文件，使用空动作组")
        action_groups = {}

def mode_1_sync_control():
    """模式1: 同步控制7个舵机的位置"""
    print("=== 模式1: 同步控制舵机位置 ===")
    positions = []
    
    # 用户输入每个舵机的目标位置
    print("请输入7个舵机的目标位置 (0-4095)：")
    for i, servo_id in enumerate(SERVO_IDS):
        while True:
            try:
                pos = int(input(f"舵机 ID {servo_id} 的位置: "))
                if 0 <= pos <= 4095:
                    positions.append(pos)
                    break
                else:
                    print("位置必须在0-4095范围内！")
            except ValueError:
                print("请输入有效的数字！")
    
    # 设置速度和加速度
    speed = 1800
    accel = 200
    
    # 同步写入所有舵机位置
    print("正在控制舵机移动到指定位置...")
    for i, servo_id in enumerate(SERVO_IDS):
        scs_comm_result, scs_error = packetHandler.WritePosEx(servo_id, positions[i], speed, accel)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        elif scs_error != 0:
            print("%s" % packetHandler.getRxPacketError(scs_error))
    
    # 等待所有舵机到达目标位置
    wait_all_servos_stop()
    print("所有舵机已到达指定位置")

def mode_2_teaching():
    """模式2: 示教模式，记录舵机位置"""
    print("=== 模式2: 示教模式 ===")
    
    # 禁用扭矩，允许手动调整舵机位置
    set_torque(False)
    
    print("已禁用舵机扭矩锁，请手动调整舵机到所需位置")
    print("调整完成后，输入'123'记录当前位置")
    
    user_input = input("输入 ('123'记录, 'q'退出): ")
    if user_input == '123':
        # 读取所有舵机的当前位置
        positions = read_all_servos()
        print("当前舵机位置:", positions)
        
        # 获取动作组名称
        action_name = input("请为此动作组命名: ")
        
        # 保存到动作组字典
        action_groups[action_name] = positions
        print(f"动作组 '{action_name}' 已保存")
        
        # 保存动作组到文件
        save_action_groups()
    
    # 重新启用扭矩
    set_torque(True)

def mode_3_execute_action():
    """模式3: 执行已保存的动作组"""
    print("=== 模式3: 执行动作组 ===")
    
    # 显示可用的动作组
    if not action_groups:
        print("没有保存的动作组")
        return
    
    print("可用的动作组:")
    for name in action_groups.keys():
        print(f"- {name}")
    
    # 用户选择动作组
    action_name = input("请输入要执行的动作组名称: ")
    if action_name not in action_groups:
        print(f"未找到动作组 '{action_name}'")
        return
    
    positions = action_groups[action_name]
    print(f"执行动作组 '{action_name}': {positions}")
    
    # 设置速度和加速度
    speed = 1700
    accel = 200
    
    # 写入所有舵机位置
    for i, servo_id in enumerate(SERVO_IDS):
        if i < len(positions):  # 确保索引有效
            scs_comm_result, scs_error = packetHandler.WritePosEx(servo_id, positions[i], speed, accel)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(scs_comm_result))
            elif scs_error != 0:
                print("%s" % packetHandler.getRxPacketError(scs_error))
    
    # 等待所有舵机到达目标位置
    wait_all_servos_stop()
    print("动作执行完成")

def mode_4_debug():
    """模式4: 调试模式"""
    print("=== 模式4: 调试模式 ===")
    
    print("调试选项:")
    print("1. 读取所有舵机位置")
    print("2. 测试单个舵机")
    print("3. 显示所有动作组")
    print("4. 删除动作组")
    print("5. 重置所有舵机到初始位置")
    
    choice = input("请选择操作: ")
    
    if choice == '1':
        # 读取所有舵机位置
        positions = read_all_servos()
        for i, servo_id in enumerate(SERVO_IDS):
            print(f"舵机 ID {servo_id}: 位置 = {positions[i]}")
    
    elif choice == '2':
        # 测试单个舵机
        try:
            servo_id = int(input("输入要测试的舵机ID (1-7): "))
            if servo_id not in SERVO_IDS:
                print("无效的舵机ID")
                return
            
            pos = int(input("输入目标位置 (0-4095): "))
            if not 0 <= pos <= 4095:
                print("位置必须在0-4095范围内")
                return
            
            # 控制舵机
            scs_comm_result, scs_error = packetHandler.WritePosEx(servo_id, pos, 60, 50)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(scs_comm_result))
            elif scs_error != 0:
                print("%s" % packetHandler.getRxPacketError(scs_error))
            
            # 等待舵机到达位置
            read_servo(servo_id)
            print("测试完成")
        except ValueError:
            print("请输入有效数字")
    
    elif choice == '3':
        # 显示所有动作组
        if not action_groups:
            print("没有保存的动作组")
        else:
            print("所有保存的动作组:")
            for name, positions in action_groups.items():
                print(f"{name}: {positions}")
    
    elif choice == '4':
        # 删除动作组
        if not action_groups:
            print("没有保存的动作组")
            return
        
        print("可删除的动作组:")
        for name in action_groups.keys():
            print(f"- {name}")
        
        action_name = input("请输入要删除的动作组名称: ")
        if action_name in action_groups:
            del action_groups[action_name]
            print(f"动作组 '{action_name}' 已删除")
            save_action_groups()
        else:
            print(f"未找到动作组 '{action_name}'")
    
    elif choice == '5':
        # 重置所有舵机到初始位置
        print("重置所有舵机到初始位置 (2048)...")
        for servo_id in SERVO_IDS:
            scs_comm_result, scs_error = packetHandler.WritePosEx(servo_id, 2048, 60, 50)
            if scs_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(scs_comm_result))
            elif scs_error != 0:
                print("%s" % packetHandler.getRxPacketError(scs_error))
        
        wait_all_servos_stop()
        print("所有舵机已重置")
    
    else:
        print("无效的选择")

def main():
    global portHandler, packetHandler
    
    # 初始化PortHandler实例
    # 设置端口路径
    # 获取PortHandlerLinux或PortHandlerWindows的方法和成员
    portHandler = PortHandler('/dev/human_arm')  # 例如Windows: "COM1"，Linux: "/dev/ttyUSB0", Mac: "/dev/tty.usbserial-*"

    # 初始化PacketHandler实例
    # 获取协议的方法和成员
    packetHandler = sms_sts(portHandler)
    
    # 打开端口
    if portHandler.openPort():
        print("成功打开端口")
    else:
        print("打开端口失败")
        return
    
    # 设置端口波特率1000000
    if portHandler.setBaudRate(1000000):
        print("成功更改波特率")
    else:
        print("更改波特率失败")
        portHandler.closePort()
        return
    
    # 加载保存的动作组
    load_action_groups()
    
    # 主循环
    try:
        while True:
            print("\n===== 舵机控制系统 =====")
            print("1. 同步控制模式")
            print("2. 示教模式")
            print("3. 执行动作组模式")
            print("4. 调试模式")
            print("0. 退出程序")
            
            mode = input("请选择模式: ")
            
            if mode == '1':
                mode_1_sync_control()
            elif mode == '2':
                mode_2_teaching()
            elif mode == '3':
                mode_3_execute_action()
            elif mode == '4':
                mode_4_debug()
            elif mode == '0':
                break
            else:
                print("无效的模式选择，请重试")
    
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    
    finally:
        # 关闭端口
        portHandler.closePort()
        print("端口已关闭，程序结束")

if __name__ == "__main__":
    main()
