#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Wheeltec Robot Base Controller
独立的轮趣机器人底盘控制脚本
支持Function Calling、图形化界面控制和独立运行

Author: AI Assistant
Date: 2024
"""

import serial
import time
import threading
import struct
import math
import json
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass
from enum import Enum
import sys
import tkinter as tk
from tkinter import ttk, messagebox
from tkinter import font as tkFont

# 常量定义
FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
RECEIVE_DATA_SIZE = 24
SEND_DATA_SIZE = 11
PI = 3.1415926

# IMU数据转换常量 - 根据文档更新
GYROSCOPE_RATIO = 1.0 / 3753.0  # 陀螺仪数据转换为弧度/秒
ACCEL_RATIO = 1672.0  # 加速度计数据转换为m/s²

# 自动回充相关常量
AutoCharge_HEADER = 0x7C
AutoCharge_TAIL = 0x7F
AutoCharge_DATA_SIZE = 8

@dataclass
class VelPosData:
    """速度和位置数据结构"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class IMUData:
    """IMU数据结构"""
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0

@dataclass
class SupersonicData:
    """超声波距离数据结构"""
    distance_a: float = 0.0
    distance_b: float = 0.0
    distance_c: float = 0.0
    distance_d: float = 0.0
    distance_e: float = 0.0
    distance_f: float = 0.0
    distance_g: float = 0.0
    distance_h: float = 0.0

@dataclass
class RobotStatus:
    """机器人状态数据结构"""
    position: VelPosData
    velocity: VelPosData
    imu: IMUData
    power_voltage: float
    supersonic: SupersonicData
    is_charging: bool = False
    charging_current: float = 0.0
    red_signal: int = 0

class ControlMode(Enum):
    """控制模式枚举"""
    MANUAL = "manual"
    AUTO = "auto"
    GUI = "gui"  # 新增GUI控制模式

class WheeltecBaseController:
    """轮趣机器人底盘控制器"""
    
    def __init__(self, port: str = "/dev/wheeltec_controller", baudrate: int = 115200):
        """
        初始化底盘控制器
        
        Args:
            port: 串口设备名
            baudrate: 波特率
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_conn: Optional[serial.Serial] = None
        
        # 控制参数
        self.max_linear_vel = 1.0  # m/s
        self.max_angular_vel = 2.0  # rad/s
        
        # 状态管理
        self.running = False
        self.control_mode = ControlMode.MANUAL
        
        # 数据存储
        self.robot_status = RobotStatus(
            position=VelPosData(),
            velocity=VelPosData(),
            imu=IMUData(),
            power_voltage=0.0,
            supersonic=SupersonicData()
        )
        
        # 线程管理
        self.data_thread: Optional[threading.Thread] = None
        self.data_lock = threading.Lock()
    
    def connect(self) -> bool:
        """
        连接串口
        
        Returns:
            bool: 连接是否成功
        """
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            
            if self.serial_conn.is_open:
                print(f"Serial port {self.port} opened successfully")
                return True
            else:
                print(f"Failed to open serial port {self.port}")
                return False
                
        except Exception as e:
            print(f"Serial connection error: {e}")
            return False
    
    def disconnect(self):
        """断开串口连接"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Serial port closed")
    
    def send_velocity_command(self, linear_x: float, linear_y: float, angular_z: float) -> bool:
        """
        发送速度控制命令
        
        Args:
            linear_x: X轴线速度 (m/s)
            linear_y: Y轴线速度 (m/s)
            angular_z: Z轴角速度 (rad/s)
            
        Returns:
            bool: 发送是否成功
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return False
        
        try:
            # 限制速度范围
            linear_x = max(-self.max_linear_vel, min(self.max_linear_vel, linear_x))
            linear_y = max(-self.max_linear_vel, min(self.max_linear_vel, linear_y))
            angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, angular_z))
            
            # 构建数据包
            data = struct.pack('<Bffff', 
                FRAME_HEADER,
                linear_x,
                linear_y, 
                angular_z,
                0.0  # 保留字段
            )
            
            # 添加校验和
            checksum = sum(data) & 0xFF
            data += struct.pack('<BB', checksum, FRAME_TAIL)
            
            # 发送数据
            self.serial_conn.write(data)
            return True
            
        except Exception as e:
            print(f"Send velocity command error: {e}")
            return False
    
    def parse_received_data(self, data: bytes) -> bool:
        """
        解析接收到的数据
        根据文档6.1节的24字节数据帧格式
        
        Args:
            data: 接收到的字节数据
            
        Returns:
            bool: 解析是否成功
        """
        if len(data) < RECEIVE_DATA_SIZE:
            return False
        
        try:
            # 查找帧头
            header_index = data.find(FRAME_HEADER)
            if header_index == -1 or len(data) < header_index + RECEIVE_DATA_SIZE:
                return False
            
            # 提取数据包
            packet = data[header_index:header_index + RECEIVE_DATA_SIZE]
            
            # 验证帧尾
            if packet[23] != FRAME_TAIL:
                return False
            
            # 验证校验和 (BCC校验 - 前22字节异或)
            checksum = 0
            for i in range(22):
                checksum ^= packet[i]
            if checksum != packet[22]:
                print(f"Checksum error: calculated {checksum:02X}, received {packet[22]:02X}")
                return False
            
            # 解包数据 - 按照文档格式：帧头(1) + flag_stop(1) + 速度(6) + 加速度(6) + 角速度(6) + 电压(2) + 校验(1) + 帧尾(1)
            # 使用小端序解析
            unpacked = struct.unpack('<BBhhhhhhhhhh', packet[:22])
            
            with self.data_lock:
                # 电机使能状态
                motor_enabled = (unpacked[1] == 0x00)
                
                # 更新速度信息 (mm/s -> m/s)
                self.robot_status.velocity.x = unpacked[2] / 1000.0  # X轴速度
                self.robot_status.velocity.y = unpacked[3] / 1000.0  # Y轴速度
                # Z轴角速度需要特殊处理 (放大1000倍存储)
                z_angular_raw = unpacked[4]
                if z_angular_raw >= 32768:  # 处理负数
                    z_angular_raw = z_angular_raw - 65536
                self.robot_status.velocity.z = z_angular_raw / 1000.0  # rad/s
                
                # 更新IMU加速度数据 (原始数据 -> m/s²)
                accel_x_raw = unpacked[5]
                accel_y_raw = unpacked[6] 
                accel_z_raw = unpacked[7]
                
                # 处理有符号数据
                if accel_x_raw >= 32768:
                    accel_x_raw = accel_x_raw - 65536
                if accel_y_raw >= 32768:
                    accel_y_raw = accel_y_raw - 65536
                if accel_z_raw >= 32768:
                    accel_z_raw = accel_z_raw - 65536
                    
                self.robot_status.imu.accel_x = accel_x_raw / ACCEL_RATIO
                self.robot_status.imu.accel_y = accel_y_raw / ACCEL_RATIO
                self.robot_status.imu.accel_z = accel_z_raw / ACCEL_RATIO
                
                # 更新IMU角速度数据 (原始数据 -> rad/s)
                gyro_x_raw = unpacked[8]
                gyro_y_raw = unpacked[9]
                gyro_z_raw = unpacked[10]
                
                # 处理有符号数据
                if gyro_x_raw >= 32768:
                    gyro_x_raw = gyro_x_raw - 65536
                if gyro_y_raw >= 32768:
                    gyro_y_raw = gyro_y_raw - 65536
                if gyro_z_raw >= 32768:
                    gyro_z_raw = gyro_z_raw - 65536
                    
                self.robot_status.imu.gyro_x = gyro_x_raw * GYROSCOPE_RATIO
                self.robot_status.imu.gyro_y = gyro_y_raw * GYROSCOPE_RATIO
                self.robot_status.imu.gyro_z = gyro_z_raw * GYROSCOPE_RATIO
                
                # 更新电池电压信息 (mV -> V)
                voltage_raw = unpacked[11]
                self.robot_status.power_voltage = voltage_raw / 1000.0
                
                # 位置信息在这个协议中不直接提供，保持为0或通过积分计算
                # self.robot_status.position 可以通过速度积分得到
            
            return True
            
        except Exception as e:
            print(f"Parse data error: {e}")
            return False
    
    def data_acquisition_loop(self):
        """数据采集循环"""
        buffer = b''
        
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    # 读取数据
                    if self.serial_conn.in_waiting > 0:
                        new_data = self.serial_conn.read(self.serial_conn.in_waiting)
                        buffer += new_data
                        
                        # 处理完整的数据包
                        while len(buffer) >= RECEIVE_DATA_SIZE:
                            if self.parse_received_data(buffer):
                                buffer = buffer[RECEIVE_DATA_SIZE:]
                            else:
                                buffer = buffer[1:]  # 移除一个字节继续搜索
                
                time.sleep(0.01)  # 100Hz
                
            except Exception as e:
                print(f"Data acquisition error: {e}")
                time.sleep(0.1)
    
    def start(self) -> bool:
        """
        启动控制器
        
        Returns:
            bool: 启动是否成功
        """
        if not self.connect():
            return False
        
        self.running = True
        
        # 启动数据采集线程
        self.data_thread = threading.Thread(target=self.data_acquisition_loop, daemon=True)
        self.data_thread.start()
        
        print("WheeltecBaseController started")
        return True
    
    def stop(self):
        """停止控制器"""
        self.running = False
        
        # 等待线程结束
        if self.data_thread and self.data_thread.is_alive():
            self.data_thread.join(timeout=1.0)
        
        self.disconnect()
        print("WheeltecBaseController stopped")
    
    # ==================== Function Calling 接口 ====================
    
    def move_base(self, linear_x: float, linear_y: float, angular_z: float, duration: float = 0.0) -> Dict:
        """
        控制机器人底盘移动
        
        Args:
            linear_x: X轴线速度 (m/s)
            linear_y: Y轴线速度 (m/s)
            angular_z: Z轴角速度 (rad/s)
            duration: 持续时间 (秒)，0表示持续执行
            
        Returns:
            Dict: 执行结果
        """
        try:
            success = self.send_velocity_command(linear_x, linear_y, angular_z)
            
            if duration > 0:
                time.sleep(duration)
                self.send_velocity_command(0, 0, 0)  # 停止
            
            return {
                "success": success,
                "message": f"Base moved with linear_x={linear_x}, linear_y={linear_y}, angular_z={angular_z}",
                "linear_x": linear_x,
                "linear_y": linear_y,
                "angular_z": angular_z,
                "duration": duration
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"Failed to move base: {str(e)}",
                "error": str(e)
            }
    
    def get_robot_status(self) -> Dict:
        """
        获取机器人状态
        
        Returns:
            Dict: 机器人状态信息
        """
        try:
            with self.data_lock:
                status = {
                    "position": {
                        "x": self.robot_status.position.x,
                        "y": self.robot_status.position.y,
                        "z": self.robot_status.position.z
                    },
                    "velocity": {
                        "x": self.robot_status.velocity.x,
                        "y": self.robot_status.velocity.y,
                        "z": self.robot_status.velocity.z
                    },
                    "imu": {
                        "accel_x": self.robot_status.imu.accel_x,
                        "accel_y": self.robot_status.imu.accel_y,
                        "accel_z": self.robot_status.imu.accel_z,
                        "gyro_x": self.robot_status.imu.gyro_x,
                        "gyro_y": self.robot_status.imu.gyro_y,
                        "gyro_z": self.robot_status.imu.gyro_z
                    },
                    "power_voltage": self.robot_status.power_voltage,
                    "supersonic": {
                        "distance_a": self.robot_status.supersonic.distance_a,
                        "distance_b": self.robot_status.supersonic.distance_b,
                        "distance_c": self.robot_status.supersonic.distance_c,
                        "distance_d": self.robot_status.supersonic.distance_d,
                        "distance_e": self.robot_status.supersonic.distance_e,
                        "distance_f": self.robot_status.supersonic.distance_f,
                        "distance_g": self.robot_status.supersonic.distance_g,
                        "distance_h": self.robot_status.supersonic.distance_h
                    },
                    "is_charging": self.robot_status.is_charging,
                    "charging_current": self.robot_status.charging_current
                }
            
            return {
                "success": True,
                "status": status,
                "timestamp": time.time()
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"Failed to get robot status: {str(e)}",
                "error": str(e)
            }
    
    def stop_base(self) -> Dict:
        """
        停止机器人底盘
        
        Returns:
            Dict: 执行结果
        """
        try:
            success = self.send_velocity_command(0, 0, 0)
            return {
                "success": success,
                "message": "Base stopped"
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"Failed to stop base: {str(e)}",
                "error": str(e)
            }
    
    def set_velocity_limits(self, max_linear: float, max_angular: float) -> Dict:
        """
        设置速度限制
        
        Args:
            max_linear: 最大线速度 (m/s)
            max_angular: 最大角速度 (rad/s)
            
        Returns:
            Dict: 执行结果
        """
        try:
            self.max_linear_vel = max(0.1, min(2.0, max_linear))
            self.max_angular_vel = max(0.1, min(5.0, max_angular))
            
            return {
                "success": True,
                "message": f"Velocity limits set to linear={self.max_linear_vel}, angular={self.max_angular_vel}",
                "max_linear_vel": self.max_linear_vel,
                "max_angular_vel": self.max_angular_vel
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"Failed to set velocity limits: {str(e)}",
                "error": str(e)
            }
    
    def reset_odometry(self) -> Dict:
        """
        重置里程计
        
        Returns:
            Dict: 执行结果
        """
        try:
            with self.data_lock:
                self.robot_status.position.x = 0.0
                self.robot_status.position.y = 0.0
                self.robot_status.position.z = 0.0
            
            return {
                "success": True,
                "message": "Odometry reset"
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"Failed to reset odometry: {str(e)}",
                "error": str(e)
            }

class WheeltecGUIController:
    """轮趣机器人图形化控制界面"""
    
    def __init__(self, controller: WheeltecBaseController):
        self.controller = controller
        self.root = tk.Tk()
        self.root.title("轮趣机器人底盘控制器")
        self.root.geometry("800x600")
        self.root.resizable(True, True)
        
        # 控制参数
        self.linear_speed = tk.DoubleVar(value=0.3)
        self.angular_speed = tk.DoubleVar(value=0.5)
        self.is_moving = False
        
        # 状态更新线程控制
        self.status_update_running = False
        self.status_thread = None
        
        self.setup_ui()
        
    def setup_ui(self):
        """设置用户界面"""
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 配置网格权重
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        
        # 标题
        title_label = ttk.Label(main_frame, text="轮趣机器人底盘控制器", 
                               font=('Arial', 16, 'bold'))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # 左侧控制面板
        self.setup_control_panel(main_frame)
        
        # 右侧状态面板
        self.setup_status_panel(main_frame)
        
        # 底部按钮
        self.setup_bottom_buttons(main_frame)
        
    def setup_control_panel(self, parent):
        """设置控制面板"""
        control_frame = ttk.LabelFrame(parent, text="运动控制", padding="10")
        control_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(0, 10))
        
        # 速度设置
        speed_frame = ttk.Frame(control_frame)
        speed_frame.grid(row=0, column=0, columnspan=3, pady=(0, 20), sticky=(tk.W, tk.E))
        
        ttk.Label(speed_frame, text="线速度 (m/s):").grid(row=0, column=0, sticky=tk.W)
        linear_scale = ttk.Scale(speed_frame, from_=0.1, to=1.0, variable=self.linear_speed, 
                                orient=tk.HORIZONTAL, length=200)
        linear_scale.grid(row=0, column=1, padx=(10, 5))
        linear_label = ttk.Label(speed_frame, textvariable=self.linear_speed)
        linear_label.grid(row=0, column=2)
        
        ttk.Label(speed_frame, text="角速度 (rad/s):").grid(row=1, column=0, sticky=tk.W, pady=(10, 0))
        angular_scale = ttk.Scale(speed_frame, from_=0.1, to=2.0, variable=self.angular_speed, 
                                 orient=tk.HORIZONTAL, length=200)
        angular_scale.grid(row=1, column=1, padx=(10, 5), pady=(10, 0))
        angular_label = ttk.Label(speed_frame, textvariable=self.angular_speed)
        angular_label.grid(row=1, column=2, pady=(10, 0))
        
        # 方向控制按钮
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=1, column=0, columnspan=3, pady=20)
        
        # 创建方向控制按钮
        btn_style = {'width': 8, 'height': 2}
        
        # 前进按钮
        self.forward_btn = tk.Button(button_frame, text="前进\n↑", 
                                    command=lambda: self.move_robot(1, 0, 0), **btn_style)
        self.forward_btn.grid(row=0, column=1, padx=5, pady=5)
        
        # 左转和右转
        self.left_btn = tk.Button(button_frame, text="左转\n↺", 
                                 command=lambda: self.move_robot(0, 0, 1), **btn_style)
        self.left_btn.grid(row=1, column=0, padx=5, pady=5)
        
        # 停止按钮
        self.stop_btn = tk.Button(button_frame, text="停止\n■", bg="red", fg="white",
                                 command=self.stop_robot, **btn_style)
        self.stop_btn.grid(row=1, column=1, padx=5, pady=5)
        
        self.right_btn = tk.Button(button_frame, text="右转\n↻", 
                                  command=lambda: self.move_robot(0, 0, -1), **btn_style)
        self.right_btn.grid(row=1, column=2, padx=5, pady=5)
        
        # 后退按钮
        self.backward_btn = tk.Button(button_frame, text="后退\n↓", 
                                     command=lambda: self.move_robot(-1, 0, 0), **btn_style)
        self.backward_btn.grid(row=2, column=1, padx=5, pady=5)
        
        # 左平移和右平移
        self.left_shift_btn = tk.Button(button_frame, text="左移\n←", 
                                       command=lambda: self.move_robot(0, 1, 0), **btn_style)
        self.left_shift_btn.grid(row=1, column=3, padx=5, pady=5)
        
        self.right_shift_btn = tk.Button(button_frame, text="右移\n→", 
                                        command=lambda: self.move_robot(0, -1, 0), **btn_style)
        self.right_shift_btn.grid(row=1, column=4, padx=5, pady=5)
        
        # 组合运动按钮
        combo_frame = ttk.LabelFrame(control_frame, text="组合运动", padding="5")
        combo_frame.grid(row=2, column=0, columnspan=3, pady=(20, 0), sticky=(tk.W, tk.E))
        combo_btn_style = {'width': 10}
        
        ttk.Button(combo_frame, text="左前", 
                  command=lambda: self.move_robot(1, 1, 0), **combo_btn_style).grid(row=0, column=0, padx=2, pady=2)
        ttk.Button(combo_frame, text="右前", 
                  command=lambda: self.move_robot(1, -1, 0), **combo_btn_style).grid(row=0, column=1, padx=2, pady=2)
        ttk.Button(combo_frame, text="左后", 
                  command=lambda: self.move_robot(-1, 1, 0), **combo_btn_style).grid(row=1, column=0, padx=2, pady=2)
        ttk.Button(combo_frame, text="右后", 
                  command=lambda: self.move_robot(-1, -1, 0), **combo_btn_style).grid(row=1, column=1, padx=2, pady=2)
        
    def setup_status_panel(self, parent):
        """设置状态面板"""
        status_frame = ttk.LabelFrame(parent, text="机器人状态", padding="10")
        status_frame.grid(row=1, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # 状态显示文本框
        self.status_text = tk.Text(status_frame, width=40, height=25, 
                                  font=('Courier', 10), state=tk.DISABLED)
        scrollbar = ttk.Scrollbar(status_frame, orient=tk.VERTICAL, command=self.status_text.yview)
        self.status_text.configure(yscrollcommand=scrollbar.set)
        
        self.status_text.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))
        
        status_frame.columnconfigure(0, weight=1)
        status_frame.rowconfigure(0, weight=1)
        
        # 刷新按钮
        ttk.Button(status_frame, text="刷新状态", 
                  command=self.update_status_display).grid(row=1, column=0, pady=(10, 0))
        
    def setup_bottom_buttons(self, parent):
        """设置底部按钮"""
        button_frame = ttk.Frame(parent)
        button_frame.grid(row=2, column=0, columnspan=2, pady=(20, 0))
        
        ttk.Button(button_frame, text="连接设备", 
                  command=self.connect_device).grid(row=0, column=0, padx=5)
        ttk.Button(button_frame, text="断开连接", 
                  command=self.disconnect_device).grid(row=0, column=1, padx=5)
        ttk.Button(button_frame, text="重置里程计", 
                  command=self.reset_odometry).grid(row=0, column=2, padx=5)
        ttk.Button(button_frame, text="退出程序", 
                  command=self.quit_application).grid(row=0, column=3, padx=5)
        
    def move_robot(self, x_dir: int, y_dir: int, z_dir: int):
        """控制机器人移动"""
        linear_x = x_dir * self.linear_speed.get()
        linear_y = y_dir * self.linear_speed.get()
        angular_z = z_dir * self.angular_speed.get()
        
        self.controller.send_velocity_command(linear_x, linear_y, angular_z)
        self.is_moving = True
        
        # 更新状态显示
        self.update_movement_status(linear_x, linear_y, angular_z)
        
    def stop_robot(self):
        """停止机器人"""
        self.controller.send_velocity_command(0, 0, 0)
        self.is_moving = False
        self.update_movement_status(0, 0, 0)
        
    def update_movement_status(self, linear_x: float, linear_y: float, angular_z: float):
        """更新运动状态显示"""
        status_msg = f"运动指令: X={linear_x:.2f} Y={linear_y:.2f} Z={angular_z:.2f}\n"
        self.append_status_text(status_msg)
        
    def update_status_display(self):
        """更新状态显示"""
        try:
            status = self.controller.get_robot_status()
            if status['success']:
                robot_status = status['status']
                
                status_text = f"""=== 机器人状态 ===
时间: {time.strftime('%H:%M:%S')}

位置信息:
  X: {robot_status['position']['x']:.3f} m
  Y: {robot_status['position']['y']:.3f} m
  Z: {robot_status['position']['z']:.3f} rad

速度信息:
  线速度X: {robot_status['velocity']['x']:.3f} m/s
  线速度Y: {robot_status['velocity']['y']:.3f} m/s
  角速度Z: {robot_status['velocity']['z']:.3f} rad/s

IMU数据:
  加速度X: {robot_status['imu']['accel_x']:.3f} m/s²
  加速度Y: {robot_status['imu']['accel_y']:.3f} m/s²
  加速度Z: {robot_status['imu']['accel_z']:.3f} m/s²
  角速度X: {robot_status['imu']['gyro_x']:.3f} rad/s
  角速度Y: {robot_status['imu']['gyro_y']:.3f} rad/s
  角速度Z: {robot_status['imu']['gyro_z']:.3f} rad/s

电源信息:
  电池电压: {robot_status['power_voltage']:.2f} V

连接状态: {'已连接' if self.controller.serial_conn and self.controller.serial_conn.is_open else '未连接'}
运行状态: {'运行中' if self.controller.running else '已停止'}
"""
                
                self.status_text.config(state=tk.NORMAL)
                self.status_text.delete(1.0, tk.END)
                self.status_text.insert(tk.END, status_text)
                self.status_text.config(state=tk.DISABLED)
                
        except Exception as e:
            self.append_status_text(f"状态更新错误: {e}\n")
            
    def append_status_text(self, text: str):
        """追加状态文本"""
        self.status_text.config(state=tk.NORMAL)
        self.status_text.insert(tk.END, text)
        self.status_text.see(tk.END)
        self.status_text.config(state=tk.DISABLED)
        
    def connect_device(self):
        """连接设备"""
        if not self.controller.serial_conn or not self.controller.serial_conn.is_open:
            if self.controller.connect():
                self.append_status_text("设备连接成功\n")
                if not self.controller.running:
                    self.controller.start()
                self.start_status_updates()
            else:
                messagebox.showerror("连接错误", "无法连接到设备")
        else:
            self.append_status_text("设备已连接\n")
            
    def disconnect_device(self):
        """断开设备连接"""
        self.stop_status_updates()
        self.controller.stop()
        self.append_status_text("设备已断开连接\n")
        
    def reset_odometry(self):
        """重置里程计"""
        result = self.controller.reset_odometry()
        if result['success']:
            self.append_status_text("里程计已重置\n")
        else:
            self.append_status_text(f"里程计重置失败: {result['message']}\n")
            
    def start_status_updates(self):
        """启动状态更新线程"""
        if not self.status_update_running:
            self.status_update_running = True
            self.status_thread = threading.Thread(target=self.status_update_loop, daemon=True)
            self.status_thread.start()
            
    def stop_status_updates(self):
        """停止状态更新线程"""
        self.status_update_running = False
        
    def status_update_loop(self):
        """状态更新循环"""
        while self.status_update_running:
            try:
                self.root.after(0, self.update_status_display)
                time.sleep(1.0)  # 每秒更新一次
            except Exception as e:
                print(f"Status update error: {e}")
                
    def quit_application(self):
        """退出应用程序"""
        self.stop_status_updates()
        self.controller.stop()
        self.root.quit()
        self.root.destroy()
        
    def run(self):
        """运行GUI"""
        try:
            self.root.protocol("WM_DELETE_WINDOW", self.quit_application)
            self.root.mainloop()
        except KeyboardInterrupt:
            self.quit_application()

# ==================== 全局控制器实例 ====================

_controller: Optional[WheeltecBaseController] = None

def get_base_controller(port: str = "/dev/wheeltec_controller", baudrate: int = 115200) -> WheeltecBaseController:
    """
    获取底盘控制器实例（单例模式）
    
    Args:
        port: 串口设备名
        baudrate: 波特率
        
    Returns:
        WheeltecBaseController: 控制器实例
    """
    global _controller
    if _controller is None:
        _controller = WheeltecBaseController(port=port, baudrate=baudrate)
    return _controller

# ==================== Function Calling 全局接口 ====================

def move_base_fc(linear_x: float, linear_y: float, angular_z: float, duration: float = 0.0) -> Dict:
    """
    Function Calling接口：控制机器人底盘移动
    
    Args:
        linear_x: X轴线速度 (m/s)
        linear_y: Y轴线速度 (m/s)
        angular_z: Z轴角速度 (rad/s)
        duration: 持续时间 (秒)，0表示持续执行
        
    Returns:
        Dict: 执行结果
    """
    try:
        controller = get_base_controller()
        if not controller.running:
            if not controller.start():
                return {
                    "success": False,
                    "message": "Failed to start base controller"
                }
        
        return controller.move_base(linear_x, linear_y, angular_z, duration)
    except Exception as e:
        return {
            "success": False,
            "message": f"Failed to move base: {str(e)}",
            "error": str(e)
        }

def get_robot_status_fc() -> Dict:
    """
    Function Calling接口：获取机器人状态
    
    Returns:
        Dict: 机器人状态信息
    """
    try:
        controller = get_base_controller()
        if not controller.running:
            if not controller.start():
                return {
                    "success": False,
                    "message": "Failed to start base controller"
                }
        
        return controller.get_robot_status()
    except Exception as e:
        return {
            "success": False,
            "message": f"Failed to get robot status: {str(e)}",
            "error": str(e)
        }

def stop_base_fc() -> Dict:
    """
    Function Calling接口：停止机器人底盘
    
    Returns:
        Dict: 执行结果
    """
    try:
        controller = get_base_controller()
        return controller.stop_base()
    except Exception as e:
        return {
            "success": False,
            "message": f"Failed to stop base: {str(e)}",
            "error": str(e)
        }

def set_velocity_limits_fc(max_linear: float, max_angular: float) -> Dict:
    """
    Function Calling接口：设置速度限制
    
    Args:
        max_linear: 最大线速度 (m/s)
        max_angular: 最大角速度 (rad/s)
        
    Returns:
        Dict: 执行结果
    """
    try:
        controller = get_base_controller()
        return controller.set_velocity_limits(max_linear, max_angular)
    except Exception as e:
        return {
            "success": False,
            "message": f"Failed to set velocity limits: {str(e)}",
            "error": str(e)
        }

def reset_odometry_fc() -> Dict:
    """
    Function Calling接口：重置里程计
    
    Returns:
        Dict: 执行结果
    """
    try:
        controller = get_base_controller()
        return controller.reset_odometry()
    except Exception as e:
        return {
            "success": False,
            "message": f"Failed to reset odometry: {str(e)}",
            "error": str(e)
        }

def shutdown_base_controller_fc() -> Dict:
    """
    Function Calling接口：关闭底盘控制器
    
    Returns:
        Dict: 执行结果
    """
    try:
        global _controller
        if _controller:
            _controller.stop()
        _controller = None
        return {
            "success": True,
            "message": "Base controller shutdown"
        }
    except Exception as e:
        return {
            "success": False,
            "message": f"Failed to shutdown base controller: {str(e)}",
            "error": str(e)
        }

# ==================== 主程序 ====================

def main():
    """主程序入口"""
    print("=== Wheeltec Robot Base Controller ===")
    print("1. 图形化界面控制")
    print("2. 手动控制模式")
    print("3. 退出")
    
    choice = input("请选择模式 (1-3): ").strip()
    
    if choice == "3":
        return
    
    # 初始化控制器
    port = input("请输入串口设备名 (默认: /dev/wheeltec_controller): ").strip() or "/dev/wheeltec_controller"
    
    controller = WheeltecBaseController(port=port)
    
    try:
        if choice == "1":
            # 图形化界面控制模式
            gui = WheeltecGUIController(controller)
            gui.run()
        
        elif choice == "2":
            # 手动控制模式
            if not controller.start():
                print("控制器启动失败")
                return
                
            print("\n=== 手动控制模式 ===")
            print("输入格式: linear_x linear_y angular_z duration")
            print("例如: 0.5 0 0.2 2.0 (前进0.5m/s，右转0.2rad/s，持续2秒)")
            print("输入 'status' 查看状态")
            print("输入 'quit' 退出")
            
            while True:
                try:
                    cmd = input("\n> ").strip().lower()
                    
                    if cmd == "quit":
                        break
                    elif cmd == "status":
                        status = controller.get_robot_status()
                        print(json.dumps(status, indent=2))
                    else:
                        parts = cmd.split()
                        if len(parts) >= 3:
                            linear_x = float(parts[0])
                            linear_y = float(parts[1])
                            angular_z = float(parts[2])
                            duration = float(parts[3]) if len(parts) > 3 else 0.0
                            
                            result = controller.move_base(linear_x, linear_y, angular_z, duration)
                            print(f"执行结果: {result['message']}")
                        else:
                            print("输入格式错误")
                            
                except ValueError:
                    print("数值格式错误")
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"错误: {e}")
    
    finally:
        controller.stop()
        print("程序结束")

if __name__ == "__main__":
    main()