import sys
import os
import time
import json
import threading
import tkinter as tk
from tkinter import ttk, messagebox
import keyboard
from typing import Dict, List, Tuple, Optional

sys.path.append("..")
from scservo_sdk import *
import math


class JointConfig:
    """关节配置类"""
    def __init__(self, name: str, servo_id: int, min_rad: float, max_rad: float, 
                 min_pos: int, max_pos: int, init_angle: float, inc_key: str, dec_key: str):
        self.name = name
        self.servo_id = servo_id
        self.min_rad = min_rad
        self.max_rad = max_rad
        self.min_pos = min_pos
        self.max_pos = max_pos
        self.init_angle = init_angle
        self.inc_key = inc_key.lower()
        self.dec_key = dec_key.lower()
        self.current_pos = self.angle_to_position(init_angle)
    
    def angle_to_position(self, angle_deg: float) -> int:
        """角度转换为舵机位置"""
        angle_rad = math.radians(angle_deg)
        angle_rad = max(self.min_rad, min(self.max_rad, angle_rad))
        normalized = (angle_rad - self.min_rad) / (self.max_rad - self.min_rad)
        return int(self.min_pos + normalized * (self.max_pos - self.min_pos))
    
    def position_to_angle(self, position: int) -> float:
        """舵机位置转换为角度"""
        position = max(self.min_pos, min(self.max_pos, position))
        normalized = (position - self.min_pos) / (self.max_pos - self.min_pos)
        angle_rad = self.min_rad + normalized * (self.max_rad - self.min_rad)
        return math.degrees(angle_rad)
    
    def validate_position(self, position: int) -> int:
        """验证并限制位置范围"""
        return max(self.min_pos, min(self.max_pos, position))

class DualArmController:
    """双机械臂控制器"""
    
    def __init__(self, port: str = '/dev/human_arm', baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.portHandler = None
        self.packetHandler = None
        self.action_groups = {}
        self.current_arm = "BOTH"
        self.keyboard_active = False
        self.gui_root = None
        
        # 关节配置
        self.joints = {
            # 左臂配置
            11: JointConfig("Left_Link0", 11, -3.14, 3.14, 0, 4096, 0, 'a', 'z'),
            12: JointConfig("Left_Link1", 12, -1.57, 1.57, 1024, 3072, 0, 's', 'x'),
            13: JointConfig("Left_Link2", 13, -1.57, 1.57, 1024, 3072, 0, 'd', 'c'),
            14: JointConfig("Left_Link3", 14, -1.57, 1.57, 1024, 3072, 0, 'f', 'v'),
            15: JointConfig("Left_Link4", 15, -1.57, 1.57, 1024, 3072, 0, 'g', 'b'),
            16: JointConfig("Left_Link5", 16, -3.14, 3.14, 0, 4096, 0, 'h', 'n'),
            17: JointConfig("Left_Gripper", 17, -0.2, 0.2, 1977, 2119, 0, 'j', 'm'),
            # 右臂配置
            1: JointConfig("Right_Link0", 1, -3.14, 3.14, 0, 4096, 0, '1', 'q'),
            2: JointConfig("Right_Link1", 2, -1.57, 1.57, 1024, 3072, 0, '2', 'w'),
            3: JointConfig("Right_Link2", 3, -1.57, 1.57, 1024, 3072, 0, '3', 'e'),
            4: JointConfig("Right_Link3", 4, -1.57, 1.57, 1024, 3072, 0, '4', 'r'),
            5: JointConfig("Right_Link4", 5, -1.57, 1.57, 1024, 3072, 0, '5', 't'),
            6: JointConfig("Right_Link5", 6, -3.14, 3.14, 0, 4096, 0, '6', 'y'),
            7: JointConfig("Right_Gripper", 7, -0.2, 0.2, 1977, 2119, 0, '7', 'u')
        }
        
        self.left_arm_ids = [11, 12, 13, 14, 15, 16, 17]
        self.right_arm_ids = [1, 2, 3, 4, 5, 6, 7]
    
    def initialize(self) -> bool:
        """初始化串口连接"""
        try:
            self.portHandler = PortHandler(self.port)
            self.packetHandler = sms_sts(self.portHandler)
            
            if not self.portHandler.openPort():
                raise Exception("无法打开串口")
            
            if not self.portHandler.setBaudRate(self.baudrate):
                raise Exception("无法设置波特率")
            
            self.load_action_groups()
            
            # 启用所有舵机的扭矩
            all_servo_ids = self.left_arm_ids + self.right_arm_ids
            if self.set_torque_enable(all_servo_ids, True):
                print("所有舵机扭矩已启用")
            else:
                print("警告：部分舵机扭矩启用失败")
            
            return True
        except Exception as e:
            print(f"初始化失败: {e}")
            return False
    
    def get_active_servo_ids(self) -> List[int]:
        """获取当前激活的舵机ID列表"""
        if self.current_arm == "LEFT":
            return self.left_arm_ids
        elif self.current_arm == "RIGHT":
            return self.right_arm_ids
        else:
            return self.left_arm_ids + self.right_arm_ids
    
    def set_joint_position(self, servo_id: int, position: int, speed: int = 1000, accel: int = 200) -> bool:
        """设置关节位置"""
        if servo_id not in self.joints:
            return False
        
        joint = self.joints[servo_id]
        position = joint.validate_position(position)
        
        try:
            result, error = self.packetHandler.WritePosEx(servo_id, position, speed, accel)
            if result != COMM_SUCCESS:
                print(f"舵机{servo_id}通信失败: {self.packetHandler.getTxRxResult(result)}")
                return False
            if error != 0:
                print(f"舵机{servo_id}错误: {self.packetHandler.getRxPacketError(error)}")
                return False
            
            joint.current_pos = position
            return True
        except Exception as e:
            print(f"设置舵机{servo_id}位置失败: {e}")
            return False
    
    def get_joint_position(self, servo_id: int) -> Optional[int]:
        """获取关节当前位置"""
        try:
            pos, speed, result, error = self.packetHandler.ReadPosSpeed(servo_id)
            if result == COMM_SUCCESS and error == 0:
                return pos
            return None
        except Exception:
            return None
    
    def set_torque_enable(self, servo_ids: List[int], enable: bool) -> bool:
        """设置扭矩使能"""
        success = True
        for servo_id in servo_ids:
            try:
                result, error = self.packetHandler.write1ByteTxRx(servo_id, 40, 1 if enable else 0)
                if result != COMM_SUCCESS or error != 0:
                    success = False
            except Exception:
                success = False
        return success
    
    def wait_movement_complete(self, servo_ids: List[int], timeout: float = 10.0) -> bool:
        """等待运动完成"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            all_stopped = True
            for servo_id in servo_ids:
                try:
                    moving, result, error = self.packetHandler.ReadMoving(servo_id)
                    if result == COMM_SUCCESS and error == 0 and moving != 0:
                        all_stopped = False
                        break
                except Exception:
                    continue
            
            if all_stopped:
                return True
            time.sleep(0.1)
        return False
    
    def move_to_position(self, positions: Dict[int, int], speed: int = 1000, accel: int = 200) -> bool:
        """移动到指定位置"""
        success = True
        for servo_id, position in positions.items():
            if not self.set_joint_position(servo_id, position, speed, accel):
                success = False
        
        if success:
            self.wait_movement_complete(list(positions.keys()))
        return success
    
    def move_to_angles(self, angles: Dict[int, float], speed: int = 1000, accel: int = 200) -> bool:
        """移动到指定角度"""
        positions = {}
        for servo_id, angle in angles.items():
            if servo_id in self.joints:
                positions[servo_id] = self.joints[servo_id].angle_to_position(angle)
        return self.move_to_position(positions, speed, accel)
    
    def get_current_angles(self) -> Dict[int, float]:
        """获取当前所有关节角度"""
        angles = {}
        for servo_id in self.get_active_servo_ids():
            pos = self.get_joint_position(servo_id)
            if pos is not None and servo_id in self.joints:
                angles[servo_id] = self.joints[servo_id].position_to_angle(pos)
        return angles
    
    def reset_to_initial_position(self) -> bool:
        """重置到初始位置"""
        positions = {}
        for servo_id in self.get_active_servo_ids():
            if servo_id in self.joints:
                positions[servo_id] = self.joints[servo_id].angle_to_position(0)
        return self.move_to_position(positions)
    
    def start_keyboard_control(self):
        """启动键盘控制"""
        if self.keyboard_active:
            return
        
        self.keyboard_active = True
        print("键盘控制已启动")
        print("按键映射:")
        for joint in self.joints.values():
            print(f"{joint.name}: {joint.inc_key.upper()}(增加) / {joint.dec_key.upper()}(减少)")
        print("按ESC退出键盘控制")
        
        def keyboard_handler():
            step = 50
            while self.keyboard_active:
                try:
                    for joint in self.joints.values():
                        if keyboard.is_pressed(joint.inc_key):
                            new_pos = joint.current_pos + step
                            self.set_joint_position(joint.servo_id, new_pos)
                            time.sleep(0.1)
                        elif keyboard.is_pressed(joint.dec_key):
                            new_pos = joint.current_pos - step
                            self.set_joint_position(joint.servo_id, new_pos)
                            time.sleep(0.1)
                    
                    if keyboard.is_pressed('esc'):
                        self.keyboard_active = False
                        break
                    
                    time.sleep(0.05)
                except Exception as e:
                    print(f"键盘控制错误: {e}")
                    break
        
        threading.Thread(target=keyboard_handler, daemon=True).start()
    
    def stop_keyboard_control(self):
        """停止键盘控制"""
        self.keyboard_active = False
        print("键盘控制已停止")
    
    def create_gui(self):
        """创建GUI界面"""
        self.gui_root = tk.Tk()
        self.gui_root.title("双机械臂控制系统")
        self.gui_root.geometry("800x600")
        
        # 创建主框架
        main_frame = ttk.Frame(self.gui_root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 机械臂选择
        arm_frame = ttk.LabelFrame(main_frame, text="机械臂选择")
        arm_frame.pack(fill=tk.X, pady=5)
        
        self.arm_var = tk.StringVar(value="BOTH")
        ttk.Radiobutton(arm_frame, text="左臂", variable=self.arm_var, value="LEFT").pack(side=tk.LEFT)
        ttk.Radiobutton(arm_frame, text="右臂", variable=self.arm_var, value="RIGHT").pack(side=tk.LEFT)
        ttk.Radiobutton(arm_frame, text="双臂", variable=self.arm_var, value="BOTH").pack(side=tk.LEFT)
        
        # 关节控制区域
        control_frame = ttk.LabelFrame(main_frame, text="关节控制")
        control_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # 创建滑动条
        self.sliders = {}
        self.angle_vars = {}
        
        canvas = tk.Canvas(control_frame)
        scrollbar = ttk.Scrollbar(control_frame, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        
        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        
        for i, (servo_id, joint) in enumerate(self.joints.items()):
            frame = ttk.Frame(scrollable_frame)
            frame.pack(fill=tk.X, pady=2)
            
            # 关节名称
            ttk.Label(frame, text=f"{joint.name} (ID:{servo_id})", width=20).pack(side=tk.LEFT)
            
            # 角度显示
            self.angle_vars[servo_id] = tk.StringVar(value="0.0°")
            ttk.Label(frame, textvariable=self.angle_vars[servo_id], width=10).pack(side=tk.LEFT)
            
            # 滑动条
            min_angle = math.degrees(joint.min_rad)
            max_angle = math.degrees(joint.max_rad)
            
            slider = ttk.Scale(frame, from_=min_angle, to=max_angle, orient=tk.HORIZONTAL,
                             command=lambda val, sid=servo_id: self.on_slider_change(sid, float(val)))
            slider.set(0)
            slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
            self.sliders[servo_id] = slider
            
            # 键盘提示
            ttk.Label(frame, text=f"{joint.inc_key.upper()}/{joint.dec_key.upper()}", width=8).pack(side=tk.RIGHT)
        
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        
        # 控制按钮
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(button_frame, text="重置位置", command=self.reset_to_initial_position).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="启动键盘控制", command=self.start_keyboard_control).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="停止键盘控制", command=self.stop_keyboard_control).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="示教模式", command=self.teaching_mode).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="执行动作组", command=self.execute_action_group).pack(side=tk.LEFT, padx=5)
        
        # 更新当前位置
        self.update_gui_positions()
    
    def on_slider_change(self, servo_id: int, angle: float):
        """滑动条变化回调"""
        self.current_arm = self.arm_var.get()
        if servo_id in self.get_active_servo_ids():
            self.set_joint_position(servo_id, self.joints[servo_id].angle_to_position(angle))
            self.angle_vars[servo_id].set(f"{angle:.1f}°")
    
    def update_gui_positions(self):
        """更新GUI显示的位置"""
        if self.gui_root is None:
            return
        
        angles = self.get_current_angles()
        for servo_id, angle in angles.items():
            if servo_id in self.sliders:
                self.sliders[servo_id].set(angle)
                self.angle_vars[servo_id].set(f"{angle:.1f}°")
        
        self.gui_root.after(100, self.update_gui_positions)
    
    def teaching_mode(self):
        """示教模式"""
        self.current_arm = self.arm_var.get()
        active_ids = self.get_active_servo_ids()
        
        # 禁用扭矩
        self.set_torque_enable(active_ids, False)
        
        result = messagebox.askyesno("示教模式", "已禁用扭矩，请手动调整机械臂到目标位置。\n调整完成后点击'是'保存当前位置。")
        
        if result:
            positions = {}
            for servo_id in active_ids:
                pos = self.get_joint_position(servo_id)
                if pos is not None:
                    positions[servo_id] = pos
            
            name = tk.simpledialog.askstring("保存动作组", "请输入动作组名称:")
            if name:
                self.action_groups[f"{self.current_arm}_{name}"] = positions
                self.save_action_groups()
                messagebox.showinfo("成功", f"动作组'{name}'已保存")
        
        # 重新启用扭矩
        self.set_torque_enable(active_ids, True)
    
    def execute_action_group(self):
        """执行动作组"""
        if not self.action_groups:
            messagebox.showwarning("警告", "没有保存的动作组")
            return
        
        # 创建选择对话框
        dialog = tk.Toplevel(self.gui_root)
        dialog.title("选择动作组")
        dialog.geometry("300x200")
        
        listbox = tk.Listbox(dialog)
        listbox.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        for name in self.action_groups.keys():
            listbox.insert(tk.END, name)
        
        def execute_selected():
            selection = listbox.curselection()
            if selection:
                name = listbox.get(selection[0])
                positions = self.action_groups[name]
                if self.move_to_position(positions):
                    messagebox.showinfo("成功", f"动作组'{name}'执行完成")
                else:
                    messagebox.showerror("错误", "动作组执行失败")
                dialog.destroy()
        
        ttk.Button(dialog, text="执行", command=execute_selected).pack(pady=5)
    
    def save_action_groups(self):
        """保存动作组到文件"""
        try:
            with open('action_groups.json', 'w', encoding='utf-8') as f:
                json.dump(self.action_groups, f, ensure_ascii=False, indent=2)
        except Exception as e:
            print(f"保存动作组失败: {e}")
    
    def load_action_groups(self):
        """从文件加载动作组"""
        try:
            with open('action_groups.json', 'r', encoding='utf-8') as f:
                self.action_groups = json.load(f)
        except FileNotFoundError:
            self.action_groups = {}
        except Exception as e:
            print(f"加载动作组失败: {e}")
            self.action_groups = {}
    
    def close(self):
        """关闭连接"""
        self.keyboard_active = False
        if self.portHandler:
            self.portHandler.closePort()
        if self.gui_root:
            self.gui_root.destroy()

# Function Calling 接口函数
def move_arm_to_position(arm: str, positions: Dict[str, float]) -> bool:
    """移动机械臂到指定位置
    
    Args:
        arm: 机械臂选择 ('LEFT', 'RIGHT', 'BOTH')
        positions: 关节位置字典，键为关节名称，值为角度
    
    Returns:
        bool: 执行是否成功
    """
    controller.current_arm = arm.upper()
    
    # 转换关节名称到舵机ID
    servo_positions = {}
    for joint_name, angle in positions.items():
        for servo_id, joint in controller.joints.items():
            if joint.name == joint_name:
                servo_positions[servo_id] = angle
                break
    
    return controller.move_to_angles(servo_positions)

def get_arm_status(arm: str = "BOTH") -> Dict[str, float]:
    """获取机械臂状态
    
    Args:
        arm: 机械臂选择 ('LEFT', 'RIGHT', 'BOTH')
    
    Returns:
        Dict[str, float]: 关节角度字典
    """
    controller.current_arm = arm.upper()
    angles = controller.get_current_angles()
    
    # 转换舵机ID到关节名称
    joint_angles = {}
    for servo_id, angle in angles.items():
        if servo_id in controller.joints:
            joint_angles[controller.joints[servo_id].name] = angle
    
    return joint_angles

def execute_saved_action(action_name: str) -> bool:
    """执行保存的动作组
    
    Args:
        action_name: 动作组名称
    
    Returns:
        bool: 执行是否成功
    """
    if action_name not in controller.action_groups:
        return False
    
    positions = controller.action_groups[action_name]
    return controller.move_to_position(positions)

def save_current_pose(action_name: str, arm: str = "BOTH") -> bool:
    """保存当前姿态为动作组
    
    Args:
        action_name: 动作组名称
        arm: 机械臂选择
    
    Returns:
        bool: 保存是否成功
    """
    controller.current_arm = arm.upper()
    active_ids = controller.get_active_servo_ids()
    
    positions = {}
    for servo_id in active_ids:
        pos = controller.get_joint_position(servo_id)
        if pos is not None:
            positions[servo_id] = pos
    
    controller.action_groups[f"{arm}_{action_name}"] = positions
    controller.save_action_groups()
    return True

def reset_arm_position(arm: str = "BOTH") -> bool:
    """重置机械臂到初始位置
    
    Args:
        arm: 机械臂选择
    
    Returns:
        bool: 重置是否成功
    """
    controller.current_arm = arm.upper()
    return controller.reset_to_initial_position()

# 全局控制器实例
controller = DualArmController()

def main():
    """主函数"""
    if not controller.initialize():
        print("初始化失败，程序退出")
        return
    
    print("双机械臂控制系统已启动")
    print("可用的Function Calling接口:")
    print("- move_arm_to_position(arm, positions)")
    print("- get_arm_status(arm)")
    print("- execute_saved_action(action_name)")
    print("- save_current_pose(action_name, arm)")
    print("- reset_arm_position(arm)")
    
    # 检查扭矩状态
    print("\n检查扭矩状态...")
    all_servo_ids = controller.left_arm_ids + controller.right_arm_ids
    controller.set_torque_enable(all_servo_ids, True)
    print("扭矩已确保启用")
    
    try:
        controller.create_gui()
        controller.gui_root.mainloop()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    finally:
        controller.close()

if __name__ == "__main__":
    import math
    import tkinter.simpledialog
    main()