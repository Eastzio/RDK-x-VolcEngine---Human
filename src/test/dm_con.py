import serial
import time
import sys
import threading
from typing import Dict, Optional, Tuple
from DM_CAN import Motor, MotorControl, DM_Motor_Type, Control_Type, DM_variable

class LiftJointController:
    """升降关节控制器 - 适配Function Calling"""
    
    def __init__(self, com_port: str = "/dev/dmmotor", motor_id: int = 0x11, 
                 master_id: int = 0x01, baud_rate: int = 921600):
        self.com_port = com_port
        self.motor_id = motor_id
        self.master_id = master_id
        self.baud_rate = baud_rate
        self.motor_type = DM_Motor_Type.DM4310
        
        # 关节限制 (0rad底部, 30rad顶部)
        self.position_min = 0.0
        self.position_max = 30.0
        self.default_velocity = 10.0  # 默认10rad/s
        
        self.serial_device = None
        self.motor = None
        self.motor_ctrl = None
        self.is_initialized = False
        self.lock = threading.Lock()
        
    def initialize(self) -> Dict[str, any]:
        """初始化升降关节"""
        try:
            with self.lock:
                if self.is_initialized:
                    return {"success": True, "message": "已初始化"}
                
                # 初始化串口
                self.serial_device = serial.Serial(self.com_port, self.baud_rate, timeout=0.5)
                
                # 初始化电机
                self.motor = Motor(self.motor_type, self.motor_id, self.master_id)
                self.motor_ctrl = MotorControl(self.serial_device)
                self.motor_ctrl.addMotor(self.motor)
                
                time.sleep(0.5)
                
                # 切换到位置-速度控制模式
                if not self.motor_ctrl.switchControlMode(self.motor, Control_Type.POS_VEL):
                    raise Exception("切换控制模式失败")
                
                # 设置阻尼因子
                self.motor_ctrl.change_motor_param(self.motor, DM_variable.Damp, 4.0)
                
                # 使能电机
                self.motor_ctrl.enable(self.motor)
                time.sleep(1)
                
                self.is_initialized = True
                return {
                    "success": True, 
                    "message": "升降关节初始化成功",
                    "position_range": [self.position_min, self.position_max],
                    "default_velocity": self.default_velocity
                }
                
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def move_to_position(self, target_position: float, velocity: Optional[float] = None) -> Dict[str, any]:
        """移动到指定位置"""
        if not self.is_initialized:
            init_result = self.initialize()
            if not init_result["success"]:
                return init_result
        
        try:
            with self.lock:
                # 限制位置范围
                target_position = max(min(target_position, self.position_max), self.position_min)
                velocity = velocity or self.default_velocity
                
                # 获取当前位置
                self.motor_ctrl.refresh_motor_status(self.motor)
                current_pos = self.motor.getPosition()
                
                # 发送控制指令
                self.motor_ctrl.control_Pos_Vel(self.motor, target_position, velocity)
                
                return {
                    "success": True,
                    "message": f"升降关节移动指令已发送",
                    "current_position": current_pos,
                    "target_position": target_position,
                    "velocity": velocity
                }
                
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def get_status(self) -> Dict[str, any]:
        """获取升降关节状态"""
        if not self.is_initialized:
            return {"success": False, "error": "未初始化"}
        
        try:
            with self.lock:
                self.motor_ctrl.refresh_motor_status(self.motor)
                return {
                    "success": True,
                    "position": self.motor.getPosition(),
                    "velocity": self.motor.getVelocity(),
                    "torque": self.motor.getTorque(),
                    "position_range": [self.position_min, self.position_max],
                    "is_enabled": self.motor.isEnable
                }
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def set_velocity_limit(self, max_velocity: float) -> Dict[str, any]:
        """设置速度限制"""
        try:
            self.default_velocity = max(0.1, min(max_velocity, 30.0))  # 限制在合理范围
            return {
                "success": True,
                "message": f"速度限制设置为 {self.default_velocity} rad/s"
            }
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def emergency_stop(self) -> Dict[str, any]:
        """紧急停止"""
        if not self.is_initialized:
            return {"success": False, "error": "未初始化"}
        
        try:
            with self.lock:
                current_pos = self.motor.getPosition()
                self.motor_ctrl.control_Pos_Vel(self.motor, current_pos, 0)
                return {"success": True, "message": "紧急停止执行"}
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def shutdown(self) -> Dict[str, any]:
        """关闭升降关节"""
        try:
            with self.lock:
                if self.is_initialized and self.motor_ctrl:
                    self.motor_ctrl.disable(self.motor)
                    time.sleep(0.5)
                if self.serial_device:
                    self.serial_device.close()
                self.is_initialized = False
                return {"success": True, "message": "升降关节已安全关闭"}
        except Exception as e:
            return {"success": False, "error": str(e)}

# Function Calling接口
lift_controller = LiftJointController()

def move_lift_to_position(position: float, velocity: float = 10.0) -> Dict[str, any]:
    """移动升降关节到指定位置
    Args:
        position: 目标位置 (0-30 rad, 0为底部, 30为顶部)
        velocity: 移动速度 (rad/s, 默认10)
    """
    return lift_controller.move_to_position(position, velocity)

def get_lift_status() -> Dict[str, any]:
    """获取升降关节当前状态"""
    return lift_controller.get_status()

def set_lift_velocity_limit(max_velocity: float) -> Dict[str, any]:
    """设置升降关节速度限制
    Args:
        max_velocity: 最大速度 (rad/s)
    """
    return lift_controller.set_velocity_limit(max_velocity)

def emergency_stop_lift() -> Dict[str, any]:
    """紧急停止升降关节"""
    return lift_controller.emergency_stop()

def shutdown_lift() -> Dict[str, any]:
    """关闭升降关节系统"""
    return lift_controller.shutdown()

if __name__ == "__main__":
    # 测试代码
    print("升降关节控制器测试")
    result = move_lift_to_position(15.0, 5.0)
    print(f"移动结果: {result}")
    
    status = get_lift_status()
    print(f"状态: {status}")