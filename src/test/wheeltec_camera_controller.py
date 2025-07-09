#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Wheeltec Camera Controller
支持 Astra_Pro、Gemini、Wheeltec_Usbcam 摄像头
提供彩色、深度、红外图像流显示功能
支持独立调用和Function Calling
"""

import cv2
import numpy as np
import threading
import time
import json
from typing import Dict, Optional, Tuple, List
from enum import Enum
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class CameraType(Enum):
    """摄像头类型枚举"""
    ASTRA_PRO = "astra_pro"
    GEMINI = "gemini"
    WHEELTEC_USBCAM = "wheeltec_usbcam"

class DisplayMode(Enum):
    """显示模式枚举"""
    SINGLE_WINDOW = "single"  # 单窗口显示
    THREE_WINDOWS = "three"   # 三个独立窗口

class WheeltecCameraController:
    """Wheeltec摄像头控制器"""
    
    def __init__(self):
        self.camera_type = CameraType.WHEELTEC_USBCAM
        self.display_mode = DisplayMode.THREE_WINDOWS
        self.is_running = False
        self.capture_thread = None
        self.data_lock = threading.Lock()
        
        # 图像数据
        self.color_frame = None
        self.depth_frame = None
        self.ir_frame = None
        
        # 摄像头对象
        self.color_cap = None
        self.depth_cap = None
        self.ir_cap = None
        
        # 摄像头配置
        self.camera_configs = {
            CameraType.ASTRA_PRO: {
                'color_device': '/dev/video0',
                'depth_device': '/dev/video1',
                'ir_device': '/dev/video2',
                'color_size': (640, 480),
                'depth_size': (640, 480),
                'ir_size': (640, 480),
                'fps': 30
            },
            CameraType.GEMINI: {
                'color_device': '/dev/video0',
                'depth_device': '/dev/video1', 
                'ir_device': '/dev/video2',
                'color_size': (640, 480),
                'depth_size': (640, 400),
                'ir_size': (640, 400),
                'fps': 30
            },
            CameraType.WHEELTEC_USBCAM: {
                'color_device': '/dev/video0',
                'depth_device': None,  # USB摄像头通常只有彩色
                'ir_device': None,
                'color_size': (640, 480),
                'depth_size': None,
                'ir_size': None,
                'fps': 30
            }
        }
        
        # 窗口名称
        self.window_names = {
            'color': 'Color Stream',
            'depth': 'Depth Stream', 
            'ir': 'IR Stream',
            'combined': 'Camera Streams'
        }
        
    def initialize_camera(self, camera_type: str = "wheeltec_usbcam", 
                         display_mode: str = "three",
                         color_device: str = None,
                         depth_device: str = None,
                         ir_device: str = None) -> Dict[str, any]:
        """初始化摄像头
        
        Args:
            camera_type: 摄像头类型 (astra_pro, gemini, wheeltec_usbcam)
            display_mode: 显示模式 (single, three)
            color_device: 彩色摄像头设备路径
            depth_device: 深度摄像头设备路径  
            ir_device: 红外摄像头设备路径
            
        Returns:
            Dict: 初始化结果
        """
        try:
            # 设置摄像头类型
            self.camera_type = CameraType(camera_type)
            self.display_mode = DisplayMode(display_mode)
            
            # 获取配置
            config = self.camera_configs[self.camera_type].copy()
            
            # 覆盖设备路径（如果提供）
            if color_device:
                config['color_device'] = color_device
            if depth_device:
                config['depth_device'] = depth_device
            if ir_device:
                config['ir_device'] = ir_device
                
            # 初始化摄像头
            success = self._init_cameras(config)
            
            if success:
                logger.info(f"摄像头初始化成功: {camera_type}")
                return {
                    "success": True,
                    "message": f"摄像头 {camera_type} 初始化成功",
                    "camera_type": camera_type,
                    "display_mode": display_mode
                }
            else:
                return {
                    "success": False,
                    "message": "摄像头初始化失败",
                    "error": "无法打开摄像头设备"
                }
                
        except Exception as e:
            logger.error(f"初始化摄像头失败: {e}")
            return {
                "success": False,
                "message": "摄像头初始化失败",
                "error": str(e)
            }
    
    def _init_cameras(self, config: Dict) -> bool:
        """初始化摄像头设备"""
        try:
            # 初始化彩色摄像头
            if config['color_device']:
                self.color_cap = cv2.VideoCapture(config['color_device'])
                if self.color_cap.isOpened():
                    width, height = config['color_size']
                    self.color_cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    self.color_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                    self.color_cap.set(cv2.CAP_PROP_FPS, config['fps'])
                    logger.info(f"彩色摄像头已连接: {config['color_device']}")
                else:
                    logger.warning(f"无法打开彩色摄像头: {config['color_device']}")
                    
            # 初始化深度摄像头（如果支持）
            if config['depth_device']:
                self.depth_cap = cv2.VideoCapture(config['depth_device'])
                if self.depth_cap.isOpened():
                    width, height = config['depth_size']
                    self.depth_cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    self.depth_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                    self.depth_cap.set(cv2.CAP_PROP_FPS, config['fps'])
                    logger.info(f"深度摄像头已连接: {config['depth_device']}")
                else:
                    logger.warning(f"无法打开深度摄像头: {config['depth_device']}")
                    
            # 初始化红外摄像头（如果支持）
            if config['ir_device']:
                self.ir_cap = cv2.VideoCapture(config['ir_device'])
                if self.ir_cap.isOpened():
                    width, height = config['ir_size']
                    self.ir_cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    self.ir_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                    self.ir_cap.set(cv2.CAP_PROP_FPS, config['fps'])
                    logger.info(f"红外摄像头已连接: {config['ir_device']}")
                else:
                    logger.warning(f"无法打开红外摄像头: {config['ir_device']}")
                    
            return True
            
        except Exception as e:
            logger.error(f"初始化摄像头设备失败: {e}")
            return False
    
    def start_camera_stream(self) -> Dict[str, any]:
        """开始摄像头数据流
        
        Returns:
            Dict: 操作结果
        """
        try:
            if self.is_running:
                return {
                    "success": False,
                    "message": "摄像头流已经在运行中"
                }
                
            self.is_running = True
            self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self.capture_thread.start()
            
            logger.info("摄像头数据流已启动")
            return {
                "success": True,
                "message": "摄像头数据流已启动"
            }
            
        except Exception as e:
            logger.error(f"启动摄像头流失败: {e}")
            return {
                "success": False,
                "message": "启动摄像头流失败",
                "error": str(e)
            }
    
    def stop_camera_stream(self) -> Dict[str, any]:
        """停止摄像头数据流
        
        Returns:
            Dict: 操作结果
        """
        try:
            if not self.is_running:
                return {
                    "success": False,
                    "message": "摄像头流未在运行"
                }
                
            self.is_running = False
            
            if self.capture_thread and self.capture_thread.is_alive():
                self.capture_thread.join(timeout=2.0)
                
            # 关闭所有窗口
            cv2.destroyAllWindows()
            
            logger.info("摄像头数据流已停止")
            return {
                "success": True,
                "message": "摄像头数据流已停止"
            }
            
        except Exception as e:
            logger.error(f"停止摄像头流失败: {e}")
            return {
                "success": False,
                "message": "停止摄像头流失败",
                "error": str(e)
            }
    
    def get_camera_status(self) -> Dict[str, any]:
        """获取摄像头状态
        
        Returns:
            Dict: 摄像头状态信息
        """
        try:
            with self.data_lock:
                status = {
                    "success": True,
                    "is_running": self.is_running,
                    "camera_type": self.camera_type.value,
                    "display_mode": self.display_mode.value,
                    "streams": {
                        "color": {
                            "available": self.color_cap is not None and self.color_cap.isOpened(),
                            "has_data": self.color_frame is not None
                        },
                        "depth": {
                            "available": self.depth_cap is not None and self.depth_cap.isOpened(),
                            "has_data": self.depth_frame is not None
                        },
                        "ir": {
                            "available": self.ir_cap is not None and self.ir_cap.isOpened(),
                            "has_data": self.ir_frame is not None
                        }
                    }
                }
                
            return status
            
        except Exception as e:
            logger.error(f"获取摄像头状态失败: {e}")
            return {
                "success": False,
                "message": "获取摄像头状态失败",
                "error": str(e)
            }
    
    def get_frame_data(self, stream_type: str = "color") -> Dict[str, any]:
        """获取指定流的帧数据
        
        Args:
            stream_type: 流类型 (color, depth, ir)
            
        Returns:
            Dict: 帧数据信息
        """
        try:
            with self.data_lock:
                if stream_type == "color" and self.color_frame is not None:
                    height, width = self.color_frame.shape[:2]
                    return {
                        "success": True,
                        "stream_type": stream_type,
                        "width": width,
                        "height": height,
                        "channels": self.color_frame.shape[2] if len(self.color_frame.shape) > 2 else 1,
                        "has_data": True
                    }
                elif stream_type == "depth" and self.depth_frame is not None:
                    height, width = self.depth_frame.shape[:2]
                    return {
                        "success": True,
                        "stream_type": stream_type,
                        "width": width,
                        "height": height,
                        "channels": self.depth_frame.shape[2] if len(self.depth_frame.shape) > 2 else 1,
                        "has_data": True
                    }
                elif stream_type == "ir" and self.ir_frame is not None:
                    height, width = self.ir_frame.shape[:2]
                    return {
                        "success": True,
                        "stream_type": stream_type,
                        "width": width,
                        "height": height,
                        "channels": self.ir_frame.shape[2] if len(self.ir_frame.shape) > 2 else 1,
                        "has_data": True
                    }
                else:
                    return {
                        "success": False,
                        "stream_type": stream_type,
                        "has_data": False,
                        "message": f"没有可用的 {stream_type} 数据"
                    }
                    
        except Exception as e:
            logger.error(f"获取帧数据失败: {e}")
            return {
                "success": False,
                "message": "获取帧数据失败",
                "error": str(e)
            }
    
    def set_display_mode(self, mode: str) -> Dict[str, any]:
        """设置显示模式
        
        Args:
            mode: 显示模式 (single, three)
            
        Returns:
            Dict: 操作结果
        """
        try:
            old_mode = self.display_mode
            self.display_mode = DisplayMode(mode)
            
            # 如果正在运行，关闭旧窗口
            if self.is_running:
                cv2.destroyAllWindows()
                
            logger.info(f"显示模式已从 {old_mode.value} 更改为 {mode}")
            return {
                "success": True,
                "message": f"显示模式已设置为 {mode}",
                "old_mode": old_mode.value,
                "new_mode": mode
            }
            
        except ValueError:
            return {
                "success": False,
                "message": f"无效的显示模式: {mode}",
                "valid_modes": [mode.value for mode in DisplayMode]
            }
        except Exception as e:
            logger.error(f"设置显示模式失败: {e}")
            return {
                "success": False,
                "message": "设置显示模式失败",
                "error": str(e)
            }
    
    def shutdown_camera(self) -> Dict[str, any]:
        """关闭摄像头系统
        
        Returns:
            Dict: 操作结果
        """
        try:
            # 停止数据流
            self.stop_camera_stream()
            
            # 释放摄像头资源
            if self.color_cap:
                self.color_cap.release()
                self.color_cap = None
                
            if self.depth_cap:
                self.depth_cap.release()
                self.depth_cap = None
                
            if self.ir_cap:
                self.ir_cap.release()
                self.ir_cap = None
                
            # 清理数据
            with self.data_lock:
                self.color_frame = None
                self.depth_frame = None
                self.ir_frame = None
                
            logger.info("摄像头系统已关闭")
            return {
                "success": True,
                "message": "摄像头系统已关闭"
            }
            
        except Exception as e:
            logger.error(f"关闭摄像头系统失败: {e}")
            return {
                "success": False,
                "message": "关闭摄像头系统失败",
                "error": str(e)
            }
    
    def _capture_loop(self):
        """摄像头捕获循环"""
        logger.info("摄像头捕获循环已启动")
        
        while self.is_running:
            try:
                # 捕获彩色图像
                if self.color_cap and self.color_cap.isOpened():
                    ret, frame = self.color_cap.read()
                    if ret:
                        with self.data_lock:
                            self.color_frame = frame.copy()
                            
                # 捕获深度图像（如果可用）
                if self.depth_cap and self.depth_cap.isOpened():
                    ret, frame = self.depth_cap.read()
                    if ret:
                        with self.data_lock:
                            self.depth_frame = frame.copy()
                            
                # 捕获红外图像（如果可用）
                if self.ir_cap and self.ir_cap.isOpened():
                    ret, frame = self.ir_cap.read()
                    if ret:
                        with self.data_lock:
                            self.ir_frame = frame.copy()
                
                # 显示图像
                self._display_frames()
                
                # 检查退出键
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC键
                    logger.info("检测到ESC键，停止摄像头流")
                    self.is_running = False
                    break
                elif key == ord('s'):  # S键切换显示模式
                    current_mode = self.display_mode
                    new_mode = DisplayMode.SINGLE_WINDOW if current_mode == DisplayMode.THREE_WINDOWS else DisplayMode.THREE_WINDOWS
                    self.set_display_mode(new_mode.value)
                    
                time.sleep(0.01)  # 控制帧率
                
            except Exception as e:
                logger.error(f"捕获循环错误: {e}")
                time.sleep(0.1)
                
        logger.info("摄像头捕获循环已结束")
    
    def _display_frames(self):
        """显示图像帧"""
        try:
            with self.data_lock:
                color_frame = self.color_frame.copy() if self.color_frame is not None else None
                depth_frame = self.depth_frame.copy() if self.depth_frame is not None else None
                ir_frame = self.ir_frame.copy() if self.ir_frame is not None else None
            
            if self.display_mode == DisplayMode.THREE_WINDOWS:
                # 三个独立窗口显示
                if color_frame is not None:
                    cv2.imshow(self.window_names['color'], color_frame)
                    
                if depth_frame is not None:
                    # 深度图像可能需要特殊处理
                    if len(depth_frame.shape) == 3:
                        depth_display = cv2.cvtColor(depth_frame, cv2.COLOR_BGR2GRAY)
                    else:
                        depth_display = depth_frame
                    depth_display = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)
                    cv2.imshow(self.window_names['depth'], depth_display)
                    
                if ir_frame is not None:
                    # 红外图像处理
                    if len(ir_frame.shape) == 3:
                        ir_display = cv2.cvtColor(ir_frame, cv2.COLOR_BGR2GRAY)
                    else:
                        ir_display = ir_frame
                    cv2.imshow(self.window_names['ir'], ir_display)
                    
            else:
                # 单窗口显示（组合显示）
                combined_frame = self._create_combined_frame(color_frame, depth_frame, ir_frame)
                if combined_frame is not None:
                    cv2.imshow(self.window_names['combined'], combined_frame)
                    
        except Exception as e:
            logger.error(f"显示帧错误: {e}")
    
    def _create_combined_frame(self, color_frame, depth_frame, ir_frame):
        """创建组合显示帧"""
        try:
            frames = []
            frame_size = (320, 240)  # 缩放后的尺寸
            
            # 处理彩色帧
            if color_frame is not None:
                color_resized = cv2.resize(color_frame, frame_size)
                cv2.putText(color_resized, 'Color', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                frames.append(color_resized)
            else:
                # 创建黑色占位符
                placeholder = np.zeros((frame_size[1], frame_size[0], 3), dtype=np.uint8)
                cv2.putText(placeholder, 'No Color', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                frames.append(placeholder)
            
            # 处理深度帧
            if depth_frame is not None:
                if len(depth_frame.shape) == 3:
                    depth_gray = cv2.cvtColor(depth_frame, cv2.COLOR_BGR2GRAY)
                else:
                    depth_gray = depth_frame
                depth_colored = cv2.applyColorMap(depth_gray, cv2.COLORMAP_JET)
                depth_resized = cv2.resize(depth_colored, frame_size)
                cv2.putText(depth_resized, 'Depth', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                frames.append(depth_resized)
            else:
                placeholder = np.zeros((frame_size[1], frame_size[0], 3), dtype=np.uint8)
                cv2.putText(placeholder, 'No Depth', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                frames.append(placeholder)
            
            # 处理红外帧
            if ir_frame is not None:
                if len(ir_frame.shape) == 3:
                    ir_gray = cv2.cvtColor(ir_frame, cv2.COLOR_BGR2GRAY)
                    ir_colored = cv2.cvtColor(ir_gray, cv2.COLOR_GRAY2BGR)
                else:
                    ir_colored = cv2.cvtColor(ir_frame, cv2.COLOR_GRAY2BGR)
                ir_resized = cv2.resize(ir_colored, frame_size)
                cv2.putText(ir_resized, 'IR', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                frames.append(ir_resized)
            else:
                placeholder = np.zeros((frame_size[1], frame_size[0], 3), dtype=np.uint8)
                cv2.putText(placeholder, 'No IR', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                frames.append(placeholder)
            
            # 水平拼接所有帧
            if frames:
                combined = np.hstack(frames)
                return combined
            else:
                return None
                
        except Exception as e:
            logger.error(f"创建组合帧错误: {e}")
            return None

# Function Calling 接口函数
def initialize_camera_system(camera_type: str = "wheeltec_usbcam", 
                            display_mode: str = "three",
                            color_device: str = None,
                            depth_device: str = None,
                            ir_device: str = None) -> str:
    """初始化摄像头系统
    
    Args:
        camera_type: 摄像头类型 (astra_pro, gemini, wheeltec_usbcam)
        display_mode: 显示模式 (single, three)
        color_device: 彩色摄像头设备路径
        depth_device: 深度摄像头设备路径
        ir_device: 红外摄像头设备路径
        
    Returns:
        str: JSON格式的操作结果
    """
    global camera_controller
    if camera_controller is None:
        camera_controller = WheeltecCameraController()
    
    result = camera_controller.initialize_camera(
        camera_type=camera_type,
        display_mode=display_mode,
        color_device=color_device,
        depth_device=depth_device,
        ir_device=ir_device
    )
    return json.dumps(result, ensure_ascii=False)

def start_camera_stream() -> str:
    """启动摄像头数据流
    
    Returns:
        str: JSON格式的操作结果
    """
    global camera_controller
    if camera_controller is None:
        return json.dumps({"success": False, "message": "摄像头系统未初始化"}, ensure_ascii=False)
    
    result = camera_controller.start_camera_stream()
    return json.dumps(result, ensure_ascii=False)

def stop_camera_stream() -> str:
    """停止摄像头数据流
    
    Returns:
        str: JSON格式的操作结果
    """
    global camera_controller
    if camera_controller is None:
        return json.dumps({"success": False, "message": "摄像头系统未初始化"}, ensure_ascii=False)
    
    result = camera_controller.stop_camera_stream()
    return json.dumps(result, ensure_ascii=False)

def get_camera_status() -> str:
    """获取摄像头状态
    
    Returns:
        str: JSON格式的状态信息
    """
    global camera_controller
    if camera_controller is None:
        return json.dumps({"success": False, "message": "摄像头系统未初始化"}, ensure_ascii=False)
    
    result = camera_controller.get_camera_status()
    return json.dumps(result, ensure_ascii=False)

def get_frame_data(stream_type: str = "color") -> str:
    """获取指定流的帧数据信息
    
    Args:
        stream_type: 流类型 (color, depth, ir)
        
    Returns:
        str: JSON格式的帧数据信息
    """
    global camera_controller
    if camera_controller is None:
        return json.dumps({"success": False, "message": "摄像头系统未初始化"}, ensure_ascii=False)
    
    result = camera_controller.get_frame_data(stream_type)
    return json.dumps(result, ensure_ascii=False)

def set_display_mode(mode: str) -> str:
    """设置显示模式
    
    Args:
        mode: 显示模式 (single, three)
        
    Returns:
        str: JSON格式的操作结果
    """
    global camera_controller
    if camera_controller is None:
        return json.dumps({"success": False, "message": "摄像头系统未初始化"}, ensure_ascii=False)
    
    result = camera_controller.set_display_mode(mode)
    return json.dumps(result, ensure_ascii=False)

def shutdown_camera_system() -> str:
    """关闭摄像头系统
    
    Returns:
        str: JSON格式的操作结果
    """
    global camera_controller
    if camera_controller is None:
        return json.dumps({"success": False, "message": "摄像头系统未初始化"}, ensure_ascii=False)
    
    result = camera_controller.shutdown_camera()
    camera_controller = None
    return json.dumps(result, ensure_ascii=False)

# 全局摄像头控制器实例
camera_controller: Optional[WheeltecCameraController] = None

def main():
    """主函数 - 独立运行模式"""
    print("Wheeltec摄像头控制器")
    print("支持的摄像头类型: astra_pro, gemini, wheeltec_usbcam")
    print("控制键:")
    print("  ESC - 退出")
    print("  S   - 切换显示模式")
    print("="*50)
    
    # 创建控制器
    controller = WheeltecCameraController()
    
    try:
        # 初始化摄像头（默认使用USB摄像头）
        result = controller.initialize_camera(
            camera_type="wheeltec_usbcam",
            display_mode="three"
        )
        
        if not result["success"]:
            print(f"初始化失败: {result['message']}")
            return
            
        print(f"初始化成功: {result['message']}")
        
        # 启动数据流
        result = controller.start_camera_stream()
        if not result["success"]:
            print(f"启动失败: {result['message']}")
            return
            
        print(f"启动成功: {result['message']}")
        print("摄像头流已启动，按ESC退出...")
        
        # 等待用户操作
        while controller.is_running:
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n检测到Ctrl+C，正在关闭...")
    except Exception as e:
        print(f"运行错误: {e}")
    finally:
        # 清理资源
        controller.shutdown_camera()
        print("摄像头系统已关闭")

if __name__ == "__main__":
    main()