#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI助手 - 基于火山方舟API的助老助残机器人大脑
支持语音交互、图像理解、机器人控制
运行在ARM RDK X5平台的Linux系统
"""

import os
import sys
import json
import time
import threading
import queue
import cv2
import numpy as np
import base64
import io
from typing import Dict, List, Optional, Any
from dataclasses import dataclass
from enum import Enum
import logging

# 语音相关
try:
    import speech_recognition as sr
    import pyaudio
    HAS_SPEECH = True
except ImportError:
    HAS_SPEECH = False
    print("Warning: 语音识别模块未安装，请运行: pip install SpeechRecognition pyaudio")

# AI API
try:
    from volcenginesdkarkruntime import Ark
    HAS_ARK = True
except ImportError:
    HAS_ARK = False
    print("Warning: 火山方舟SDK未安装，请运行: pip install volcengine-python-sdk[ark]")

# 导入机器人控制模块
try:
    from both_arm import JointConfig, DualArmController # Changed BothArmController to DualArmController
    from dm_con import LiftJointController
    from wheeltec_base_controller import WheeltecBaseController
    from wheeltec_camera_controller import WheeltecCameraController
    from tts_player import TTSPlayer, TTSConfig
    from read_all import EnvironmentSensor
    HAS_ROBOT_MODULES = True
except ImportError as e:
    HAS_ROBOT_MODULES = False
    print(f"Warning: 机器人控制模块导入失败: {e}")

# GUI
try:
    import tkinter as tk
    from tkinter import ttk, scrolledtext
    HAS_GUI = True
except ImportError:
    HAS_GUI = False

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class AssistantState(Enum):
    """助手状态枚举"""
    IDLE = "idle"                    # 空闲状态
    LISTENING = "listening"          # 监听唤醒词
    RECORDING = "recording"          # 录音中
    PROCESSING = "processing"        # 处理中
    SPEAKING = "speaking"            # 播放语音
    ERROR = "error"                  # 错误状态

@dataclass
class AIConfig:
    """AI配置类"""
    api_key: str = "sk-b18aaf41432e4d209bea69b6bf3442e09naqtyf53v78o15q"
    base_url: str = "https://ai-gateway.vei.volces.com/v1"
    model: str = "doubao-1.5-thinking-pro-vision"
    wake_word: str = "互联你好"
    max_tokens: int = 1000
    temperature: float = 0.7

class RobotFunctionCaller:
    """机器人功能调用器"""
    
    def __init__(self):
        self.arm_controller = None
        self.lift_controller = None
        self.base_controller = None
        self.camera_controller = None
        self.env_sensor = None
        self.tts_player = None
        self._initialize_controllers()
    
    def _initialize_controllers(self):
        """初始化所有控制器"""
        if not HAS_ROBOT_MODULES:
            logger.warning("机器人控制模块未可用")
            return
            
        try:
            # 初始化双臂控制器
            self.arm_controller = DualArmController()  # Changed from BothArmController() to DualArmController()
            
            # 初始化升降控制器
            self.lift_controller = LiftJointController()
            
            # 初始化底盘控制器
            self.base_controller = WheeltecBaseController()
            
            # 初始化摄像头控制器
            self.camera_controller = WheeltecCameraController()
            
            # 初始化环境传感器
            self.env_sensor = EnvironmentSensor()
            
            # 初始化TTS播放器
            tts_config = TTSConfig(
                voice_name="default",
                speed=60,
                volume=0.8,
                play_audio=True,
                engine_type="pyttsx3"
            )
            self.tts_player = TTSPlayer(tts_config)
            self.tts_player.initialize()
            
            logger.info("机器人控制器初始化完成")
            
        except Exception as e:
            logger.error(f"控制器初始化失败: {e}")
    
    def get_available_tools(self) -> List[Dict]:
        """获取可用的工具列表"""
        tools = [
            {
                "type": "function",
                "function": {
                    "name": "move_robot_base",
                    "description": "控制机器人底盘移动",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "direction": {
                                "type": "string",
                                "enum": ["forward", "backward", "left", "right", "stop"],
                                "description": "移动方向"
                            },
                            "speed": {
                                "type": "number",
                                "description": "移动速度 (0.1-1.0)",
                                "minimum": 0.1,
                                "maximum": 1.0
                            },
                            "duration": {
                                "type": "number",
                                "description": "移动持续时间(秒)",
                                "minimum": 0.1,
                                "maximum": 10.0
                            }
                        },
                        "required": ["direction"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "control_arm",
                    "description": "控制机械臂",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "arm": {
                                "type": "string",
                                "enum": ["left", "right", "both"],
                                "description": "控制的手臂"
                            },
                            "action": {
                                "type": "string",
                                "enum": ["wave", "grab", "release", "point", "home"],
                                "description": "手臂动作"
                            }
                        },
                        "required": ["arm", "action"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "control_lift",
                    "description": "控制升降机构",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "action": {
                                "type": "string",
                                "enum": ["up", "down", "stop"],
                                "description": "升降动作"
                            },
                            "height": {
                                "type": "number",
                                "description": "目标高度(cm)",
                                "minimum": 0.0,
                                "maximum": 30.0
                            }
                        },
                        "required": ["action"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "capture_image",
                    "description": "拍摄照片进行分析",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "camera_type": {
                                "type": "string",
                                "enum": ["color", "depth", "ir"],
                                "description": "摄像头类型"
                            }
                        },
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "read_environment",
                    "description": "读取环境信息(温度、湿度、气压等)",
                    "parameters": {
                        "type": "object",
                        "properties": {},
                        "required": []
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "speak_text",
                    "description": "播放语音",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "text": {
                                "type": "string",
                                "description": "要播放的文本"
                            }
                        },
                        "required": ["text"]
                    }
                }
            }
        ]
        return tools
    
    def execute_function(self, function_name: str, parameters: Dict) -> Dict:
        """执行功能调用"""
        try:
            if function_name == "move_robot_base":
                return self._move_robot_base(**parameters)
            elif function_name == "control_arm":
                return self._control_arm(**parameters)
            elif function_name == "control_lift":
                return self._control_lift(**parameters)
            elif function_name == "capture_image":
                return self._capture_image(**parameters)
            elif function_name == "read_environment":
                return self._read_environment()
            elif function_name == "speak_text":
                return self._speak_text(**parameters)
            else:
                return {"success": False, "error": f"未知功能: {function_name}"}
                
        except Exception as e:
            logger.error(f"执行功能 {function_name} 失败: {e}")
            return {"success": False, "error": str(e)}
    
    def _move_robot_base(self, direction: str, speed: float = 0.5, duration: float = 1.0) -> Dict:
        """移动机器人底盘"""
        if not self.base_controller:
            return {"success": False, "error": "底盘控制器未初始化"}
        
        try:
            if direction == "forward":
                result = self.base_controller.move_forward(speed, duration)
            elif direction == "backward":
                result = self.base_controller.move_backward(speed, duration)
            elif direction == "left":
                result = self.base_controller.turn_left(speed, duration)
            elif direction == "right":
                result = self.base_controller.turn_right(speed, duration)
            elif direction == "stop":
                result = self.base_controller.stop()
            else:
                return {"success": False, "error": f"未知方向: {direction}"}
            
            return {"success": True, "result": result, "message": f"机器人{direction}移动完成"}
            
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def _control_arm(self, arm: str, action: str) -> Dict:
        """控制手臂"""
        if not self.arm_controller:
            return {"success": False, "error": "手臂控制器未初始化"}
        
        try:
            if action == "wave":
                result = self.arm_controller.wave_gesture(arm)
            elif action == "grab":
                result = self.arm_controller.grab_object(arm)
            elif action == "release":
                result = self.arm_controller.release_object(arm)
            elif action == "point":
                result = self.arm_controller.point_gesture(arm)
            elif action == "home":
                result = self.arm_controller.go_home_position(arm)
            else:
                return {"success": False, "error": f"未知动作: {action}"}
            
            return {"success": True, "result": result, "message": f"{arm}手臂{action}动作完成"}
            
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def _control_lift(self, action: str, height: Optional[float] = None) -> Dict:
        """控制升降关节"""
        if not self.lift_controller:
            return {"success": False, "error": "升降控制器未初始化"}
        
        try:
            if action == "up":
                result = self.lift_controller.move_up(height or 5.0)
            elif action == "down":
                result = self.lift_controller.move_down(height or 5.0)
            elif action == "stop":
                result = self.lift_controller.stop()
            elif action == "home":
                result = self.lift_controller.go_home()
            else:
                return {"success": False, "error": f"未知动作: {action}"}
            
            return {"success": True, "result": result, "message": f"升降关节{action}动作完成"}
            
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def _capture_image(self, camera_type: str = "color") -> Dict:
        """拍摄图像"""
        if not self.camera_controller:
            return {"success": False, "error": "摄像头控制器未初始化"}
        
        try:
            result = self.camera_controller.capture_single_frame()
            if result["success"]:
                # 将图像转换为base64编码
                frame = result["frames"].get(camera_type)
                if frame is not None:
                    _, buffer = cv2.imencode('.jpg', frame)
                    img_base64 = base64.b64encode(buffer).decode('utf-8')
                    return {
                        "success": True, 
                        "image_base64": img_base64,
                        "message": f"{camera_type}图像拍摄完成"
                    }
                else:
                    return {"success": False, "error": f"未获取到{camera_type}图像"}
            else:
                return result
                
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def _read_environment(self) -> Dict:
        """读取环境信息"""
        if not self.env_sensor:
            return {"success": False, "error": "环境传感器未初始化"}
        
        try:
            result = self.env_sensor.read_sensor_data()
            return {"success": True, "data": result, "message": "环境数据读取完成"}
            
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def _speak_text(self, text: str) -> Dict:
        """播放语音"""
        if not self.tts_player:
            return {"success": False, "error": "TTS播放器未初始化"}
        
        try:
            self.tts_player.synthesize_and_play(text)
            return {"success": True, "message": f"语音播放完成: {text}"}
            
        except Exception as e:
            return {"success": False, "error": str(e)}

class VoiceRecognizer:
    """语音识别器"""
    
    def __init__(self, wake_word: str = "互联你好"):
        self.wake_word = wake_word.lower()
        self.recognizer = None
        self.microphone = None
        self.is_listening = False
        
        if HAS_SPEECH:
            self.recognizer = sr.Recognizer()
            self.microphone = sr.Microphone()
            # 调整环境噪音
            with self.microphone as source:
                self.recognizer.adjust_for_ambient_noise(source)
    
    def listen_for_wake_word(self, timeout: float = 1.0) -> bool:
        """监听唤醒词"""
        if not HAS_SPEECH:
            return False
            
        try:
            with self.microphone as source:
                # 短时间监听
                audio = self.recognizer.listen(source, timeout=timeout, phrase_time_limit=3)
            
            # 识别语音
            text = self.recognizer.recognize_google(audio, language='zh-CN')
            logger.info(f"识别到语音: {text}")
            
            # 检查是否包含唤醒词
            return self.wake_word in text.lower()
            
        except sr.WaitTimeoutError:
            return False
        except sr.UnknownValueError:
            return False
        except sr.RequestError as e:
            logger.error(f"语音识别服务错误: {e}")
            return False
        except Exception as e:
            logger.error(f"语音识别错误: {e}")
            return False
    
    def record_speech(self, timeout: float = 5.0) -> Optional[str]:
        """录制并识别语音"""
        if not HAS_SPEECH:
            return None
            
        try:
            with self.microphone as source:
                logger.info("开始录音...")
                audio = self.recognizer.listen(source, timeout=timeout, phrase_time_limit=10)
            
            logger.info("识别语音中...")
            text = self.recognizer.recognize_google(audio, language='zh-CN')
            logger.info(f"识别结果: {text}")
            return text
            
        except sr.WaitTimeoutError:
            logger.warning("录音超时")
            return None
        except sr.UnknownValueError:
            logger.warning("无法识别语音")
            return None
        except sr.RequestError as e:
            logger.error(f"语音识别服务错误: {e}")
            return None
        except Exception as e:
            logger.error(f"录音错误: {e}")
            return None

class AIAssistant:
    """AI助手主类"""
    
    def __init__(self, config: AIConfig):
        self.config = config
        self.state = AssistantState.IDLE
        self.client = None
        self.voice_recognizer = VoiceRecognizer(config.wake_word)
        self.function_caller = RobotFunctionCaller()
        self.conversation_history = []
        
        # 线程控制
        self.running = False
        self.main_thread = None
        self.state_lock = threading.Lock()
        
        # 初始化AI客户端
        self._initialize_ai_client()
    
    def _initialize_ai_client(self):
        """初始化AI客户端"""
        if not HAS_ARK:
            logger.error("火山方舟SDK未安装")
            return
            
        try:
            self.client = Ark(
                api_key=self.config.api_key,
                base_url=self.config.base_url
            )
            logger.info("AI客户端初始化成功")
        except Exception as e:
            logger.error(f"AI客户端初始化失败: {e}")
    
    def _set_state(self, new_state: AssistantState):
        """设置助手状态"""
        with self.state_lock:
            old_state = self.state
            self.state = new_state
            logger.info(f"状态变更: {old_state.value} -> {new_state.value}")
    
    def _call_ai_api(self, messages: List[Dict], tools: Optional[List[Dict]] = None) -> Optional[Dict]:
        """调用AI API"""
        if not self.client:
            logger.error("AI客户端未初始化")
            return None
            
        try:
            kwargs = {
                "model": self.config.model,
                "messages": messages,
                "max_tokens": self.config.max_tokens,
                "temperature": self.config.temperature
            }
            
            if tools:
                kwargs["tools"] = tools
                kwargs["tool_choice"] = "auto"
            
            response = self.client.chat.completions.create(**kwargs)
            return response
            
        except Exception as e:
            logger.error(f"AI API调用失败: {e}")
            return None
    
    def _process_user_input(self, user_input: str, image_base64: Optional[str] = None):
        """处理用户输入"""
        self._set_state(AssistantState.PROCESSING)
        
        try:
            # 构建消息
            content = [{"type": "text", "text": user_input}]
            
            if image_base64:
                content.append({
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{image_base64}"
                    }
                })
            
            messages = self.conversation_history + [{
                "role": "user",
                "content": content
            }]
            
            # 获取可用工具
            tools = self.function_caller.get_available_tools()
            
            # 调用AI API
            response = self._call_ai_api(messages, tools)
            
            if not response:
                self._speak_response("抱歉，我现在无法处理您的请求。")
                return
            
            choice = response.choices[0]
            message = choice.message
            
            # 处理工具调用
            if hasattr(message, 'tool_calls') and message.tool_calls:
                tool_call = message.tool_calls[0]
                function_name = tool_call.function.name
                function_args = json.loads(tool_call.function.arguments)
                
                logger.info(f"执行功能调用: {function_name}({function_args})")
                
                # 执行功能
                function_result = self.function_caller.execute_function(function_name, function_args)
                
                # 将工具调用结果添加到对话历史
                messages.append({
                    "role": "assistant",
                    "content": None,
                    "tool_calls": [{
                        "id": tool_call.id,
                        "type": "function",
                        "function": {
                            "name": function_name,
                            "arguments": json.dumps(function_args)
                        }
                    }]
                })
                
                messages.append({
                    "role": "tool",
                    "tool_call_id": tool_call.id,
                    "content": json.dumps(function_result)
                })
                
                # 再次调用AI获取最终回复
                final_response = self._call_ai_api(messages)
                if final_response:
                    final_message = final_response.choices[0].message.content
                    self._speak_response(final_message)
                    
                    # 更新对话历史
                    self.conversation_history = messages + [{
                        "role": "assistant",
                        "content": final_message
                    }]
                else:
                    self._speak_response("功能执行完成，但无法获取回复。")
            else:
                # 普通回复
                response_text = message.content
                self._speak_response(response_text)
                
                # 更新对话历史
                self.conversation_history.append({
                    "role": "user",
                    "content": user_input
                })
                self.conversation_history.append({
                    "role": "assistant",
                    "content": response_text
                })
            
            # 限制对话历史长度
            if len(self.conversation_history) > 20:
                self.conversation_history = self.conversation_history[-20:]
                
        except Exception as e:
            logger.error(f"处理用户输入失败: {e}")
            self._speak_response("抱歉，处理您的请求时出现了错误。")
        
        finally:
            self._set_state(AssistantState.LISTENING)
    
    def _speak_response(self, text: str):
        """播放回复语音"""
        self._set_state(AssistantState.SPEAKING)
        logger.info(f"AI回复: {text}")
        
        # 使用TTS播放
        result = self.function_caller._speak_text(text)
        if not result["success"]:
            logger.error(f"语音播放失败: {result['error']}")
    
    def _main_loop(self):
        """主循环"""
        logger.info("AI助手启动，等待唤醒词...")
        self._set_state(AssistantState.LISTENING)
        
        while self.running:
            try:
                if self.state == AssistantState.LISTENING:
                    # 监听唤醒词
                    if self.voice_recognizer.listen_for_wake_word():
                        logger.info(f"检测到唤醒词: {self.config.wake_word}")
                        self._speak_response("我在听，请说话。")
                        
                        # 录制用户语音
                        self._set_state(AssistantState.RECORDING)
                        user_speech = self.voice_recognizer.record_speech()
                        
                        if user_speech:
                            # 检查是否需要拍照
                            if any(keyword in user_speech for keyword in ["看看", "拍照", "图像", "照片"]):
                                # 拍摄图像
                                capture_result = self.function_caller._capture_image()
                                if capture_result["success"]:
                                    image_base64 = capture_result["image_base64"]
                                    self._process_user_input(user_speech, image_base64)
                                else:
                                    self._process_user_input(user_speech)
                            else:
                                self._process_user_input(user_speech)
                        else:
                            self._speak_response("没有听清楚，请重新说话。")
                            self._set_state(AssistantState.LISTENING)
                
                elif self.state in [AssistantState.PROCESSING, AssistantState.SPEAKING]:
                    # 等待处理完成
                    time.sleep(0.1)
                
                else:
                    time.sleep(0.1)
                    
            except KeyboardInterrupt:
                logger.info("接收到中断信号")
                break
            except Exception as e:
                logger.error(f"主循环错误: {e}")
                self._set_state(AssistantState.ERROR)
                time.sleep(1)
                self._set_state(AssistantState.LISTENING)
    
    def start(self):
        """启动助手"""
        if self.running:
            logger.warning("助手已在运行")
            return
            
        self.running = True
        self.main_thread = threading.Thread(target=self._main_loop, daemon=True)
        self.main_thread.start()
        logger.info("AI助手已启动")
    
    def stop(self):
        """停止助手"""
        self.running = False
        if self.main_thread and self.main_thread.is_alive():
            self.main_thread.join(timeout=5)
        logger.info("AI助手已停止")
    
    def get_status(self) -> Dict:
        """获取助手状态"""
        return {
            "state": self.state.value,
            "running": self.running,
            "conversation_length": len(self.conversation_history)
        }

class AssistantGUI:
    """助手图形界面"""
    
    def __init__(self, assistant: AIAssistant):
        self.assistant = assistant
        self.root = None
        self.status_label = None
        self.log_text = None
        self.input_entry = None
        
        if HAS_GUI:
            self._create_gui()
    
    def _create_gui(self):
        """创建GUI界面"""
        self.root = tk.Tk()
        self.root.title("AI助手 - 助老助残机器人")
        self.root.geometry("800x600")
        
        # 状态显示
        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(status_frame, text="状态:").pack(side=tk.LEFT)
        self.status_label = ttk.Label(status_frame, text="未启动", foreground="red")
        self.status_label.pack(side=tk.LEFT, padx=10)
        
        # 控制按钮
        button_frame = ttk.Frame(self.root)
        button_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Button(button_frame, text="启动助手", command=self._start_assistant).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="停止助手", command=self._stop_assistant).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="清空日志", command=self._clear_log).pack(side=tk.LEFT, padx=5)
        
        # 日志显示
        log_frame = ttk.LabelFrame(self.root, text="运行日志")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=20)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # 手动输入
        input_frame = ttk.LabelFrame(self.root, text="手动输入")
        input_frame.pack(fill=tk.X, padx=10, pady=5)
        
        self.input_entry = ttk.Entry(input_frame)
        self.input_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)
        self.input_entry.bind('<Return>', self._manual_input)
        
        ttk.Button(input_frame, text="发送", command=self._manual_input).pack(side=tk.RIGHT, padx=5, pady=5)
        
        # 启动状态更新
        self._update_status()
    
    def _start_assistant(self):
        """启动助手"""
        self.assistant.start()
        self._log("助手已启动")
    
    def _stop_assistant(self):
        """停止助手"""
        self.assistant.stop()
        self._log("助手已停止")
    
    def _clear_log(self):
        """清空日志"""
        if self.log_text:
            self.log_text.delete(1.0, tk.END)
    
    def _manual_input(self, event=None):
        """手动输入处理"""
        if not self.input_entry:
            return
            
        text = self.input_entry.get().strip()
        if text:
            self._log(f"手动输入: {text}")
            # 在新线程中处理输入
            threading.Thread(
                target=self.assistant._process_user_input,
                args=(text,),
                daemon=True
            ).start()
            self.input_entry.delete(0, tk.END)
    
    def _log(self, message: str):
        """添加日志"""
        if self.log_text:
            timestamp = time.strftime("%H:%M:%S")
            self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
            self.log_text.see(tk.END)
    
    def _update_status(self):
        """更新状态显示"""
        if self.status_label and self.assistant:
            status = self.assistant.get_status()
            state_text = status["state"]
            
            if status["running"]:
                self.status_label.config(text=f"运行中 - {state_text}", foreground="green")
            else:
                self.status_label.config(text="已停止", foreground="red")
        
        # 每秒更新一次
        if self.root:
            self.root.after(1000, self._update_status)
    
    def run(self):
        """运行GUI"""
        if self.root:
            self.root.mainloop()

def main():
    """主函数"""
    print("=== AI助手 - 助老助残机器人系统 ===")
    print("基于火山方舟API的多模态AI助手")
    print("支持语音交互、图像理解、机器人控制")
    print()
    
    # 检查依赖
    missing_deps = []
    if not HAS_ARK:
        missing_deps.append("volcengine-python-sdk[ark]")
    if not HAS_SPEECH:
        missing_deps.append("SpeechRecognition pyaudio")
    if not HAS_GUI:
        missing_deps.append("tkinter")
    
    if missing_deps:
        print("缺少依赖包，请安装:")
        for dep in missing_deps:
            print(f"  pip install {dep}")
        print()
    
    # 创建配置
    config = AIConfig()
    
    # 创建助手
    assistant = AIAssistant(config)
    
    # 选择运行模式
    if len(sys.argv) > 1 and sys.argv[1] == "--cli":
        # 命令行模式
        print("启动命令行模式...")
        try:
            assistant.start()
            print(f"说出'{config.wake_word}'来唤醒助手")
            print("按Ctrl+C退出")
            
            while True:
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\n正在退出...")
        finally:
            assistant.stop()
    
    else:
        # GUI模式
        if HAS_GUI:
            print("启动图形界面模式...")
            gui = AssistantGUI(assistant)
            gui.run()
        else:
            print("GUI不可用，切换到命令行模式")
            # 回退到命令行模式
            main()

if __name__ == "__main__":
    main()