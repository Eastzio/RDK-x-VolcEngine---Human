#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AI助手配置文件
"""

import os
from dataclasses import dataclass
from typing import Dict, Any

@dataclass
class SystemConfig:
    """系统配置"""
    # 串口配置
    arm_serial_port: str = "/dev/ttyUSB0"
    lift_serial_port: str = "/dev/dmmotor"
    base_serial_port: str = "/dev/ttyUSB1"
    
    # 摄像头配置
    camera_device: int = 0
    camera_width: int = 640
    camera_height: int = 480
    camera_fps: int = 30
    
    # 音频配置
    audio_sample_rate: int = 16000
    audio_chunk_size: int = 1024
    audio_channels: int = 1
    
    # 传感器配置
    i2c_bus: int = 1
    sensor_update_interval: float = 1.0

@dataclass
class AIConfig:
    """AI配置"""
    # API配置
    api_key: str = os.getenv("ARK_API_KEY", "sk-b18aaf41432e4d209bea69b6bf3442e09naqtyf53v78o15q")
    base_url: str = "https://ai-gateway.vei.volces.com/v1"
    model: str = "doubao-1.5-thinking-pro-vision"
    
    # 对话配置
    max_tokens: int = 1000
    temperature: float = 0.7
    max_history_length: int = 20
    
    # 语音配置
    wake_word: str = "互联你好"
    speech_timeout: float = 5.0
    wake_word_timeout: float = 1.0
    
    # 系统提示词
    system_prompt: str = """
你是一个助老助残机器人的AI助手，名字叫"小智"。你的主要职责是：

1. 帮助老年人和残障人士完成日常任务
2. 提供友善、耐心、清晰的交流
3. 通过控制机器人硬件来协助用户
4. 监测环境信息，确保用户安全
5. 在紧急情况下提供帮助

你可以控制的硬件包括：
- 机器人底盘移动
- 双臂机械臂
- 升降关节
- 摄像头拍照
- 环境传感器读取
- 语音播放

请始终保持友善、耐心的态度，用简单易懂的语言与用户交流。
当用户需要帮助时，主动询问具体需求并提供相应的协助。
"""

@dataclass
class UIConfig:
    """界面配置"""
    window_title: str = "AI助手 - 助老助残机器人"
    window_size: str = "800x600"
    log_max_lines: int = 1000
    update_interval: int = 1000  # 毫秒

def load_config() -> Dict[str, Any]:
    """加载配置"""
    return {
        "system": SystemConfig(),
        "ai": AIConfig(),
        "ui": UIConfig()
    }

def save_config(config: Dict[str, Any], config_file: str = "config.json"):
    """保存配置"""
    import json
    
    # 转换为可序列化的字典
    serializable_config = {}
    for key, value in config.items():
        if hasattr(value, '__dict__'):
            serializable_config[key] = value.__dict__
        else:
            serializable_config[key] = value
    
    with open(config_file, 'w', encoding='utf-8') as f:
        json.dump(serializable_config, f, indent=2, ensure_ascii=False)

if __name__ == "__main__":
    # 生成默认配置文件
    config = load_config()
    save_config(config)
    print("配置文件已生成: config.json")