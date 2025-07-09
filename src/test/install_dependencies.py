#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
依赖安装脚本
适用于ARM RDK X5平台的Linux系统
"""

import subprocess
import sys
import os

def run_command(command):
    """执行命令"""
    print(f"执行: {command}")
    try:
        result = subprocess.run(command, shell=True, check=True, 
                              capture_output=True, text=True)
        print(f"成功: {result.stdout}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"失败: {e.stderr}")
        return False

def install_system_dependencies():
    """安装系统依赖"""
    print("=== 安装系统依赖 ===")
    
    # 更新包列表
    run_command("sudo apt update")
    
    # 安装音频相关依赖
    audio_deps = [
        "portaudio19-dev",
        "python3-pyaudio",
        "alsa-utils",
        "pulseaudio",
        "espeak",
        "espeak-data",
        "libespeak1",
        "libespeak-dev",
        "festival",
        "festvox-kallpc16k"
    ]
    
    for dep in audio_deps:
        run_command(f"sudo apt install -y {dep}")
    
    # 安装摄像头相关依赖
    camera_deps = [
        "libopencv-dev",
        "python3-opencv",
        "v4l-utils",
        "uvcdynctrl"
    ]
    
    for dep in camera_deps:
        run_command(f"sudo apt install -y {dep}")
    
    # 安装串口相关依赖
    serial_deps = [
        "python3-serial",
        "setserial"
    ]
    
    for dep in serial_deps:
        run_command(f"sudo apt install -y {dep}")

def install_python_dependencies():
    """安装Python依赖"""
    print("=== 安装Python依赖 ===")
    
    # 升级pip
    run_command(f"{sys.executable} -m pip install --upgrade pip")
    
    # 安装依赖包
    deps_file = "requirements_ai_assistant.txt"
    if os.path.exists(deps_file):
        run_command(f"{sys.executable} -m pip install -r {deps_file}")
    else:
        # 手动安装主要依赖
        main_deps = [
            "volcengine-python-sdk[ark]",
            "SpeechRecognition",
            "opencv-python",
            "numpy",
            "pyttsx3",
            "pygame",
            "gtts",
            "pyserial",
            "keyboard"
        ]
        
        for dep in main_deps:
            run_command(f"{sys.executable} -m pip install {dep}")

def setup_permissions():
    """设置权限"""
    print("=== 设置权限 ===")
    
    # 添加用户到音频组
    run_command("sudo usermod -a -G audio $USER")
    
    # 添加用户到视频组
    run_command("sudo usermod -a -G video $USER")
    
    # 添加用户到串口组
    run_command("sudo usermod -a -G dialout $USER")
    
    # 设置串口权限
    run_command("sudo chmod 666 /dev/ttyUSB*")
    run_command("sudo chmod 666 /dev/ttyACM*")

def test_installation():
    """测试安装"""
    print("=== 测试安装 ===")
    
    test_imports = [
        "import cv2; print(f'OpenCV: {cv2.__version__}')",
        "import speech_recognition; print('SpeechRecognition: OK')",
        "import pyttsx3; print('pyttsx3: OK')",
        "import serial; print('pyserial: OK')",
        "import numpy; print(f'NumPy: {numpy.__version__}')",
        "from volcenginesdkarkruntime import Ark; print('Ark SDK: OK')"
    ]
    
    for test in test_imports:
        try:
            exec(test)
        except ImportError as e:
            print(f"导入失败: {e}")
        except Exception as e:
            print(f"测试错误: {e}")

def main():
    """主函数"""
    print("AI助手依赖安装脚本")
    print("适用于ARM RDK X5平台的Linux系统")
    print()
    
    if os.geteuid() == 0:
        print("请不要使用root用户运行此脚本")
        sys.exit(1)
    
    try:
        # 安装系统依赖
        install_system_dependencies()
        
        # 安装Python依赖
        install_python_dependencies()
        
        # 设置权限
        setup_permissions()
        
        # 测试安装
        test_installation()
        
        print("\n=== 安装完成 ===")
        print("请重新登录或重启系统以使权限设置生效")
        print("然后运行: python3 ai_assistant.py")
        
    except KeyboardInterrupt:
        print("\n安装被中断")
    except Exception as e:
        print(f"\n安装失败: {e}")

if __name__ == "__main__":
    main()