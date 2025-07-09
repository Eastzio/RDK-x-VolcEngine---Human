#!/bin/bash
# AI助手启动脚本
# 适用于ARM RDK X5平台

echo "=== AI助手启动脚本 ==="
echo "检查环境和依赖..."

# 检查Python版本
PYTHON_VERSION=$(python3 --version 2>&1)
echo "Python版本: $PYTHON_VERSION"

# 检查是否在正确目录
if [ ! -f "ai_assistant.py" ]; then
    echo "错误: 未找到ai_assistant.py文件"
    echo "请确保在test目录下运行此脚本"
    exit 1
fi

# 设置环境变量
export PYTHONPATH="$PYTHONPATH:$(pwd)/.."

# 检查API密钥
if [ -z "$ARK_API_KEY" ]; then
    echo "警告: 未设置ARK_API_KEY环境变量"
    echo "将使用代码中的默认密钥"
fi

# 检查音频设备
echo "检查音频设备..."
arecord -l 2>/dev/null || echo "警告: 未检测到录音设备"
aplay -l 2>/dev/null || echo "警告: 未检测到播放设备"

# 检查摄像头设备
echo "检查摄像头设备..."
ls /dev/video* 2>/dev/null || echo "警告: 未检测到摄像头设备"

# 检查串口设备
echo "检查串口设备..."
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "警告: 未检测到串口设备"

echo "启动AI助手..."
echo "说出'互联你好'来唤醒助手"
echo "按Ctrl+C退出"
echo ""

# 启动助手
python3 ai_assistant.py "$@"