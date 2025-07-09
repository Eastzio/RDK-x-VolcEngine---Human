from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取配置文件路径
    config_file = os.path.join(
        get_package_share_directory('human_mic_ros2'),
        'config',
        'param.yaml'
    )
    
    # 声明参数
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to config file'
    )
    
    # 语音模块串口通信节点
    human_mic = Node(
        package="human_mic_ros2",
        executable="human_mic",
        name="human_mic",
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )
    
    # 语音控制处理节点
    voice_control = Node(
        package="human_mic_ros2",
        executable="voice_control",
        name="voice_control",
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )
    
    # 呼叫识别节点
    call_recognition = Node(
        package="human_mic_ros2",
        executable="call_recognition",
        name="call_recognition",
        output='screen',
        parameters=[LaunchConfiguration('config_file')]
    )
    
    ld = LaunchDescription()
    ld.add_action(declare_config_file)
    ld.add_action(human_mic)
    ld.add_action(voice_control)
    ld.add_action(call_recognition)
    
    return ld