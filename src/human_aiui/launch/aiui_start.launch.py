from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    human_mic = Node(
        package="human_mic_ros2",
        executable="human_mic",
        output='screen',
        parameters=[{"usart_port_name": "/dev/wheeltec_mic",
                    "serial_baud_rate": 115200}]
    )

    human_aiui = Node(
        package="human_aiui",
        executable="human_aiui",
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(human_mic)
    ld.add_action(human_aiui)
    
    return ld