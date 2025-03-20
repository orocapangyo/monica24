from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent_wifi_node',
            arguments=["udp4", "--port", "8090", "-v4"],
            output='screen'
        )
    ])