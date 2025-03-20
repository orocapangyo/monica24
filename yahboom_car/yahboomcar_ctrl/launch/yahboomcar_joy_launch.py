from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node1 = Node(
        package='yahboomcar_ctrl',
        executable='yahboom_joy',
    )
    node2 = Node(
            package='joy',
            executable='joy_node',
    )
    launch_description = LaunchDescription([node1,node2])
    return launch_description
