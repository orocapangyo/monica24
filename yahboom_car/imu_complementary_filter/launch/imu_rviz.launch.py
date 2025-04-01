import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():	
    rviz_config_dir = os.path.join(
            get_package_share_directory('imu_complementary_filter'),
            'rviz',
            'yahboom_imu.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'
    )
    
    # base_link → imu_link 트랜스폼 (IMU의 위치 설정)
    base_link_to_imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_imu',
        arguments=['-0.002999', '-0.0030001','0.031701',
                   '0','0','0','1',
                   'base_link','imu_link']
    ) 
    
    # imu_link → imu_frame 트랜스폼 (IMU의 회전 처리)
    imu_link_to_imu_frame_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_link_to_imu_frame',
        arguments=['0', '0', '0', 
                   '0', '0', '0', '1', 
                   'imu_link', 'imu_frame']
    )
    
    return LaunchDescription([
        base_link_to_imu_tf_node,
        imu_link_to_imu_frame_tf_node,
        rviz_node
    ])