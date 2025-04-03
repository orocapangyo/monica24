import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rviz_config_dir = os.path.join(
            get_package_share_directory('yahboomcar_lidar'),
            'rviz',
            'rplidar.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )
    
    # base_link → laser_link 트랜스폼 (lidar의 위치 설정)
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser',
        arguments=['-0.0046412', '0' , '0.094079',
                   '0','0','0','1',
                   'base_link','laser_link']
    )
    
    # imu_link → imu_frame 트랜스폼 (lidar의 회전 처리)
    laser_link_to_laser_frame_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_link_to_imu_frame',
        arguments=['0', '0', '0', 
                   '0', '0', '0', '1', 
                   'laser_link', 'laser_frame']
    )
    
    

    return LaunchDescription([
        base_link_to_laser_tf_node,
        laser_link_to_laser_frame_tf_node,
        rviz_node
    ])
    