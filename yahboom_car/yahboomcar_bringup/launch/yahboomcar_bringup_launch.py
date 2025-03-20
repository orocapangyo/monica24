from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

print("---------------------robot_type = x3---------------------")
def generate_launch_description():
    imu_filter_config = os.path.join(              
        get_package_share_directory('yahboomcar_bringup'),
        'param',
        'imu_filter_param.yaml'
    ) 

    imu_filter_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('imu_complementary_filter'), 'launch'),
            '/complementary_filter.launch.py'])
    )
    

    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_localization'), 'launch'),
            '/ekf.launch.py'])
    )
    
    
     
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('yahboomcar_description'), 'launch'),
         '/description_launch.py'])
    )   
    
    base_link_to_imu_tf_node = Node(
     package='tf2_ros',
     executable='static_transform_publisher',
     name='base_link_to_base_imu',
     arguments=['-0.002999', '-0.0030001','0.031701','0','0','0','base_link','imu_frame']
    ) 
    
    return LaunchDescription([
        imu_filter_node,
        ekf_node,
        base_link_to_imu_tf_node,
        description_launch
    ])
