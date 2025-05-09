from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(name='robot_name',default_value=str('robot1'))
    name_space = DeclareLaunchArgument(name='RobotName',default_value="robot1")
    urdf_tutorial_path = get_package_share_path('yahboomcar_description')
    model_path = 'urdf/MicroROS_robot1.urdf'
    default_model_path = urdf_tutorial_path / model_path

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file')
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('robot_name'),
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=LaunchConfiguration('robot_name')
    )

    tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=LaunchConfiguration('robot_name'),
        arguments=['0', '0', '0.05', '0.0', '0.0', '0.0', 'robot1/base_footprint', 'robot1/base_link'],
    )

    return LaunchDescription([
        model_arg,
        robot_name_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        tf_base_footprint_to_base_link
    ])
