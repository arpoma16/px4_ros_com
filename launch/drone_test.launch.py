from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    #package_dir = get_package_share_directory('px4_ros_com',')

    # Declare the namespace argument (it can be provided when launching)
    namespace = LaunchConfiguration('namespace', default='px4_3')

    mav_sys_id = LaunchConfiguration('mav_sys_id', default='3')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='px4_3',
            description='Namespace of the nodes'
        ),
        DeclareLaunchArgument(
            'mav_sys_id',
            default_value='3',
            description='MAVLink system ID'
        ),
        Node(
            package='px4_ros_com',
            namespace=namespace,
            executable='offboard_control_status.py',
            name='control',
            output='screen',
            parameters= [
                {'MAV_SYS_ID': mav_sys_id},
                {'namespace': namespace}
            ]
        )
    ])