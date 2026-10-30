from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('unitree_lidar_ros2')

    use_rviz = LaunchConfiguration('use_rviz')
    port = LaunchConfiguration('port')

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz alongside the lidar node',
    )

    declare_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port of the lidar',
    )

    lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters=[{
            'port': port,
            'rotate_yaw_bias': 0.0,
            'range_scale': 0.001,
            'range_bias': 0.0,
            'range_max': 50.0,
            'range_min': 0.0,
            'cloud_frame': 'unilidar_lidar',
            'cloud_topic': 'unilidar/cloud',
            'cloud_scan_num': 18,
            'imu_frame': 'unilidar_imu',
            'imu_topic': 'unilidar/imu',
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'view.rviz')],
        output='log',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_rviz,
        declare_port,
        lidar_node,
        rviz_node,
    ])
