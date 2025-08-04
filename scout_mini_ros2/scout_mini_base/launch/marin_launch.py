import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

# 네가지 런치 파일 동시 실행 (scout_mini, velodyne, d455 camera, genz_icp)
def generate_launch_description(): 
    scout_mini_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('scout_mini_base'),
                'launch',
                'base_launch.py'
            )
        ])
    )

    velodyne_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('velodyne'),
                'launch',
                'velodyne-all-nodes-VLP16-composed-launch.py'
            )
        ])
    )
    d455_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('d455_custom_launch'),
                'launch',
                'custom.launch.py'
            )
        ])
    )

    genz_icp_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            '/home/marin/marine/src/genz_icp/ros/launch/odometry.launch.py'
        ])
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output = 'screen',
    )

    return LaunchDescription(
        [
            scout_mini_launch,
            velodyne_launch,
            d455_launch,
            genz_icp_odometry_launch,
            rviz
        ]
    )
    