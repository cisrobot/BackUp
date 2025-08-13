#!/usr/bin/env python3

"""
- Scout Mini 로봇 기본 구동 환경 런치
- URDF(xacro)로 로봇 모델을 로드하고 robot_state_publisher로 TF 브로드캐스트
- ros2_control_node와 설정 파일(scout_mini.yaml)로 하드웨어 인터페이스 및 컨트롤러 구동
- joint_state_broadcaster, scout_mini_base_controller 스포너 실행
- twist_mux로 다중 속도 명령 소스 관리, teleop_twist_keyboard로 키보드 조종 지원
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    robot_config_dir = LaunchConfiguration(
        "robot_config_dir",
        default=os.path.join(
            get_package_share_directory("scout_mini_base"), "config", "scout_mini.yaml"
        ),
    )

    #scout_mini_description에서 로봇 xacro 호출
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("scout_mini_description"),
                    "urdf",
                    "scout_mini.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description]
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, robot_config_dir],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "scout_mini_base_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="twist_mux",
                executable="twist_mux",
                output="screen",
                parameters=[robot_config_dir],
                remappings={
                    ("/cmd_vel_out", "/scout_mini_base_controller/cmd_vel_unstamped")
                },
            ),
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="teleop_twist_keyboard",
                output="screen",
                prefix="xterm -e"
            ),
        ]
    )