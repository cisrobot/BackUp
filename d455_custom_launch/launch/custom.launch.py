import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("d455_custom_launch"), "params", "d455_config.yaml"
        ),
    )
    return LaunchDescription([
        # 런치 인자 선언
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='d455_camera',
            output='screen',
            parameters=[param_dir],
        )
    ])