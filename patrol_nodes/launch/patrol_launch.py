from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='patrol_nodes',
            executable='no_dummmmy',
            name='no_dummmmy_node',
            output='screen'
        ),
        Node(
            package='patrol_nodes',
            executable='firestore_bridge',
            name='firestore_bridge_node',
            output='screen'
        ),
    ])