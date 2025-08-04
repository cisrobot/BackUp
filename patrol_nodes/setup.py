from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'patrol_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='haehae_00@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'map_send = patrol_nodes.map_send_node:main',
        'map_receive = patrol_nodes.map_receive_node:main',
        'app_to_ros2 = patrol_nodes.app_to_ros2_node:main',
        'ros2_to_app = patrol_nodes.ros2_to_app_node:main',
        'ros2_internal_publisher = patrol_nodes.ros2_internal_publisher_node:main',
        'dummy_data_publisher = patrol_nodes.dummy_data_publisher:main',
        'firestore_bridge = patrol_nodes.firestore_bridge:main',
        'no_dummmmy = patrol_nodes.no_dummmmy:main',
        'dummy_gps_node = patrol_nodes.dummy_gps_publisher:main',      # 새로 추가된 더미 GPS 발행 노드
        ],
    },
)
