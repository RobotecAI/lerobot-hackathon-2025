from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory('lerobot_o3de'),
        'urdf',
        'so101_new_calib.urdf'
    )
    with open(urdf_path, 'r') as file:
        urdf = file.read()

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            namespace='base',
            name='gui',
            parameters=[{'rate': 30}],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='base',
            name='state_publisher',
            parameters=[{'robot_description': urdf}],
        ),
        Node(
            package='lerobot_o3de',
            executable='joint_states',
            namespace='base',
            name='sim',
        ),
    ])