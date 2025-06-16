from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lerobot_o3de',
            executable='put_ball_in_basket_service',
            name='put_ball_in_basket_service',
            output='screen',
            parameters=[],
        ),
    ]) 