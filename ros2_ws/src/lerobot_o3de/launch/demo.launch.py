import pathlib
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    game_launcher_arg = DeclareLaunchArgument(
        "game_launcher",
        default_value="",
        description="Path to the game launcher executable",
    )

    launch_game_launcher = ExecuteProcess(
        cmd=[
            LaunchConfiguration("game_launcher"),
            "+LoadLevel=RoomWithRobotAndBalls",
            "-bg_ConnectToAssetProcessor=0",
            "-r_displayInfo=0",
        ],
        output="screen",
    )

    launch_put_ball_in_basket_service = Node(
        package="lerobot_o3de",
        executable="put_ball_in_basket_service",
        name="put_ball_in_basket_service",
        output="screen",
        parameters=[],
    )

    launch_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(pathlib.Path(__file__).parent.absolute().joinpath('navigation.launch.py'))]),
    )

    return LaunchDescription([
        launch_game_launcher,
        launch_put_ball_in_basket_service,
        launch_navigation,
    ])