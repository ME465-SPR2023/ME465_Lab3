from launch_ros.actions import Node, SetParameter
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    sim_share = get_package_share_directory("ME465_Sim")
    description_share = get_package_share_directory("ME465_Description")
    lab3_share = get_package_share_directory("ME465_Lab3")
    with open(os.path.join(lab3_share, "simulation.yaml")) as f:
        params = yaml.safe_load(f)
    return LaunchDescription([
        DeclareLaunchArgument(
            name="simulation",
            default_value="false",
            description="Start simulation",
        ),
        DeclareLaunchArgument(
            name="visualization",
            default_value="true",
            description="Show visualization",
        ),
        DeclareLaunchArgument(
            name="node",
            default_value="true",
            description="Run the lab 3 node",
        ),
        SetParameter(
            name="use_sim_time",
            value=LaunchConfiguration("simulation"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sim_share, "launch", "simulation-launch.py"),
            ),
            condition=IfCondition(LaunchConfiguration("simulation")),
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("simulation")),
            package="apriltag_ros",
            executable="apriltag_node",
            parameters=[params["apriltag"]["ros__parameters"]],
            remappings=(
                ("/image_rect", "/camera/image_raw"),
                ("/camera_info", "/camera/camera_info"),
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description_share, "launch", "description-launch.py"),
            ),
        ),
        Node(
            package="ME465_Lab3",
            executable="lab3_node",
            parameters=[params["lab3_node"]["ros__parameters"]],
            condition=IfCondition(LaunchConfiguration("node")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(LaunchConfiguration("visualization")),
            arguments=["-d", os.path.join(lab3_share, "lab3.rviz")],
        ),
    ])
