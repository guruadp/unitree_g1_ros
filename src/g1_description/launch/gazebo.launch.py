from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _spawn(context, g1_share):
    model = LaunchConfiguration("model").perform(context)
    urdf_path = os.path.join(g1_share, "urdf", f"g1_{model}.urdf")
    return [
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-entity", "g1_description",
                "-file", urdf_path,
                "-z", "1.05",
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    g1_share = get_package_share_directory("g1_description")
    gazebo_master_uri = LaunchConfiguration("gazebo_master_uri")

    empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "model",
            default_value="23dof",
            description="G1 model variant: 23dof or 29dof"
        ),
        DeclareLaunchArgument(
            "gazebo_master_uri",
            default_value="http://127.0.0.1:11346",
            description="Gazebo master URI to avoid port collisions (default 11346).",
        ),
        SetEnvironmentVariable("GAZEBO_MASTER_URI", gazebo_master_uri),
        SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1"),
        empty_world,
        OpaqueFunction(function=lambda context: _spawn(context, g1_share)),
    ])
