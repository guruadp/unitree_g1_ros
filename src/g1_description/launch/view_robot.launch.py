from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_gazebo = LaunchConfiguration("use_gazebo")
    model = LaunchConfiguration("model")  # <-- use the launch arg

    g1_share = get_package_share_directory("g1_description")
    urdf_path = os.path.join(g1_share, "urdf", f"g1_23dof.urdf")  # default; see note below

    # If you truly have multiple URDF filenames (g1_23dof.urdf / g1_29dof.urdf),
    # you can't f-string a LaunchConfiguration at Python runtime.
    # Either:
    #   A) Keep separate launch files per model, OR
    #   B) Use xacro and pass model as a parameter, OR
    #   C) Use substitutions (advanced).
    #
    # For now I'm keeping a fixed URDF file; swap it manually or move to xacro.

    with open(urdf_path, "r") as f:
        robot_desc = f.read()

    gazebo_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(g1_share, "launch", "gazebo.launch.py")),
        condition=IfCondition(use_gazebo),
        launch_arguments={"model": model}.items(),  # <-- IMPORTANT
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": use_gazebo}],
        output="screen",
    )

    jsp_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=UnlessCondition(use_gazebo),
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        condition=UnlessCondition(use_gazebo),
        parameters=[{"use_sim_time": use_gazebo}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="false",
            description="Start Gazebo and use /clock",
        ),
        DeclareLaunchArgument(
            "model",
            default_value="23dof",
            description="G1 model variant: 23dof or 29dof",
        ),

        gazebo_include,
        rsp,
        jsp_gui,
        rviz,
    ])
