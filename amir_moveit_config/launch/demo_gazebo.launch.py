# English comments only.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch args (ROS1 demo_gazebo.launch equivalent)
    pipeline = LaunchConfiguration("pipeline")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    paused = LaunchConfiguration("paused")
    use_rviz = LaunchConfiguration("use_rviz")
    db = LaunchConfiguration("db")
    debug = LaunchConfiguration("debug")

    pkg_share = get_package_share_directory("amir_moveit_config")
    launch_dir = os.path.join(pkg_share, "launch")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "gazebo.launch.py")),
        launch_arguments={
            "paused": paused,
            "gazebo_gui": gazebo_gui,
        }.items(),
    )

    # MoveIt2 bringup (do NOT use demo.launch.py here; it starts ros2_control_node)
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "move_group.launch.py")),
        launch_arguments={
            "pipeline": pipeline,
            "debug": debug,
        }.items(),
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "moveit_rviz.launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "debug": debug,
        }.items(),
    )

    warehouse_db_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "warehouse_db.launch.py")),
        condition=IfCondition(db),
    )

    # Static TFs for virtual joints (if your package uses them)
    static_virtual_joint_tfs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "static_virtual_joint_tfs.launch.py")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("pipeline", default_value="ompl"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("db", default_value="false"),
        DeclareLaunchArgument("debug", default_value="false"),

        gazebo_launch,
        static_virtual_joint_tfs_launch,
        move_group_launch,
        moveit_rviz_launch,
        warehouse_db_launch,
    ])

