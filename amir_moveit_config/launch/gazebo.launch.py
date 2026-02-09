# English comments only.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.actions import ExecuteProcess


def _is_gzclient_process(action) -> bool:
    # Match gzclient ExecuteProcess started by gazebo_ros gzclient.launch.py
    if not isinstance(action, ExecuteProcess):
        return False
    try:
        cmd = action.cmd
    except Exception:
        return False
    if not cmd:
        return False
    return any("gzclient" in str(x) for x in cmd)


def generate_launch_description():
    # Launch args
    paused = LaunchConfiguration("paused")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    spawn_delay = LaunchConfiguration("spawn_delay")

    # Paths
    amir_moveit_config_share = get_package_share_directory("amir_moveit_config")
    xacro_path = os.path.join(amir_moveit_config_share, "config", "amir.urdf.xacro")

    # robot_description from xacro
    robot_description = ParameterValue(
        Command(["xacro", " ", xacro_path, " ", "ros2_control_hardware_type:=gazebo"]),
        value_type=str,
    )

    # Gazebo Classic (gazebo_ros) launch files
    gazebo_ros_share = get_package_share_directory("gazebo_ros")
    gzserver_launch = os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
    gzclient_launch = os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")

    # Use an empty world
    world_path = "worlds/empty.world"

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch),
        launch_arguments={
            "world": world_path,
            "pause": paused,
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzclient_launch),
        condition=IfCondition(gazebo_gui),
    )

    # Publish robot_description to TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True},
        ],
    )

    # Spawn entity in Gazebo from /robot_description
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "robot",
        ],
    )

    # GUI enabled: spawn N seconds after gzclient ExecuteProcess starts
    spawn_after_gzclient_start = RegisterEventHandler(
        OnProcessStart(
            target_action=_is_gzclient_process,
            on_start=[
                TimerAction(period=spawn_delay, actions=[spawn_entity]),
            ],
        )
    )

    # GUI disabled: spawn after N seconds from launch start
    spawn_without_gui = TimerAction(
        period=spawn_delay,
        actions=[spawn_entity],
        condition=UnlessCondition(gazebo_gui),
    )

    # rqt joint trajectory controller (ROS2)
    rqt_jtc = Node(
        package="rqt_joint_trajectory_controller",
        executable="rqt_joint_trajectory_controller",
        output="screen",
    )

    # Spawn controllers (after Gazebo + controller_manager is up)
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    spawner_gripper = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay spawners a bit to avoid race at startup
    delayed_spawners = TimerAction(
        period=2.0,
        actions=[spawner_jsb, spawner_arm, spawner_gripper],
    )

    return LaunchDescription([
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("spawn_delay", default_value="2.0"),

        gazebo_server,
        gazebo_client,

        robot_state_publisher,

        spawn_after_gzclient_start,
        spawn_without_gui,

        delayed_spawners,
        rqt_jtc,
    ])

