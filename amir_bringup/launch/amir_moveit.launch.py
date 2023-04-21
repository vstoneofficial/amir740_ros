import os

from ament_index_python.packages import get_package_share_path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    # Get URDF via xacro
    urdf_path = os.path.join(
        get_package_share_directory('amir_moveit_config'),
        'config',
        'amir.urdf.xacro')

    default_rviz_config_path = get_package_share_path('amir_bringup') / 'rviz/amir_moveit.rviz'

    model_arg = DeclareLaunchArgument(name='model',
                                      default_value=str(urdf_path),
                                      description='Absolute path to robot urdf file')
    
    rviz_arg = DeclareLaunchArgument(name='rvizconfig',
                                     default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file')
    
    ros2_control_hardware_type = DeclareLaunchArgument(
        name="ros2_control_hardware_type",
        default_value="mock_components",
        description="ROS2 control hardware interface type to use for the launch file -- possible values: [mock_components, amir_hardware]",
    )

    doc = xacro.process_file(urdf_path, 
                             mappings={'ros2_control_hardware_type': "amir_hardware"}
    )

    robot_description_real = doc.toprettyxml(indent='  ')

    amir_controllers = os.path.join(
        get_package_share_directory('amir_moveit_config'),
        'config',
        'ros2_controllers.yaml'
    )

    control_node_real = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description_real}, amir_controllers],
        output="both",
    )
 
    robot_state_pub_node_real = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_description_real}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output='screen',
        arguments=["-d", LaunchConfiguration('rvizconfig')],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # include move_group launch file from moveit_config
    launch_move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('amir_moveit_config'),
                'launch/move_group.launch.py'))
    )

    nodes = [
        model_arg,
        rviz_arg,
        ros2_control_hardware_type,
        control_node_real,
        robot_state_pub_node_real,
        launch_move_group,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(nodes)
