from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='pudu',
        description='Top-level namespace'
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--controller-manager-timeout", "100", 
             "-c", [namespace, '/controller_manager'], 
             'joint_state_broadcaster',
             ],
        output='screen',
    )

    omni_wheel_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--controller-manager-timeout", "100", 
             "-c", [namespace, '/controller_manager'], 
             'omni_wheel_drive_controller',
             ],
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(omni_wheel_drive_controller_spawner)

    return ld