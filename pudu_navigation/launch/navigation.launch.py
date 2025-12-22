from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

import os

from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    
    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace)}
    )

    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='pudu',
        description='Top-level namespace'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=os.path.join(get_package_share_directory("pudu_navigation"), "config", "navigation.yaml"),
        description='Localization pamareters in yaml file to load'
    )
    
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        namespace=namespace,
        parameters=[configured_params],
        arguments=[],
        remappings=remappings,
    )
    
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        namespace=namespace,
        arguments=[],
        parameters=[configured_params],
    )
    
    planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        namespace=namespace,
        parameters=[configured_params],
        arguments=[],
        remappings=remappings,
    )
    
    controller_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        namespace=namespace,
        parameters=[configured_params],
        arguments=[],
        remappings=remappings + [('cmd_vel', 'omni_wheel_drive_controller/cmd_vel')],
    )
    
    behavior_cmd = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        namespace=namespace,
        parameters=[configured_params],
        arguments=[],
        remappings=remappings + [('cmd_vel', 'omni_wheel_drive_controller/cmd_vel')],
    )

    ld = LaunchDescription()
    
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(bt_navigator_cmd)
    ld.add_action(planner_cmd)
    ld.add_action(controller_cmd)
    ld.add_action(behavior_cmd)
    ld.add_action(lifecycle_manager_cmd)

    return ld
