from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch_ros.actions import Node

import os

def generate_launch_description():

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='pudu',
        description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_file_cmd = DeclareLaunchArgument(
        name='map',
        default_value=os.path.join(get_package_share_directory("pudu_navigation"), "maps", "empty", "empty.yaml"),
        description='Full path to map yaml file to load'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        name='params_file',
        default_value=os.path.join(get_package_share_directory("pudu_navigation"), "config", "localization.yaml"),
        description='Localization pamareters in yaml file to load'
    )

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )
    
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        namespace=namespace,
        parameters=[configured_params, {'yaml_filename': map_file}],
        arguments=[],
        remappings=[('/tf','tf'),
                    ('/tf_static','tf_static')],
    )
    
    lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        namespace=namespace,
        arguments=[],
        parameters=[configured_params],
    )
    
    amcl_cmd = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        namespace=namespace,
        parameters=[configured_params],
        arguments=[],
        remappings=[('/tf','tf'),
                    ('/tf_static','tf_static')],
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(amcl_cmd)
    ld.add_action(lifecycle_manager_cmd)

    return ld
