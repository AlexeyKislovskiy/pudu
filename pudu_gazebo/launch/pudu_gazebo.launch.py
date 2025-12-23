import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import ReplaceString

def generate_launch_description():
    
    world_path = LaunchConfiguration("world_path")
    verbose = LaunchConfiguration("verbose")

    namespace = LaunchConfiguration("namespace")
    robot_name = LaunchConfiguration("robot_name")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z_rot = LaunchConfiguration("z_rot")

    declare_world_path_cmd = DeclareLaunchArgument(
        name="world_path",
        default_value=os.path.join(get_package_share_directory("pudu_gazebo"), "worlds", "empty_world.world"),
        description="Gazebo world"
    )

    declare_verbose_cmd = DeclareLaunchArgument(
        name="verbose",
        default_value="-v4",
        description="Gazebo verbose"
    )


    declare_namespace_cmd = DeclareLaunchArgument(
        name="namespace",
        default_value="pudu",
        description="Robot namespace"
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        name="robot_name",
        default_value=namespace,
        description="Robot name"
    )

    declare_x_cmd = DeclareLaunchArgument(
        name="x",
        default_value="0.0",
        description="Robot X axis"
    )

    declare_y_cmd = DeclareLaunchArgument(
        name="y",
        default_value="0.0",
        description="Robot Y axis"
    )

    declare_z_rot_cmd = DeclareLaunchArgument(
        name="z_rot",
        default_value="0.0",
        description="Robot Z ROT axis"
    )

    gazebo_cmd = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("pudu_gazebo"), "launch", "gazebo.launch.py"),
        launch_arguments={
            'world': world_path,
            'verbose': verbose
        }.items()
    )

    spawn_cmd = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("pudu_gazebo"), "launch", "spawn.launch.py"),
        launch_arguments={
            'namespace': namespace,
            'robot_name': robot_name,
            'x': x,
            'y': y,
            'z_rot': z_rot
        }.items()
    )

    bridge_file_path = LaunchConfiguration('bridge_file_path')

    declare_bridge_file_path_cmd = DeclareLaunchArgument(
        name='bridge_file_path',
        default_value=os.path.join(get_package_share_directory('pudu_gazebo'), 'config', 'bridge.yaml'),
        description='Bridge configuration'
    )

    namespaced_bridge_file_path = ReplaceString(
        source_file=bridge_file_path,
        replacements={'<robot_namespace>': ('/', namespace)}
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
 
    gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        namespace=namespace,
        parameters=[{'config_file': namespaced_bridge_file_path, 'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_path_cmd)
    ld.add_action(declare_verbose_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_rot_cmd)

    ld.add_action(gazebo_cmd)
    ld.add_action(spawn_cmd)
    ld.add_action(declare_bridge_file_path_cmd)
    ld.add_action(gazebo_ros_bridge_cmd)

    return ld
