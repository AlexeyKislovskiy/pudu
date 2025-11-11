import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z_rot = LaunchConfiguration('z_rot')

    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='pudu',
        description='Top-level namespace'
    )

    declare_x_cmd = DeclareLaunchArgument(
        name='x',
        default_value='0.0',
        description='Robot position (x)'
    )

    declare_y_cmd = DeclareLaunchArgument(
        name='y',
        default_value='0.0',
        description='Robot position (y)'
    )

    declare_z_rot_cmd = DeclareLaunchArgument(
        name='z_rot',
        default_value='0.0',
        description='Robot orientation (yaw)'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        name='robot_name',
        default_value=namespace,
        description='Robot name in Gazebo'
    )

    robot_description_cmd = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('pudu_description'), 'launch', 'description.launch.py'),
        launch_arguments={'namespace': namespace,
                          'use_sim_time': 'true'}.items()
    )

    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        name='urdf_spawner',
        output='screen',
        arguments=['-topic', [namespace, '/robot_description'],
                   '-name', robot_name,
                   '-x', x,
                   '-y', y,
                   '-Y', z_rot]
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_rot_cmd)

    ld.add_action(robot_description_cmd)
    ld.add_action(spawn_entity_cmd)

    return ld