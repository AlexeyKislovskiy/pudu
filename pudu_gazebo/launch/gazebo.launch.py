import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    world_path = LaunchConfiguration('world_path')

    declare_world_path_cmd = DeclareLaunchArgument(
        name='world_path',
        default_value=os.path.join(get_package_share_directory('pudu_gazebo'), 'worlds', 'empty_world.world'),
        description='Specify world file'
    )

    gzserver_cmd = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
        launch_arguments={'gz_args': ['-r -s -v4 ', world_path],
                          'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_path_cmd)

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    return ld