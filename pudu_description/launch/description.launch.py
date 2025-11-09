import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    robot_xacro_path = os.path.join(get_package_share_directory('pudu_description'), 'urdf/robots', 'pudu.urdf.xacro')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=namespace,
        parameters=[{'robot_description': ParameterValue(Command(['xacro ', robot_xacro_path, ' robot_namespace:=', namespace]), value_type=str)},
                    {'use_sim_time': use_sim_time}],
        remappings=[('/tf','tf'),
                    ('/tf_static','tf_static')]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(robot_state_publisher_cmd)

    return ld
