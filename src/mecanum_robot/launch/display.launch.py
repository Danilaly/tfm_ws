import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def filtered_library_path():
    library_path = os.environ.get('LD_LIBRARY_PATH', '')
    if not library_path:
        return ''

    filtered_entries = [
        entry for entry in library_path.split(':')
        if entry and '/snap/' not in entry
    ]
    return ':'.join(filtered_entries)


def generate_launch_description():
    package_share = get_package_share_directory('mecanum_robot')
    xacro_file = os.path.join(package_share, 'urdf', 'mecanum.urdf.xacro')
    rviz_file = os.path.join(package_share, 'rviz', 'mecanum.rviz')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            xacro_file
        ]),
        value_type=str
    )
    gui_env = {'LD_LIBRARY_PATH': filtered_library_path()}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='mecanum_robot',
            executable='mecanum_kinematics_node',
            name='mecanum_kinematics_node',
            output='screen',
        ),
        Node(
            package='mecanum_robot',
            executable='line_tracking_marker_node',
            name='line_tracking_marker_node',
            output='screen',
        ),
        Node(
            package='mecanum_robot',
            executable='scan_interpreter_node',
            name='scan_interpreter_node',
            output='screen',
        ),
        Node(
            package='mecanum_robot',
            executable='navigation_decision_node',
            name='navigation_decision_node',
            output='screen',
        ),
        Node(
            package='mecanum_robot',
            executable='fault_monitor_node',
            name='fault_monitor_node',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            additional_env=gui_env,
            output='screen',
        ),
    ])
