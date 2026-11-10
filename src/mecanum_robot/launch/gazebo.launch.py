from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('mecanum_robot'),
        'urdf',
        'mecanum.urdf.xacro'
    )

    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # Gazebo convierte package:// a model:// internamente y busca las mallas
    # en GAZEBO_MODEL_PATH, no en el indice de paquetes de ROS. Sin esto,
    # el robot se spawnea pero queda invisible (sin error, solo advertencias).
    package_share_parent = os.path.dirname(get_package_share_directory('mecanum_robot'))

    world_file = os.path.join(
        get_package_share_directory('mecanum_robot'),
        'worlds',
        'mecanum_world.world'
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':', package_share_parent]
        ),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'mecanum_robot'
            ],
            output='screen'
        ),

        Node(
            package='mecanum_robot',
            executable='gazebo_head_bridge_node',
            name='gazebo_head_bridge_node',
            output='screen'
        ),

        Node(
            package='mecanum_robot',
            executable='gazebo_line_bridge_node',
            name='gazebo_line_bridge_node',
            output='screen'
        ),

        Node(
            package='mecanum_hardware',
            executable='head_ultrasonic_scan_node',
            name='head_ultrasonic_scan_node',
            output='screen'
        )
    ])
