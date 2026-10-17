from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('mecanum_robot'),
        'urdf',
        'mecanum.urdf.xacro'
    )

    robot_description = Command(['xacro ', urdf_file])

    # Gazebo convierte package:// a model:// internamente y busca las mallas
    # en GAZEBO_MODEL_PATH, no en el indice de paquetes de ROS. Sin esto,
    # el robot se spawnea pero queda invisible (sin error, solo advertencias).
    package_share_parent = os.path.dirname(get_package_share_directory('mecanum_robot'))

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[os.environ.get('GAZEBO_MODEL_PATH', ''), ':', package_share_parent]
        ),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
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
        )
    ])
