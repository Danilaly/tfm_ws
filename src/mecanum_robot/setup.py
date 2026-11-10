from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = 'mecanum_robot'


def package_data_files(directory, pattern):
    files = glob(join(directory, pattern))
    if not files:
        return []
    return [(join('share', package_name, directory), files)]


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    + package_data_files('launch', '*.py')
    + package_data_files('urdf', '*.xacro')
    + package_data_files('meshes', '*.stl')
    + package_data_files('rviz', '*.rviz')
    + package_data_files('config', '*.yaml')
    + package_data_files('worlds', '*.world'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danilaly',
    maintainer_email='danilaly@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_command = mecanum_robot.keyboard_command_node:main',
            'keyboard_command_node = mecanum_robot.keyboard_command_node:main',
            'mecanum_kinematics = mecanum_robot.mecanum_kinematics_node:main',
            'mecanum_kinematics_node = mecanum_robot.mecanum_kinematics_node:main',
            'line_tracking_marker_node = mecanum_robot.line_tracking_marker_node:main',
            'scan_interpreter_node = mecanum_robot.scan_interpreter_node:main',
            'navigation_decision_node = mecanum_robot.navigation_decision_node:main',
            'navigation_control_node = mecanum_robot.navigation_control_node:main',
            'visual_rotation_test_node = mecanum_robot.visual_rotation_test_node:main',
            'fault_monitor_node = mecanum_robot.fault_monitor_node:main',
            'validation_test_node = mecanum_robot.validation_test_node:main',
            'gazebo_head_bridge_node = mecanum_robot.gazebo_head_bridge_node:main',
            'gazebo_line_bridge_node = mecanum_robot.gazebo_line_bridge_node:main',
        ],
    },
)
