from setuptools import find_packages, setup

package_name = 'mecanum_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'geometry_msgs', 'std_msgs'],
    zip_safe=True,
    maintainer='austin',
    maintainer_email='austin@example.com',
    description='ROS 2 hardware interface for mecanum drive using the Freenove motor controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mecanum_motor_node = mecanum_hardware.mecanum_motor_node:main',
            'ultrasonic_sensor_node = mecanum_hardware.ultrasonic_sensor_node:main',
            'line_tracking_sensor_node = mecanum_hardware.line_tracking_sensor_node:main',
            'usb_camera_node = mecanum_hardware.usb_camera_node:main',
            'head_pan_node = mecanum_hardware.head_pan_node:main',
            'head_ultrasonic_scan_node = mecanum_hardware.head_ultrasonic_scan_node:main',
        ],
    },
)
