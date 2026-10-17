#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class MecanumKinematicsNode(Node):
    def __init__(self):
        super().__init__('mecanum_kinematics_node')

        self.declare_parameter('wheel_radius', 0.026)
        self.declare_parameter('lx', 0.040)
        self.declare_parameter('ly', 0.064517)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.lx = float(self.get_parameter('lx').value)
        self.ly = float(self.get_parameter('ly').value)
        self.wheel_base_factor = self.lx + self.ly

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.wheel_angles = {
            'front_left_wheel_joint': 0.0,
            'front_right_wheel_joint': 0.0,
            'rear_left_wheel_joint': 0.0,
            'rear_right_wheel_joint': 0.0,
        }
        self.wheel_velocities = {
            'front_left_wheel_joint': 0.0,
            'front_right_wheel_joint': 0.0,
            'rear_left_wheel_joint': 0.0,
            'rear_right_wheel_joint': 0.0,
        }

        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.last_cmd_time = self.get_clock().now()

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = self.get_clock().now()
        self.vx = float(msg.linear.x)
        self.vy = float(msg.linear.y)
        self.wz = float(msg.angular.z)

        self.wheel_velocities['front_left_wheel_joint'] = (
            self.vx - self.vy - self.wheel_base_factor * self.wz
        ) / self.wheel_radius
        self.wheel_velocities['front_right_wheel_joint'] = (
            self.vx + self.vy + self.wheel_base_factor * self.wz
        ) / self.wheel_radius
        self.wheel_velocities['rear_left_wheel_joint'] = (
            self.vx + self.vy - self.wheel_base_factor * self.wz
        ) / self.wheel_radius
        self.wheel_velocities['rear_right_wheel_joint'] = (
            self.vx - self.vy + self.wheel_base_factor * self.wz
        ) / self.wheel_radius

    def timer_callback(self):
        dt = 1.0 / 50.0

        idle_seconds = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if idle_seconds > self.cmd_vel_timeout:
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
            for joint_name in self.wheel_velocities:
                self.wheel_velocities[joint_name] = 0.0

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (self.vx * cos_yaw - self.vy * sin_yaw) * dt
        self.y += (self.vx * sin_yaw + self.vy * cos_yaw) * dt
        self.yaw += self.wz * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        for joint_name, wheel_velocity in self.wheel_velocities.items():
            self.wheel_angles[joint_name] += wheel_velocity * dt

        self.publish_joint_states()
        self.publish_transform()

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]
        msg.position = [
            self.wheel_angles['front_left_wheel_joint'],
            self.wheel_angles['front_right_wheel_joint'],
            self.wheel_angles['rear_left_wheel_joint'],
            self.wheel_angles['rear_right_wheel_joint'],
        ]
        msg.velocity = [
            self.wheel_velocities['front_left_wheel_joint'],
            self.wheel_velocities['front_right_wheel_joint'],
            self.wheel_velocities['rear_left_wheel_joint'],
            self.wheel_velocities['rear_right_wheel_joint'],
        ]
        self.joint_state_publisher.publish(msg)

    def publish_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        half_yaw = self.yaw * 0.5
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(half_yaw)
        transform.transform.rotation.w = math.cos(half_yaw)

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
