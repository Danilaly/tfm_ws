#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

CENTER_DEG = 90.0


class GazeboHeadBridgeNode(Node):
    def __init__(self):
        super().__init__('gazebo_head_bridge_node')

        self.declare_parameter('publish_rate', 10.0)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.current_angle_deg = CENTER_DEG

        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, '/set_joint_trajectory', 10
        )
        self.subscription = self.create_subscription(
            Float64, '/head_pan_angle', self.pan_angle_callback, 10
        )

        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_joint_state)

        self.pan_angle_callback(Float64(data=CENTER_DEG))

        self.get_logger().info('Gazebo head bridge ready, centered at 90 degrees')

    def pan_angle_callback(self, msg: Float64):
        angle_deg = max(0.0, min(180.0, float(msg.data)))
        self.current_angle_deg = angle_deg

        trajectory = JointTrajectory()
        trajectory.header.frame_id = 'world'
        trajectory.joint_names = ['head_joint']
        point = JointTrajectoryPoint()
        point.positions = [math.radians(angle_deg - CENTER_DEG)]
        trajectory.points = [point]
        self.trajectory_publisher.publish(trajectory)

    def publish_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['head_joint']
        msg.position = [math.radians(self.current_angle_deg - CENTER_DEG)]
        self.joint_state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboHeadBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
