#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker


class LineTrackingMarkerNode(Node):
    def __init__(self):
        super().__init__('line_tracking_marker_node')

        self.declare_parameter('frame_id', 'base_footprint')
        self.declare_parameter('height', 0.25)
        self.frame_id = self.get_parameter('frame_id').value
        self.height = float(self.get_parameter('height').value)

        self.publisher = self.create_publisher(Marker, '/line_tracking_marker', 10)
        self.subscription = self.create_subscription(
            Int32MultiArray, '/line_tracking_sensors', self.line_tracking_callback, 10
        )

        self.get_logger().info('Line tracking marker node ready')

    def line_tracking_callback(self, msg: Int32MultiArray):
        left, center, right = (list(msg.data) + [0, 0, 0])[:3]

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'line_tracking'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = self.height
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.08
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = f'{left}-{center}-{right}'

        self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = LineTrackingMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
