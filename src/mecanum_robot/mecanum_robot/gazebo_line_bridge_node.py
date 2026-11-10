#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray

# Umbral de intensidad (laser_retro) para distinguir la franja de linea
# (laser_retro=100 en el mundo) del suelo (laser_retro por defecto, ~0).
INTENSITY_THRESHOLD = 50.0


class GazeboLineBridgeNode(Node):
    def __init__(self):
        super().__init__('gazebo_line_bridge_node')

        self.readings = {'left': 0, 'center': 0, 'right': 0}

        self.publisher = self.create_publisher(Int32MultiArray, '/line_tracking_sensors', 10)

        for side in self.readings:
            self.create_subscription(
                LaserScan, f'/line_raw/{side}',
                self._make_callback(side), 10
            )

        self.timer = self.create_timer(0.05, self.publish_line_tracking)

        self.get_logger().info('Gazebo line bridge ready')

    def _make_callback(self, side):
        def callback(msg: LaserScan):
            intensity = msg.intensities[0] if msg.intensities else 0.0
            self.readings[side] = 1 if intensity >= INTENSITY_THRESHOLD else 0
        return callback

    def publish_line_tracking(self):
        msg = Int32MultiArray()
        msg.data = [self.readings['left'], self.readings['center'], self.readings['right']]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboLineBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
