#!/usr/bin/env python3
import math
import statistics
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import Float64

CENTER_DEG = 90.0
SWEEP_OFFSETS_DEG = [-60, -45, -30, -15, 0, 15, 30, 45, 60]
HISTORY_LENGTH = 3


def history_median(history):
    finite_values = [v for v in history if not math.isnan(v)]
    if not finite_values:
        return float('nan')
    return statistics.median(finite_values)


class HeadUltrasonicScanNode(Node):
    def __init__(self):
        super().__init__('head_ultrasonic_scan_node')

        self.declare_parameter('settle_time', 0.5)
        self.settle_time = float(self.get_parameter('settle_time').value)

        self.pan_publisher = self.create_publisher(Float64, '/head_pan_angle', 10)
        self.scan_publisher = self.create_publisher(LaserScan, '/head_scan', 10)
        self.range_subscription = self.create_subscription(
            Range, '/ultrasonic_range', self.range_callback, 10
        )

        self.sample_buffer = []
        self.sweep_index = 0
        self.direction = 1
        self.ranges = [float('nan')] * len(SWEEP_OFFSETS_DEG)
        # Historial de las ultimas lecturas por angulo, entre barridos completos,
        # para filtrar el parpadeo entre pasadas (un mismo angulo puede dar
        # lecturas distintas de un barrido a otro por reflejos en el borde del cono).
        self.history = [deque(maxlen=HISTORY_LENGTH) for _ in SWEEP_OFFSETS_DEG]
        self.state = 'settling'
        self.state_start_time = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.step)
        self.command_angle(SWEEP_OFFSETS_DEG[self.sweep_index])

        self.get_logger().info(
            f'Head ultrasonic scan node started, sweeping {SWEEP_OFFSETS_DEG[0]} to {SWEEP_OFFSETS_DEG[-1]} deg'
        )

    def range_callback(self, msg: Range):
        if self.state == 'settling':
            self.sample_buffer.append(msg.range)

    def command_angle(self, offset_deg):
        msg = Float64()
        msg.data = CENTER_DEG + offset_deg
        self.pan_publisher.publish(msg)
        self.sample_buffer = []
        self.state = 'settling'
        self.state_start_time = self.get_clock().now()

    def step(self):
        elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
        if self.state == 'settling' and elapsed >= self.settle_time:
            raw_value = statistics.median(self.sample_buffer) if self.sample_buffer else float('nan')
            self.history[self.sweep_index].append(raw_value)
            self.ranges[self.sweep_index] = history_median(self.history[self.sweep_index])
            self.state = 'done'
            self.advance()

    def advance(self):
        self.sweep_index += self.direction
        if self.sweep_index >= len(SWEEP_OFFSETS_DEG):
            self.sweep_index = len(SWEEP_OFFSETS_DEG) - 1
            self.direction = -1
            self.publish_scan()
        elif self.sweep_index < 0:
            self.sweep_index = 0
            self.direction = 1
            self.publish_scan()
        self.command_angle(SWEEP_OFFSETS_DEG[self.sweep_index])

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'
        scan.angle_min = math.radians(SWEEP_OFFSETS_DEG[0])
        scan.angle_max = math.radians(SWEEP_OFFSETS_DEG[-1])
        scan.angle_increment = math.radians(SWEEP_OFFSETS_DEG[1] - SWEEP_OFFSETS_DEG[0])
        scan.range_min = 0.02
        scan.range_max = 3.0
        scan.ranges = list(self.ranges)
        self.scan_publisher.publish(scan)
        self.get_logger().info(f'Published head scan: {["%.2f" % r for r in self.ranges]}')


def main(args=None):
    rclpy.init(args=args)
    node = HeadUltrasonicScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
