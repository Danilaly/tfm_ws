#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

CENTER_DEG = 90.0


class VisualRotationTestNode(Node):
    def __init__(self):
        super().__init__('visual_rotation_test_node')

        self.declare_parameter('angular_speed', 0.9)
        self.declare_parameter('turn_duration', 0.8)
        self.declare_parameter('camera_horizontal_fov_deg', 60.0)
        self.declare_parameter('settle_time', 0.5)

        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.turn_duration = float(self.get_parameter('turn_duration').value)
        self.fov_deg = float(self.get_parameter('camera_horizontal_fov_deg').value)
        self.settle_time = float(self.get_parameter('settle_time').value)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_a = None
        self.frame_b = None

        self.image_subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.pan_publisher = self.create_publisher(Float64, '/head_pan_angle', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.stage = 'centering_head'
        self.stage_start_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.run_experiment)

        self.get_logger().info('Visual rotation test node started: centrando cabeza')
        self.pan_publisher.publish(Float64(data=CENTER_DEG))

    def image_callback(self, msg: Image):
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def elapsed(self):
        return (self.get_clock().now() - self.stage_start_time).nanoseconds / 1e9

    def run_experiment(self):
        if self.stage == 'centering_head':
            if self.elapsed() >= 1.5:
                self.get_logger().info('Cabeza centrada. Capturando frame A')
                self.stage = 'capture_a'

        elif self.stage == 'capture_a':
            if self.latest_frame is not None:
                self.frame_a = self.latest_frame.copy()
                self.get_logger().info(f'Frame A capturado. Girando {self.turn_duration:.1f}s')
                twist = Twist()
                twist.angular.z = self.angular_speed
                self.cmd_vel_publisher.publish(twist)
                self.stage = 'turning'
                self.stage_start_time = self.get_clock().now()

        elif self.stage == 'turning':
            if self.elapsed() >= self.turn_duration:
                self.cmd_vel_publisher.publish(Twist())
                self.get_logger().info('Giro terminado. Esperando asentamiento')
                self.stage = 'settling'
                self.stage_start_time = self.get_clock().now()

        elif self.stage == 'settling':
            if self.elapsed() >= self.settle_time:
                self.frame_b = self.latest_frame.copy()
                self.get_logger().info('Frame B capturado. Calculando giro observado')
                self.compute_rotation()
                self.stage = 'done'

    def compute_rotation(self):
        cv2.imwrite('/tmp/visual_rotation_frame_a.jpg', self.frame_a)
        cv2.imwrite('/tmp/visual_rotation_frame_b.jpg', self.frame_b)
        cv2.imwrite('/tmp/visual_rotation_edges_a.jpg', cv2.Canny(cv2.cvtColor(self.frame_a, cv2.COLOR_BGR2GRAY), 50, 150))
        cv2.imwrite('/tmp/visual_rotation_edges_b.jpg', cv2.Canny(cv2.cvtColor(self.frame_b, cv2.COLOR_BGR2GRAY), 50, 150))

        profile_a = self.vertical_profile(self.frame_a)
        profile_b = self.vertical_profile(self.frame_b)

        correlation = np.correlate(profile_a, profile_b, mode='full')
        shift_px = int(np.argmax(correlation) - (len(profile_b) - 1))

        width = self.frame_a.shape[1]
        deg_per_px = self.fov_deg / width
        observed_deg = shift_px * deg_per_px
        commanded_deg = np.degrees(self.angular_speed * self.turn_duration)

        self.get_logger().info(
            f'RESULTADO: desplazamiento={shift_px}px | '
            f'giro ordenado={commanded_deg:.1f} deg | '
            f'giro observado (aprox, FOV={self.fov_deg:.0f} deg asumido)={observed_deg:.1f} deg'
        )

    @staticmethod
    def vertical_profile(frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        return edges.sum(axis=0).astype(np.float64)


def main(args=None):
    rclpy.init(args=args)
    node = VisualRotationTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
