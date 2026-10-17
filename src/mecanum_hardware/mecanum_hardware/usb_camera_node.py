#!/usr/bin/env python3
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class UsbCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')

        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('frame_id', 'camera_link')

        device_id = int(self.get_parameter('device_id').value)
        width = int(self.get_parameter('width').value)
        height = int(self.get_parameter('height').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        self.capture = cv2.VideoCapture(device_id)
        if not self.capture.isOpened():
            raise RuntimeError(f'Could not open USB camera at /dev/video{device_id}')
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_frame)

        self.get_logger().info(
            f'USB camera node started: device_id={device_id}, {width}x{height}, publish_rate={publish_rate} Hz'
        )

    def publish_frame(self):
        ret, frame = self.capture.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from USB camera')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        self.publisher.publish(msg)

    def destroy_node(self):
        try:
            self.capture.release()
        except Exception as e:
            self.get_logger().warn(f'Error releasing camera: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
