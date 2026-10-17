#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker

# Los 9 puntos del barrido van de -60 a 60 grados. Se agrupan en 3 sectores
# de 3 puntos cada uno. Angulos positivos = izquierda del robot (REP103).
RIGHT_SLICE = slice(0, 3)
CENTER_SLICE = slice(3, 6)
LEFT_SLICE = slice(6, 9)


def classify(distance, blocked_threshold, free_threshold):
    # NaN = todavia no hay lectura para ese sector (no confundir con "libre")
    if distance is None or math.isnan(distance):
        return 'DESCONOCIDO'
    # inf = el ultrasonido no detecto nada dentro de su rango maximo (si es libre)
    if math.isinf(distance):
        return 'LIBRE'
    if distance < blocked_threshold:
        return 'BLOQUEADO'
    if distance < free_threshold:
        return 'PRECAUCION'
    return 'LIBRE'


def sector_min(ranges, sector_slice):
    finite_values = [r for r in ranges[sector_slice] if not math.isnan(r) and not math.isinf(r)]
    if finite_values:
        return min(finite_values)
    if any(math.isinf(r) for r in ranges[sector_slice]):
        return float('inf')
    return float('nan')


class ScanInterpreterNode(Node):
    def __init__(self):
        super().__init__('scan_interpreter_node')

        self.declare_parameter('blocked_threshold', 0.3)
        self.declare_parameter('free_threshold', 0.6)
        self.blocked_threshold = float(self.get_parameter('blocked_threshold').value)
        self.free_threshold = float(self.get_parameter('free_threshold').value)

        self.distances_publisher = self.create_publisher(
            Float64MultiArray, '/scan_interpretation/distances', 10
        )
        self.status_publisher = self.create_publisher(String, '/scan_interpretation/status', 10)
        self.marker_publisher = self.create_publisher(Marker, '/scan_interpretation/marker', 10)

        self.subscription = self.create_subscription(LaserScan, '/head_scan', self.scan_callback, 10)

        self.get_logger().info('Scan interpreter node ready')

    def scan_callback(self, msg: LaserScan):
        ranges = list(msg.ranges)
        if len(ranges) < 9:
            self.get_logger().warn(f'Expected 9 samples in /head_scan, got {len(ranges)}')
            return

        left = sector_min(ranges, LEFT_SLICE)
        center = sector_min(ranges, CENTER_SLICE)
        right = sector_min(ranges, RIGHT_SLICE)

        left_status = classify(left, self.blocked_threshold, self.free_threshold)
        center_status = classify(center, self.blocked_threshold, self.free_threshold)
        right_status = classify(right, self.blocked_threshold, self.free_threshold)

        # Publicar distancias [izquierda, centro, derecha] para el futuro nodo de decision
        distances_msg = Float64MultiArray()
        distances_msg.data = [left, center, right]
        self.distances_publisher.publish(distances_msg)

        # Publicar resumen legible para logging/depuracion
        status_text = (
            f'IZQUIERDA: {left:.2f} m {left_status} | '
            f'CENTRO: {center:.2f} m {center_status} | '
            f'DERECHA: {right:.2f} m {right_status}'
        )
        status_msg = String()
        status_msg.data = status_text
        self.status_publisher.publish(status_msg)
        self.get_logger().info(status_text)

        self.publish_marker(left, left_status, center, center_status, right, right_status)

    def publish_marker(self, left, left_status, center, center_status, right, right_status):
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'scan_interpretation'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.4
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.06
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = (
            f'IZQ:{left:.2f}-{left_status}\n'
            f'CEN:{center:.2f}-{center_status}\n'
            f'DER:{right:.2f}-{right_status}'
        )
        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ScanInterpreterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
