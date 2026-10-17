#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from visualization_msgs.msg import Marker


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


class NavigationDecisionNode(Node):
    def __init__(self):
        super().__init__('navigation_decision_node')

        self.declare_parameter('blocked_threshold', 0.3)
        self.declare_parameter('free_threshold', 0.6)
        self.blocked_threshold = float(self.get_parameter('blocked_threshold').value)
        self.free_threshold = float(self.get_parameter('free_threshold').value)

        self.decision_publisher = self.create_publisher(String, '/navigation_decision', 10)
        self.marker_publisher = self.create_publisher(Marker, '/navigation_decision/marker', 10)

        self.subscription = self.create_subscription(
            Float64MultiArray, '/scan_interpretation/distances', self.distances_callback, 10
        )

        self.get_logger().info('Navigation decision node ready (no publica a /cmd_vel todavia)')

    def distances_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 3:
            return
        left, center, right = msg.data[0], msg.data[1], msg.data[2]

        left_status = classify(left, self.blocked_threshold, self.free_threshold)
        center_status = classify(center, self.blocked_threshold, self.free_threshold)
        right_status = classify(right, self.blocked_threshold, self.free_threshold)

        if 'DESCONOCIDO' in (left_status, center_status, right_status):
            # Sin datos suficientes todavia (p.ej. al arrancar, antes del primer barrido
            # completo): no hay que arriesgar avanzando o girando a ciegas.
            decision = 'PARAR'
        elif left_status == 'BLOQUEADO' and center_status == 'BLOQUEADO' and right_status == 'BLOQUEADO':
            # Encajonado por todos lados: parar no ayuda a salir de ahi, hay que retroceder.
            decision = 'RETROCEDER'
        elif center_status == 'LIBRE':
            decision = 'AVANZAR'
        elif right > left:
            decision = 'GIRAR_DERECHA'
        else:
            decision = 'GIRAR_IZQUIERDA'

        decision_msg = String()
        decision_msg.data = decision
        self.decision_publisher.publish(decision_msg)
        self.get_logger().info(
            f'Decision: {decision} '
            f'(izq={left:.2f}/{left_status}, cen={center:.2f}/{center_status}, der={right:.2f}/{right_status})'
        )

        self.publish_marker(decision)

    def publish_marker(self, decision):
        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'navigation_decision'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.z = 0.15
        marker.scale.x = 0.25
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        yaw = 0.0
        if decision == 'GIRAR_IZQUIERDA':
            yaw = math.radians(45)
        elif decision == 'GIRAR_DERECHA':
            yaw = math.radians(-45)
        elif decision == 'RETROCEDER':
            yaw = math.radians(180)

        if decision == 'PARAR':
            marker.scale.x = 0.01
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif decision == 'RETROCEDER':
            marker.color.r = 0.6
            marker.color.g = 0.0
            marker.color.b = 0.8
        elif decision == 'AVANZAR':
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.r = 1.0
            marker.color.g = 0.65
            marker.color.b = 0.0
        marker.color.a = 1.0

        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)

        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
