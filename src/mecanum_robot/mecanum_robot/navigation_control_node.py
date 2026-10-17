#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


class NavigationControlNode(Node):
    def __init__(self):
        super().__init__('navigation_control_node')

        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.9)
        self.declare_parameter('move_duration', 0.8)
        self.declare_parameter('emergency_stop_distance', 0.15)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.move_duration = float(self.get_parameter('move_duration').value)
        self.emergency_stop_distance = float(self.get_parameter('emergency_stop_distance').value)

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.latest_distances = None
        self.stop_timer = None

        self.distances_subscription = self.create_subscription(
            Float64MultiArray, '/scan_interpretation/distances', self.distances_callback, 10
        )
        self.decision_subscription = self.create_subscription(
            String, '/navigation_decision', self.decision_callback, 10
        )

        self.publish_stop()
        self.get_logger().info(
            'Navigation control node ready (ciclo discreto: decidir -> mover -> parar)'
        )

    def distances_callback(self, msg: Float64MultiArray):
        self.latest_distances = list(msg.data)

    def emergency_stop_triggered(self):
        if not self.latest_distances:
            return False
        for distance in self.latest_distances:
            if distance == distance and distance < self.emergency_stop_distance:  # descarta NaN
                return True
        return False

    def decision_callback(self, msg: String):
        if self.stop_timer is not None:
            # Todavia ejecutando el movimiento anterior: ignorar decisiones nuevas
            # hasta terminar el ciclo actual (parar -> escanear -> decidir -> mover -> parar).
            return

        decision = msg.data

        # El override de emergencia solo debe bloquear avanzar hacia algo cercano.
        # RETROCEDER es precisamente la maniobra para alejarse cuando todo esta
        # bloqueado, asi que no debe cancelarse a si misma con este chequeo.
        if decision == 'AVANZAR' and self.emergency_stop_triggered():
            self.get_logger().warn('Parada de emergencia: obstaculo demasiado cerca')
            self.publish_stop()
            return

        twist = Twist()
        if decision == 'AVANZAR':
            twist.linear.x = self.linear_speed
        elif decision == 'RETROCEDER':
            twist.linear.x = -self.linear_speed
        elif decision == 'GIRAR_IZQUIERDA':
            twist.angular.z = self.angular_speed
        elif decision == 'GIRAR_DERECHA':
            twist.angular.z = -self.angular_speed
        else:
            self.publish_stop()
            return

        self.get_logger().info(f'Ejecutando: {decision} durante {self.move_duration:.1f}s')
        self.cmd_vel_publisher.publish(twist)
        self.stop_timer = self.create_timer(self.move_duration, self.finish_move)

    def finish_move(self):
        self.publish_stop()
        self.stop_timer.cancel()
        self.stop_timer = None

    def publish_stop(self):
        self.cmd_vel_publisher.publish(Twist())

    def destroy_node(self):
        self.publish_stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NavigationControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
