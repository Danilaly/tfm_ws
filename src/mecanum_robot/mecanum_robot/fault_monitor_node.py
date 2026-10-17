#!/usr/bin/env python3
import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker


class FaultMonitorNode(Node):
    def __init__(self):
        super().__init__('fault_monitor_node')

        self.declare_parameter('timeout_seconds', 2.0)
        self.declare_parameter('frozen_count_threshold', 30)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('range_min', 0.02)
        self.declare_parameter('range_max', 3.0)

        self.timeout_seconds = float(self.get_parameter('timeout_seconds').value)
        self.frozen_count_threshold = int(self.get_parameter('frozen_count_threshold').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.last_ultrasonic_time = None
        self.last_ir_time = None
        self.last_head_time = None
        self.last_ultrasonic_value = None
        self.ultrasonic_frozen_count = 0
        self.last_ultrasonic_invalid = False
        self.last_ir_invalid = False

        self.create_subscription(Range, '/ultrasonic_range', self.ultrasonic_callback, 10)
        self.create_subscription(Int32MultiArray, '/line_tracking_sensors', self.ir_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        self.diag_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.marker_publisher = self.create_publisher(Marker, '/fault_status/marker', 10)

        self.timer = self.create_timer(1.0 / publish_rate, self.check_and_publish)

        self.get_logger().info('Fault monitor node ready')

    def ultrasonic_callback(self, msg: Range):
        self.last_ultrasonic_time = self.get_clock().now()
        value = msg.range

        # inf es un valor LEGITIMO del sensor (nada detectado dentro de rango),
        # no es un fallo. Invalido = NaN, negativo, o finito fuera de rango fisico.
        self.last_ultrasonic_invalid = math.isnan(value) or value < 0 or (
            math.isfinite(value) and not (self.range_min <= value <= self.range_max)
        )

        if self.last_ultrasonic_value is not None and value == self.last_ultrasonic_value:
            self.ultrasonic_frozen_count += 1
        else:
            self.ultrasonic_frozen_count = 0
        self.last_ultrasonic_value = value

    def ir_callback(self, msg: Int32MultiArray):
        self.last_ir_time = self.get_clock().now()
        data = list(msg.data)
        self.last_ir_invalid = len(data) != 3 or any(v not in (0, 1) for v in data)

    def joint_callback(self, msg: JointState):
        if 'head_joint' in msg.name:
            self.last_head_time = self.get_clock().now()

    def seconds_since(self, stamp):
        if stamp is None:
            return None
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def duplicate_node_names(self, topic):
        try:
            infos = self.get_publishers_info_by_topic(topic)
        except Exception:
            return []
        names = [info.node_name for info in infos]
        return sorted({n for n in names if names.count(n) > 1})

    def check_and_publish(self):
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()

        ultrasonic_age = self.seconds_since(self.last_ultrasonic_time)
        ir_age = self.seconds_since(self.last_ir_time)
        head_age = self.seconds_since(self.last_head_time)

        # Ultrasonido
        status = DiagnosticStatus(name='gemelo_digital: ultrasonido', hardware_id='ultrasonic_sensor_node')
        if ultrasonic_age is None or ultrasonic_age > self.timeout_seconds:
            status.level = DiagnosticStatus.ERROR
            status.message = 'FAULT: ULTRASONIC_TIMEOUT'
        elif self.ultrasonic_frozen_count >= self.frozen_count_threshold:
            status.level = DiagnosticStatus.WARN
            status.message = f'WARNING: ULTRASONIC_FROZEN ({self.ultrasonic_frozen_count} lecturas identicas)'
        elif self.last_ultrasonic_invalid:
            status.level = DiagnosticStatus.WARN
            status.message = 'WARNING: ULTRASONIC_INVALID'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'OK'
        array.status.append(status)

        # Infrarrojo
        status = DiagnosticStatus(name='gemelo_digital: infrarrojo', hardware_id='line_tracking_sensor_node')
        if ir_age is None or ir_age > self.timeout_seconds:
            status.level = DiagnosticStatus.ERROR
            status.message = 'FAULT: INFRARED_TIMEOUT'
        elif self.last_ir_invalid:
            status.level = DiagnosticStatus.WARN
            status.message = 'WARNING: INFRARED_INVALID'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'OK'
        array.status.append(status)

        # Cabeza (head_pan_node)
        status = DiagnosticStatus(name='gemelo_digital: cabeza (head_pan)', hardware_id='head_pan_node')
        if head_age is None or head_age > self.timeout_seconds:
            status.level = DiagnosticStatus.ERROR
            status.message = 'FAULT: HEAD_NODE_TIMEOUT'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'OK'
        array.status.append(status)

        # Comunicacion total con la Raspberry Pi (los 3 sensores caidos a la vez)
        status = DiagnosticStatus(name='gemelo_digital: comunicacion Raspberry Pi', hardware_id='raspberry_pi')
        ages = (ultrasonic_age, ir_age, head_age)
        if all(age is None or age > self.timeout_seconds for age in ages):
            status.level = DiagnosticStatus.ERROR
            status.message = 'FAULT: ROBOT_COMMUNICATION_LOST'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'OK'
        array.status.append(status)

        # Nodos duplicados publicando en el mismo topic (bug real detectado hoy)
        status = DiagnosticStatus(name='gemelo_digital: nodos duplicados', hardware_id='ros_graph')
        duplicates = set()
        for topic in ('/joint_states', '/cmd_vel', '/tf'):
            duplicates.update(self.duplicate_node_names(topic))
        if duplicates:
            status.level = DiagnosticStatus.ERROR
            status.message = f'FAULT: DUPLICATE_NODE ({", ".join(sorted(duplicates))})'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'OK'
        array.status.append(status)

        self.diag_publisher.publish(array)
        self.publish_marker(array)

        problems = [s.message for s in array.status if s.level != DiagnosticStatus.OK]
        if problems:
            self.get_logger().warn('Estado: ' + ' | '.join(problems))
        else:
            self.get_logger().info('Estado: OK', throttle_duration_sec=5.0)

    def publish_marker(self, array: DiagnosticArray):
        problems = [s.message for s in array.status if s.level != DiagnosticStatus.OK]
        text = 'ESTADO:OK' if not problems else '\n'.join(p.replace(' ', '-') for p in problems)
        worst = max((s.level for s in array.status), default=DiagnosticStatus.OK)

        marker = Marker()
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'fault_monitor'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 0.55
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.06
        marker.color.a = 1.0
        if worst == DiagnosticStatus.OK:
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
        elif worst == DiagnosticStatus.WARN:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.65, 0.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
        marker.text = text
        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FaultMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
