#!/usr/bin/env python3
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

# Agregar la carpeta del proyecto Freenove al sys.path
FREENOVE_SERVER_PATH = Path('/home/austin/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server')

if not FREENOVE_SERVER_PATH.exists():
    raise FileNotFoundError(f'Freenove Server directory not found at {FREENOVE_SERVER_PATH}')

if not FREENOVE_SERVER_PATH.joinpath('infrared.py').exists():
    raise FileNotFoundError(f'infrared.py not found at {FREENOVE_SERVER_PATH / "infrared.py"}')

if str(FREENOVE_SERVER_PATH) not in sys.path:
    sys.path.insert(0, str(FREENOVE_SERVER_PATH))

from infrared import Infrared


class LineTrackingSensorNode(Node):
    def __init__(self):
        super().__init__('line_tracking_sensor_node')

        # Declarar parámetro de frecuencia
        self.declare_parameter('publish_rate', 20.0)
        publish_rate = self.get_parameter('publish_rate').value

        # Crear el publicador
        self.publisher = self.create_publisher(Int32MultiArray, '/line_tracking_sensors', 10)

        # Inicializar los sensores de línea
        self.sensor = Infrared()

        # Crear Timer para publicar a la frecuencia especificada
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_line_tracking)

        self.get_logger().info(f'Line tracking sensor node started with publish_rate={publish_rate} Hz')

    def publish_line_tracking(self):
        """Leer los sensores IR y publicar su estado [izquierda, centro, derecha].

        El sensor crudo de Freenove devuelve 1 cuando NO hay línea (superficie
        reflectante) y 0 cuando detecta la línea negra. Se invierte aquí para
        publicar la convención estándar: 1 = línea detectada, 0 = sin línea.
        """
        self.get_logger().info('Line tracking timer callback entered')
        try:
            left = 1 - self.sensor.read_one_infrared(1)
            center = 1 - self.sensor.read_one_infrared(2)
            right = 1 - self.sensor.read_one_infrared(3)
            self.get_logger().info(f'Raw line tracking reading: left={left}, center={center}, right={right}')

            msg = Int32MultiArray()
            msg.data = [left, center, right]

            self.publisher.publish(msg)

            self.get_logger().info(f'Published line tracking sensors: {msg.data}')

        except Exception as e:
            self.get_logger().error(f'{e}')

    def destroy_node(self):
        """Cerrar los sensores al destruir el nodo."""
        try:
            self.sensor.close()
        except Exception as e:
            self.get_logger().warn(f'Error closing sensors: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineTrackingSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
