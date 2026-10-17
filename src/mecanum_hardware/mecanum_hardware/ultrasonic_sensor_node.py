#!/usr/bin/env python3
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

# Agregar la carpeta del proyecto Freenove al sys.path
FREENOVE_SERVER_PATH = Path('/home/austin/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server')

if not FREENOVE_SERVER_PATH.exists():
    raise FileNotFoundError(f'Freenove Server directory not found at {FREENOVE_SERVER_PATH}')

if not FREENOVE_SERVER_PATH.joinpath('ultrasonic.py').exists():
    raise FileNotFoundError(f'ultrasonic.py not found at {FREENOVE_SERVER_PATH / "ultrasonic.py"}')

if str(FREENOVE_SERVER_PATH) not in sys.path:
    sys.path.insert(0, str(FREENOVE_SERVER_PATH))

from ultrasonic import Ultrasonic


class UltrasonicSensorNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor_node')

        # Declarar parámetro de frecuencia
        self.declare_parameter('publish_rate', 10.0)
        publish_rate = self.get_parameter('publish_rate').value

        # Crear el publicador
        self.publisher = self.create_publisher(Range, '/ultrasonic_range', 10)

        # Inicializar el sensor ultrasónico
        self.sensor = Ultrasonic()

        # Crear Timer para publicar a la frecuencia especificada
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.publish_range)

        self.get_logger().info(f'Ultrasonic sensor node started with publish_rate={publish_rate} Hz')

    def publish_range(self):
        """Leer distancia del sensor y publicarla."""
        self.get_logger().info('Ultrasonic timer callback entered')
        try:
            # Leer distancia en centímetros
            raw_distance = self.sensor.get_distance()
            self.get_logger().info(f'Raw ultrasonic reading: {raw_distance}')

            # Crear mensaje Range
            msg = Range()
            msg.header.frame_id = 'ultrasonic_link'
            msg.header.stamp.sec = 0
            msg.header.stamp.nanosec = 0
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26
            msg.min_range = 0.02
            msg.max_range = 3.0

            # Convertir a metros y validar
            if raw_distance is not None and raw_distance > 0:
                msg.range = float(raw_distance) / 100.0
            else:
                msg.range = float('inf')

            # Publicar
            self.publisher.publish(msg)

            self.get_logger().info(f'Published ultrasonic range: {msg.range} m')

        except Exception as e:
            self.get_logger().error(f'{e}')

    def destroy_node(self):
        """Cerrar el sensor al destruir el nodo."""
        try:
            self.sensor.close()
        except Exception as e:
            self.get_logger().warn(f'Error closing sensor: {e}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
