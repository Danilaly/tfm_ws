#!/usr/bin/env python3

import csv
import math
import signal
import statistics
import threading
import time
from datetime import datetime

import rclpy
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Range

# Referencia global al nodo activo, para que el manejador de SIGINT (Ctrl+C)
# pueda mandar la parada de emergencia de forma sincrona, antes de que rclpy
# invalide el contexto del publisher. Confirmado en pruebas reales: sin esto,
# node.stop() en el flujo normal de cierre puede fallar silenciosamente
# ("publisher's context is invalid") y el robot se queda ejecutando la
# ultima velocidad distinta de cero.
_active_node = {'node': None}


def _emergency_stop_handler(signum, frame):
    node = _active_node.get('node')
    if node is not None:
        node.stop(retries=8, period=0.05)
    raise KeyboardInterrupt


class ValidationTestNode(Node):
    def __init__(self):
        super().__init__('validation_test_node')

        self.declare_parameter('mode', 'real')
        self.declare_parameter('velocities', [0.10, 0.15, 0.20])
        self.declare_parameter('durations', [1.0, 2.0, 2.5])
        self.declare_parameter('repetitions', 5)
        self.declare_parameter('safety_margin', 0.12)
        self.declare_parameter('settle_time', 0.8)
        self.declare_parameter('sample_count', 5)
        self.declare_parameter('sample_period', 0.1)
        self.declare_parameter('cmd_rate', 10.0)
        self.declare_parameter('output_csv', '')

        self.mode = self.get_parameter('mode').value
        if self.mode not in ('real', 'sim'):
            raise ValueError(f"mode desconocido: {self.mode!r} (usa 'real' o 'sim')")

        self.velocities = list(self.get_parameter('velocities').value)
        self.durations = list(self.get_parameter('durations').value)
        self.repetitions = int(self.get_parameter('repetitions').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.sample_count = int(self.get_parameter('sample_count').value)
        self.sample_period = float(self.get_parameter('sample_period').value)
        self.cmd_rate = float(self.get_parameter('cmd_rate').value)

        output_csv = self.get_parameter('output_csv').value
        self.output_csv = output_csv if output_csv else f'validation_{self.mode}_results.csv'

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.latest_range = None
        self.latest_odom_xy = None

        if self.mode == 'real':
            self.create_subscription(Range, '/ultrasonic_range', self._range_callback, 10)
        else:
            self.create_subscription(Odometry, '/gazebo_odom', self._odom_callback, 10)
            self.set_entity_state_client = self.create_client(SetEntityState, '/set_entity_state')

    def _range_callback(self, msg):
        self.latest_range = msg.range

    def _odom_callback(self, msg):
        self.latest_odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def sample_distance_real(self):
        samples = []
        for _ in range(self.sample_count):
            if self.latest_range is not None and math.isfinite(self.latest_range):
                samples.append(self.latest_range)
            time.sleep(self.sample_period)
        return statistics.median(samples) if samples else None

    def sample_position_sim(self):
        time.sleep(self.sample_period)
        return self.latest_odom_xy

    def publish_cmd(self, vx, duration):
        twist = Twist()
        twist.linear.x = vx
        steps = max(1, int(duration * self.cmd_rate))
        period = 1.0 / self.cmd_rate
        for _ in range(steps):
            self.cmd_vel_publisher.publish(twist)
            time.sleep(period)
        # Parada explicita: en Gazebo el plugin planar_move no tiene deadman
        # propio, se queda con la ultima velocidad para siempre si no se le
        # manda cero. En el robot real, mandar la parada varias veces protege
        # contra perdida de paquetes por WiFi (un solo mensaje puede no llegar).
        self.stop()

    def stop(self, retries=5, period=0.05):
        zero = Twist()
        for _ in range(retries):
            try:
                self.cmd_vel_publisher.publish(zero)
            except Exception:
                pass
            time.sleep(period)

    def reset_sim_pose(self, timeout=2.0):
        if not self.set_entity_state_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn('Servicio /set_entity_state no disponible, no se reseteo la pose.')
            return
        request = SetEntityState.Request()
        request.state.name = 'mecanum_robot'
        request.state.pose.position.x = 0.0
        # y = 0.3 para que las pruebas de desplazamiento recto no crucen la
        # franja de linea situada en y=0 (pensada para los sensores IR, no
        # para que el robot pase fisicamente por encima durante estas
        # pruebas de cinematica pura).
        request.state.pose.position.y = 0.3
        # z = 0.002 coincide con la altura de reposo real del modelo (medida
        # con "gz model -p"). Con z mayor el robot cae y rebota durante
        # ~1.4 s tras el reset, y el solver de contactos de ODE bloquea el
        # movimiento lateral mientras se asienta, invalidando la medida.
        request.state.pose.position.z = 0.002
        request.state.pose.orientation.w = 1.0
        future = self.set_entity_state_client.call_async(request)
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.02)
        time.sleep(0.3)


def run_trial(node, writer, condition_index, vx, duration, repetition):
    if node.mode == 'real':
        expected = vx * duration
        d_before = node.sample_distance_real()
        while d_before is None or (d_before - expected * 1.3) < node.safety_margin:
            node.stop()
            if d_before is None:
                print('Lectura del ultrasonico invalida (fuera de rango). '
                      'Revisa que el robot este frente a la pared, dentro del cono del sensor.')
            else:
                required = expected * 1.3 + node.safety_margin
                print(f'Quedan {d_before:.2f} m hasta la pared, hacen falta al menos '
                      f'{required:.2f} m para este movimiento con margen de seguridad.')
            input('Reposiciona el robot mas lejos de la pared y presiona Enter para continuar...')
            d_before = node.sample_distance_real()
    else:
        # Se resetea la pose antes de cada repeticion para que todas partan
        # del mismo punto; sin esto, el desplazamiento se acumula entre
        # repeticiones hasta que el robot choca con una pared del mundo.
        node.reset_sim_pose()
        d_before = node.sample_position_sim()
        while d_before is None:
            time.sleep(0.2)
            d_before = node.sample_position_sim()

    node.publish_cmd(vx, duration)
    time.sleep(node.settle_time)

    if node.mode == 'real':
        d_after = node.sample_distance_real()
        valid = d_after is not None
        displacement = (d_before - d_after) if valid else None
    else:
        d_after = node.sample_position_sim()
        valid = d_after is not None
        if valid:
            dx = d_after[0] - d_before[0]
            dy = d_after[1] - d_before[1]
            displacement = math.hypot(dx, dy)
        else:
            displacement = None

    writer.writerow({
        'mode': node.mode,
        'condition_index': condition_index,
        'velocity_mps': vx,
        'duration_s': duration,
        'repetition': repetition,
        'd_before': d_before,
        'd_after': d_after,
        'displacement_m': displacement,
        'valid': valid,
        'timestamp': datetime.now().isoformat(timespec='seconds'),
    })

    status = f'{displacement:.3f} m' if valid else 'INVALIDO'
    print(f'[cond {condition_index} v={vx} T={duration}] rep {repetition + 1}/{node.repetitions}: {status}')
    return displacement if valid else None


def main(args=None):
    rclpy.init(args=args)
    node = ValidationTestNode()
    _active_node['node'] = node
    signal.signal(signal.SIGINT, _emergency_stop_handler)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    time.sleep(1.0)

    conditions = list(zip(node.velocities, node.durations))
    results_by_condition = {i: [] for i in range(len(conditions))}

    fieldnames = ['mode', 'condition_index', 'velocity_mps', 'duration_s', 'repetition',
                  'd_before', 'd_after', 'displacement_m', 'valid', 'timestamp']

    with open(node.output_csv, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        try:
            for condition_index, (vx, duration) in enumerate(conditions):
                print(f'\n--- Condicion {condition_index}: v={vx} m/s, T={duration} s ---')
                for repetition in range(node.repetitions):
                    displacement = run_trial(node, writer, condition_index, vx, duration, repetition)
                    f.flush()
                    if displacement is not None:
                        results_by_condition[condition_index].append(displacement)
                if node.mode == 'real' and condition_index < len(conditions) - 1:
                    input('\nCondicion terminada. Reposiciona el robot frente a la pared si hace falta '
                          'y presiona Enter para la siguiente condicion...')
        except KeyboardInterrupt:
            print('\nInterrumpido por el usuario.')

    node.stop()

    print(f'\nResultados guardados en {node.output_csv}')
    print('\nResumen:')
    for condition_index, (vx, duration) in enumerate(conditions):
        values = results_by_condition[condition_index]
        if values:
            mean = statistics.mean(values)
            std = statistics.stdev(values) if len(values) > 1 else 0.0
            print(f'  Condicion {condition_index} (v={vx}, T={duration}): '
                  f'n={len(values)} desplazamiento medio={mean:.3f} m (std={std:.3f})')
        else:
            print(f'  Condicion {condition_index}: sin datos validos')

    # Apagar el contexto antes de destruir el nodo: si se destruye primero
    # mientras el hilo de spin de fondo todavia lo esta usando, rclpy aborta
    # el proceso con "terminate called without an active exception".
    rclpy.shutdown()
    spin_thread.join(timeout=2.0)
    node.destroy_node()


if __name__ == '__main__':
    main()
