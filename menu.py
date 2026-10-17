#!/usr/bin/env python3
"""Menu orquestador para el proyecto mecanum_robot / mecanum_hardware.

No es un nodo ROS2 -- es un script que lanza/para por SSH los nodos de la
Raspberry Pi y localmente los de la laptop, segun el modo elegido, y limpia
todo al salir (incluyendo procesos remotos huerfanos, que con SSH no mueren
solos al matar el proceso local).
"""
import os
import signal
import subprocess
import sys
import time

PI_HOST = 'austin@192.168.50.145'
PI_SOURCE = 'cd ~/tfm_ws && source install/setup.bash'
LOCAL_SOURCE = 'source /opt/ros/humble/setup.bash && cd /home/danilaly/tfm_ws && source install/setup.bash'

# nombre corto -> (paquete, ejecutable)
PI_NODES = {
    'motor': ('mecanum_hardware', 'mecanum_motor_node'),
    'ultrasonic': ('mecanum_hardware', 'ultrasonic_sensor_node'),
    'line': ('mecanum_hardware', 'line_tracking_sensor_node'),
    'head_pan': ('mecanum_hardware', 'head_pan_node'),
    'camera': ('mecanum_hardware', 'usb_camera_node'),
    'scan': ('mecanum_hardware', 'head_ultrasonic_scan_node'),
}

LOCAL_NODES = {
    'keyboard': ('mecanum_robot', 'keyboard_command_node'),
    'control': ('mecanum_robot', 'navigation_control_node'),
    'visual_rotation': ('mecanum_robot', 'visual_rotation_test_node'),
}

MODES = {
    '1': {
        'name': 'Teleclado (mover con teclado)',
        'pi': ['motor'],
        'local': ['keyboard'],
        'rviz': True,
    },
    '2': {
        'name': 'Prueba de ultrasonido',
        'pi': ['ultrasonic', 'head_pan'],  # head_pan es necesario: ultrasonic_link cuelga de head_link
        'local': [],
        'rviz': True,
    },
    '3': {
        'name': 'Prueba de linea',
        'pi': ['line'],
        'local': [],
        'rviz': True,
    },
    '4': {
        'name': 'Prueba de camara',
        'pi': ['camera'],
        'local': [],
        'rviz': True,
    },
    '5': {
        'name': 'Navegacion autonoma completa (percepcion + decision + control)',
        'pi': ['motor', 'ultrasonic', 'head_pan', 'scan'],
        'local': ['control'],
        'rviz': True,
    },
    '6': {
        'name': 'Experimento de rotacion visual',
        'pi': ['motor', 'head_pan', 'camera'],
        'local': ['visual_rotation'],
        'rviz': False,
    },
    '7': {
        'name': 'Robot completo conectado (todos los sensores, sin movimiento autonomo)',
        # sin 'scan': ese nodo mueve la cabeza activamente (barrido), y sin
        # 'control': nada mueve el chasis. head_pan solo centra una vez y se
        # queda quieto si nadie le manda comandos nuevos.
        'pi': ['motor', 'ultrasonic', 'line', 'head_pan', 'camera'],
        'local': [],
        'rviz': True,
    },
}

running_processes = []  # lista de subprocess.Popen (locales, incluyendo los que hacen ssh)
pi_nodes_started = []  # nombres cortos de nodos de la Pi que se lanzaron, para limpiar despues


def launch_pi_node(name):
    package, executable = PI_NODES[name]
    cmd = f"{PI_SOURCE} && ros2 run {package} {executable}"
    proc = subprocess.Popen(['ssh', PI_HOST, cmd], start_new_session=True)
    running_processes.append(proc)
    pi_nodes_started.append(name)
    print(f"  -> Pi: {executable}")


def launch_local_node(name):
    package, executable = LOCAL_NODES[name]
    cmd = f"{LOCAL_SOURCE} && ros2 run {package} {executable}"
    # start_new_session=True mete el proceso (y todo lo que lance) en su propio
    # grupo, para poder matarlo completo despues con os.killpg().
    proc = subprocess.Popen(['bash', '-c', cmd], start_new_session=True)
    running_processes.append(proc)
    print(f"  -> laptop: {executable}")


def launch_rviz():
    cmd = f"{LOCAL_SOURCE} && ros2 launch mecanum_robot display.launch.py"
    proc = subprocess.Popen(['bash', '-c', cmd], start_new_session=True)
    running_processes.append(proc)
    print("  -> laptop: RViz (display.launch.py)")


def stop_everything():
    print("\nDeteniendo todo...")

    for proc in running_processes:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
    time.sleep(1)
    for proc in running_processes:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
    running_processes.clear()

    # Los nodos de la Pi lanzados por SSH no siempre mueren al matar el
    # cliente ssh local, asi que se limpian explicitamente por nombre.
    # El patron se manda por stdin (no como argumento del comando ssh) para
    # que pkill -f no se autodetecte a si mismo en el cmdline del proceso
    # que lo invoca (bug real que ya nos mordio hoy mismo).
    if pi_nodes_started:
        # Nombre simple del ejecutable: coincide tanto con el proceso "ros2 run"
        # envoltorio como con el proceso real instalado en lib/mecanum_hardware/.
        patterns = [PI_NODES[name][1] for name in set(pi_nodes_started)]
        script_lines = [f'pkill -9 -f "{p}" 2>/dev/null' for p in patterns]
        script = (
            '\n'.join(script_lines)
            + '\nsleep 1\n'
            + 'lsof /dev/gpiochip0 2>/dev/null\n'
            + 'fuser /dev/video0 2>/dev/null\n'
            + 'echo pi_clean\n'
        )
        result = subprocess.run(
            ['ssh', PI_HOST, 'bash -s'], input=script, capture_output=True, text=True
        )
        print(result.stdout.strip())
        if result.returncode != 0:
            print(f"(aviso: limpieza remota devolvio codigo {result.returncode}: {result.stderr.strip()})")
        pi_nodes_started.clear()

    print("Listo, todo detenido.")


def run_mode(mode):
    print(f"\nLanzando: {mode['name']}")
    for name in mode['pi']:
        launch_pi_node(name)
        time.sleep(1.5)  # evita condiciones de carrera al abrir varias sesiones SSH de golpe
    for name in mode['local']:
        launch_local_node(name)
    if mode['rviz']:
        launch_rviz()
    print("\nCorriendo. Presiona Ctrl+C para detener y volver al menu.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    stop_everything()


def print_menu():
    print("\n=== Menu mecanum_robot ===")
    for key, mode in MODES.items():
        print(f"  {key}) {mode['name']}")
    print("  0) Salir")


def main():
    while True:
        print_menu()
        choice = input("Elige una opcion: ").strip()
        if choice == '0':
            break
        elif choice in MODES:
            run_mode(MODES[choice])
        else:
            print("Opcion invalida.")


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        stop_everything()
        sys.exit(0)
