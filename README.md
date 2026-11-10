# Gemelo digital de un robot móvil omnidireccional tipo Mecanum

Trabajo de Fin de Máster — Máster Universitario en Sistemas Interactivos Inteligentes, Universidad Autónoma de Madrid.

**Autora:** Daniela Jiménez Da Silva
**Tutor:** Jaime Moreno Llorena, Departamento de Ingeniería Informática

## Descripción

Este repositorio contiene el código fuente de un gemelo digital desarrollado para un robot móvil omnidireccional de cuatro ruedas Mecanum (plataforma Freenove, controlada mediante Raspberry Pi), orientado a la monitorización en tiempo real y la detección de fallos sobre una plataforma robótica de bajo coste.

El sistema se construye sobre ROS 2 como infraestructura de comunicación distribuida entre la Raspberry Pi del robot y un ordenador externo, un modelo geométrico desarrollado en Fusion 360 y traducido a URDF/Xacro, y un modelo cinemático que relaciona las órdenes de movimiento del chasis con las velocidades individuales de las cuatro ruedas Mecanum. Sobre esta base se implementan funcionalidades de monitorización bidireccional en RViz, escaneo y navegación reactiva mediante un sensor ultrasónico montado en una cabeza móvil, detección de fallos basada en umbrales, y una representación paralela del sistema en el simulador Gazebo.

## Estructura del repositorio

- **`src/mecanum_hardware/`** — Paquete ROS 2 que corre directamente en la Raspberry Pi del robot: control de motores, lectura de sensores (ultrasónico, infrarrojos de línea, cámara, servos de la cabeza) y publicación de los datos en ROS 2.
- **`src/mecanum_robot/`** — Paquete ROS 2 que corre en el ordenador externo: modelo URDF/Xacro del robot, nodos de monitorización, interpretación de escaneo, toma de decisiones y control reactivo, detección de fallos, teleoperación, puentes con Gazebo, y los scripts de validación usados en las pruebas del TFM.
- **`rosbags/`** — Grabaciones de pruebas de navegación usadas durante el desarrollo.

## Requisitos

- ROS 2 (desarrollado sobre Ubuntu 22.04)
- Gazebo Classic
- Python 3

## Memoria

El documento completo del TFM, con la descripción detallada del sistema, la metodología y los resultados, no se incluye en este repositorio.
