#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen

import sys
import tty
import termios
import threading
import time
import math


def get_key():
    """Lee una tecla del teclado en modo raw (sin Enter)."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


class TurtleController(Node):
    """
    Nodo único que:
      - Controla la tortuga con flechas del teclado.
      - Dibuja las letras E y L.
    """

    def __init__(self):
        super().__init__('turtle_controller')

        # Publicador de velocidad
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Pose actual de la tortuga
        self.current_pose = None
        self.pose_subscriber = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        # Cliente del servicio de teletransporte
        self.teleport_client = self.create_client(
            TeleportAbsolute,
            '/turtle1/teleport_absolute'
        )
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /turtle1/teleport_absolute...')

        # Cliente para levantar/bajar el lápiz
        self.set_pen_client = self.create_client(
            SetPen,
            '/turtle1/set_pen'
        )
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /turtle1/set_pen...')

        # Hilo de teclado
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

    # ------------------------------------------------------------------
    # Callbacks y utilidades básicas
    # ------------------------------------------------------------------
    def pose_callback(self, msg: Pose):
        self.current_pose = msg

    def save_initial_pose(self):
        if self.current_pose is None:
            self.get_logger().error("Pose no disponible aún")
            return None
        return (self.current_pose.x, self.current_pose.y, self.current_pose.theta)

    def teleport(self, x, y, theta):
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta

        future = self.teleport_client.call_async(req)
        start_time = time.time()

        while not future.done() and (time.time() - start_time < 2.0):
            time.sleep(0.1)

        if future.done():
            try:
                future.result()
            except Exception as e:
                self.get_logger().error(f"Error al teleportar: {e}")
        else:
            self.get_logger().error("Teleport no completado a tiempo")

    def set_pen(self, off: bool):
        """
        off = True  -> deja de dibujar
        off = False -> vuelve a dibujar
        """
        req = SetPen.Request()
        req.r = 255
        req.g = 255
        req.b = 255
        req.width = 2
        req.off = 1 if off else 0
        self.set_pen_client.call_async(req)

    def move_distance(self, distance, speed=2.0):
        duration = abs(distance) / speed
        msg = Twist()
        msg.linear.x = speed if distance >= 0 else -speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)

        self.publisher_.publish(Twist())
        time.sleep(0.1)

    def turn_angle(self, angle, angular_speed=1.0):
        duration = abs(angle) / angular_speed
        msg = Twist()
        msg.angular.z = angular_speed if angle >= 0 else -angular_speed

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)

        self.publisher_.publish(Twist())
        time.sleep(0.1)

    def move_step(self, linear=0.0, angular=0.0, duration=0.3):
        """Paso corto para movimiento con flechas."""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher_.publish(msg)
            time.sleep(0.05)

        self.publisher_.publish(Twist())

    # ------------------------------------------------------------------
    # Lector de teclado: flechas + letras
    # ------------------------------------------------------------------
    def keyboard_listener(self):
        self.get_logger().info(
            "Controles:\n"
            "  Flechas: mover tortuga\n"
            "  e: dibujar E\n"
            "  l: dibujar L\n"
            "  Ctrl+C: salir"
        )

        while rclpy.ok():
            key = get_key()

            # Flechas (ESC [ A/B/C/D)
            if key == '\x1b':
                second = get_key()
                if second == '[':
                    third = get_key()
                    if third == 'A':      # Arriba
                        self.move_step(linear=2.0, angular=0.0)
                    elif third == 'B':    # Abajo
                        self.move_step(linear=-2.0, angular=0.0)
                    elif third == 'C':    # Derecha
                        self.move_step(linear=0.0, angular=-1.5)
                    elif third == 'D':    # Izquierda
                        self.move_step(linear=0.0, angular=1.5)
                continue

            k = key.lower()
            if k == 'e':
                self.get_logger().info("Dibujando letra E")
                self.draw_E()
            elif k == 'l':
                self.get_logger().info("Dibujando letra L")
                self.draw_L()
            elif key == '\x03':  # Ctrl+C
                break

    # ------------------------------------------------------------------
    # Letras
    # ------------------------------------------------------------------
    def draw_E(self):
        """
        Letra E:
          - El punto donde está la tortuga es la ESQUINA SUPERIOR IZQUIERDA.
          - No hay trazo previo: lo primero que se ve es la propia letra.
        """
        initial = self.save_initial_pose()
        if not initial:
            return
        x0, y0, theta0 = initial

        h = 2.5        # altura total (hacia abajo)
        width = 1.5    # largo de horizontales
        spacing = 2.5  # separación hacia siguiente letra

        # Reposicionamiento invisible para fijar orientación a 0 rad (derecha)
        self.set_pen(True)
        self.teleport(x0, y0, 0.0)   # usamos este punto como esquina superior izquierda
        self.set_pen(False)

        # 1) Línea superior (de izquierda a derecha)
        #   estamos mirando a la derecha
        self.move_distance(width)

        # 2) Volver al borde izquierdo
        self.turn_angle(math.radians(180))   # mirar a la izquierda
        self.move_distance(width)

        # 3) Trazo vertical completo hacia abajo
        self.turn_angle(math.radians(90))    # de izquierda a abajo
        self.move_distance(h)                # hasta la esquina inferior izquierda

        # 4) Línea inferior (de izquierda a derecha)
        self.turn_angle(math.radians(90))    # de abajo a la derecha
        self.move_distance(width)

        # 5) Volver al borde izquierdo inferior
        self.turn_angle(math.radians(180))   # derecha -> izquierda
        self.move_distance(width)

        # 6) Subir hasta la mitad para la línea media
        self.turn_angle(math.radians(-90))   # izquierda -> arriba
        self.move_distance(h / 2)

        # 7) Línea media (un poco más corta)
        self.turn_angle(math.radians(-90))   # arriba -> derecha
        self.move_distance(width * 0.8)

        # 8) Volver al eje vertical (opcional)
        self.turn_angle(math.radians(180))   # derecha -> izquierda
        self.move_distance(width * 0.8)

        # 9) Reposicionar SIN dibujar a la posición de la siguiente letra
        self.set_pen(True)
        self.teleport(x0 + spacing, y0, 0.0)   # mismo nivel, desplazado a la derecha
        self.set_pen(False)

    def draw_L(self):
        """
        Letra L:
        - El punto donde está la tortuga es la ESQUINA SUPERIOR IZQUIERDA.
        - El palo baja y luego se dibuja la línea horizontal inferior.
        """
        initial = self.save_initial_pose()
        if not initial:
            return
        x0, y0, theta0 = initial

        h = 2.5
        width = 1.5
        spacing = 2.5  # si quieres dejar espacio para la siguiente letra

        # Reposicionamiento invisible para fijar orientación a 0 rad (mirando a la derecha)
        self.set_pen(True)
        self.teleport(x0, y0, 0.0)
        self.set_pen(False)

        # Palo vertical hacia abajo (como en la E)
        self.turn_angle(math.radians(-90))   # derecha -> abajo
        self.move_distance(h)                # bajamos hasta la esquina inferior izquierda

        # Línea horizontal inferior hacia la derecha
        self.turn_angle(math.radians(90))    # abajo -> derecha
        self.move_distance(width)

        # (Opcional) reposicionar para otra letra al mismo nivel
        self.set_pen(True)
        self.teleport(x0 + spacing, y0, 0.0)  # mismo nivel superior
        self.set_pen(False)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupción por teclado")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
