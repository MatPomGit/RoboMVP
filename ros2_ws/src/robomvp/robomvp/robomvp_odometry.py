#!/usr/bin/env python3
"""Węzeł odometrii dla systemu RoboMVP – dead reckoning z komend prędkości.

CZYM JEST ODOMETRIA I DLACZEGO JEST WAŻNA?
===========================================
Odometria to szacowanie pozycji i orientacji robota w czasie, na podstawie
pomiarów ruchu. W klasycznym robocie kołowym odometria pochodzi z enkoderów
kół – każdy impuls enkodera odpowiada określonemu przesunięciu.

Robot humanoidalny Unitree G1 EDU w trybie Sport Mode NIE udostępnia
odometrii bezpośrednio przez LocoClient. Zamiast tego stosujemy
DEAD RECKONING (zliczanie przebiegu) – całkujemy komendy prędkości po czasie:

    pozycja_x(t) = pozycja_x(t-1) + vx * cos(yaw) * dt - vy * sin(yaw) * dt
    pozycja_y(t) = pozycja_y(t-1) + vx * sin(yaw) * dt + vy * cos(yaw) * dt
    yaw(t)       = yaw(t-1) + vyaw * dt

To jest aproksymacja – robot może się ześlizgnąć lub utknąć, a my o tym nie
wiemy. Ale dla krótkiego scenariusza MVP (~3 metry marszu) błąd jest akceptowalny.

GDZIE JEST UŻYWANA ODOMETRIA?
W pełnym systemie nawigacyjnym (ROS2 Nav2) odometria zasila EKF (Extended
Kalman Filter), który łączy ją z IMU, dając lepszą estymację pozycji.
W RoboMVP publikujemy odometrię jako nav_msgs/Odometry żeby:
  - RViz mógł narysować ścieżkę robota
  - Narzędzia debugowania (rqt_plot) mogły wyświetlić trajektorię
  - Przyszłe wersje mogły podłączyć Nav2 lub EKF bez zmian architektury

Subskrybowane tematy:
    /robomvp/motion_command  – komendy ruchu (aby wiedzieć czy robot się rusza)

Wewnętrznie węzeł monitoruje komendy prędkości przez VelocityTracker
(klasa wewnętrzna) aktualizowaną przez UnitreeRobotAPI przez shared state.

Publikowane tematy:
    /odom              – nav_msgs/Odometry (pozycja, orientacja, prędkość)
    /tf                – transformacja odom → base_link (wymagana przez Nav2/RViz)
"""

import math
import threading

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class OdometryNode(Node):
    """Węzeł publikujący odometrię dead-reckoning na podstawie komend prędkości."""

    def __init__(self):
        super().__init__('robomvp_odometry')

        self.declare_parameter('publish_rate', 50.0)   # 50 Hz = standardowe dla odometrii
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')

        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self._frame_id       = self.get_parameter('frame_id').get_parameter_value().string_value
        self._child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        # Stan odometrii – całkowany w czasie
        self._x:   float = 0.0
        self._y:   float = 0.0
        self._yaw: float = 0.0

        # Bieżące prędkości (aktualizowane przez set_velocity, wywoływane przez API robota)
        self._vx:   float = 0.0
        self._vy:   float = 0.0
        self._vyaw: float = 0.0
        self._vel_lock = threading.Lock()

        self._last_update_time = self.get_clock().now()

        # TF Broadcaster wysyła transformację odom → base_link do drzewa tf2.
        # RViz używa tf2 do wyświetlania modeli robotów w odpowiednim miejscu.
        self._tf_broadcaster = TransformBroadcaster(self)

        self._pub_odom = self.create_publisher(Odometry, '/odom', 10)

        # Subskrybujemy komendy ruchu żeby logować kiedy robot się porusza
        self.create_subscription(String, '/robomvp/motion_command', self._on_motion_cmd, 10)

        self.create_timer(1.0 / rate, self._update_and_publish)
        self.get_logger().info(
            f'Węzeł odometrii uruchomiony ({rate:.0f} Hz, '
            f'frame: {self._frame_id} → {self._child_frame_id}).'
        )

    # ------------------------------------------------------------------
    # Interfejs publiczny – wywoływany przez UnitreeRobotAPI
    # ------------------------------------------------------------------

    def set_velocity(self, vx: float, vy: float, vyaw: float):
        """Aktualizuje bieżące prędkości dla całkowania odometrii.

        Wywoływana przez UnitreeRobotAPI._send_velocity() za każdym razem
        gdy robot dostaje nową komendę prędkości. Mutex chroni przed race
        condition gdy update_and_publish działa jednocześnie.
        """
        with self._vel_lock:
            self._vx   = vx
            self._vy   = vy
            self._vyaw = vyaw

    def stop_velocity(self):
        """Zeruje prędkości po zatrzymaniu robota (StopMove)."""
        with self._vel_lock:
            self._vx   = 0.0
            self._vy   = 0.0
            self._vyaw = 0.0

    # ------------------------------------------------------------------
    # Callback subskrypcji
    # ------------------------------------------------------------------

    def _on_motion_cmd(self, msg: String):
        if msg.data in ('finished', 'align_with_box'):
            self.stop_velocity()

    # ------------------------------------------------------------------
    # Całkowanie i publikacja
    # ------------------------------------------------------------------

    def _update_and_publish(self):
        """Całkuje prędkości, aktualizuje pozę i publikuje odometrię + TF."""
        now = self.get_clock().now()
        dt = (now - self._last_update_time).nanoseconds * 1e-9
        self._last_update_time = now

        # Odczyt prędkości pod lockiem (krótki czas – minimalna blokada)
        with self._vel_lock:
            vx, vy, vyaw = self._vx, self._vy, self._vyaw

        # Całkowanie Eulera (wystarczające przy 50 Hz i małych prędkościach)
        # Transformacja z układu ciała (body frame) do układu świata (world frame):
        #   dx_world = vx * cos(yaw) - vy * sin(yaw)
        #   dy_world = vx * sin(yaw) + vy * cos(yaw)
        self._x   += (vx * math.cos(self._yaw) - vy * math.sin(self._yaw)) * dt
        self._y   += (vx * math.sin(self._yaw) + vy * math.cos(self._yaw)) * dt
        self._yaw += vyaw * dt

        # Normalizacja yaw do [-π, π] zapobiega przekroczeniu zakresu float
        self._yaw = (self._yaw + math.pi) % (2 * math.pi) - math.pi

        stamp = now.to_msg()
        quat  = self._yaw_to_quaternion(self._yaw)

        # ── Wiadomość Odometry ────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self._frame_id
        odom.child_frame_id  = self._child_frame_id

        odom.pose.pose = Pose(
            position    = Point(x=self._x, y=self._y, z=0.0),
            orientation = Quaternion(**quat),
        )
        odom.twist.twist = Twist(
            linear  = Vector3(x=vx,   y=vy,   z=0.0),
            angular = Vector3(x=0.0,  y=0.0,  z=vyaw),
        )

        # Macierz kowariancji 6×6 (spłaszczona do listy 36 elementów).
        # Duże wartości na przekątnej = duża niepewność (dead reckoning dryfuje).
        # Nav2 / EKF używa tej macierzy do ważenia odometrii względem innych sensorów.
        odom.pose.covariance[0]  = 0.1   # σ²(x)
        odom.pose.covariance[7]  = 0.1   # σ²(y)
        odom.pose.covariance[35] = 0.05  # σ²(yaw)

        self._pub_odom.publish(odom)

        # ── Transformacja TF odom → base_link ────────────────────────
        tf_msg                     = TransformStamped()
        tf_msg.header.stamp        = stamp
        tf_msg.header.frame_id     = self._frame_id
        tf_msg.child_frame_id      = self._child_frame_id
        tf_msg.transform.translation.x = self._x
        tf_msg.transform.translation.y = self._y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation   = Quaternion(**quat)
        self._tf_broadcaster.sendTransform(tf_msg)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> dict:
        """Konwertuje kąt yaw (obrót wokół osi Z) na kwaternion.

        Kwaternion to czterowymiarowa liczba zespolona reprezentująca obrót
        w przestrzeni 3D. Dla obrotu wyłącznie wokół osi Z (ruch w płaszczyźnie)
        x=0, y=0, a w i z wynikają z kąta pół-obrotu:
            w = cos(yaw/2),  z = sin(yaw/2)
        """
        return dict(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
