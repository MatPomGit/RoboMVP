#!/usr/bin/env python3
"""Moduł estymacji pozy markerów dla systemu RoboMVP.

Subskrybuje wykrycia markerów i oblicza ich pozycję 3D
używając parametrów kalibracji kamery.
Publikuje pozę markera oraz obliczony offset korekcji.
"""

import math

import numpy as np
import rclpy
import yaml
from rclpy.node import Node

from robomvp.logger_utils import stamp
from robomvp.msg import MarkerDetection, MarkerPose, Offset


class MarkerPoseEstimatorNode(Node):
    """Węzeł ROS2 obliczający pozycję 3D markerów na podstawie wykryć."""

    def __init__(self):
        super().__init__('marker_pose_estimator')

        self.declare_parameter('camera_config_path', '')
        self.declare_parameter('marker_size', 0.1)

        config_path = (
            self.get_parameter('camera_config_path')
            .get_parameter_value()
            .string_value
        )
        self._marker_size = (
            self.get_parameter('marker_size').get_parameter_value().double_value
        )

        # Wczytanie parametrów kalibracji kamery
        self._camera_matrix, self._dist_coeffs = self._load_camera_params(config_path)

        # Subskrypcja wykryć markerów
        self._sub = self.create_subscription(
            MarkerDetection,
            '/robomvp/marker_detections',
            self._on_detection,
            10,
        )

        # Wydawcy: poza markera i offset korekcji
        self._pub_pose = self.create_publisher(MarkerPose, '/robomvp/marker_pose', 10)
        self._pub_offset = self.create_publisher(Offset, '/robomvp/offset', 10)

        self.get_logger().info(stamp(
            'Węzeł estymacji pozy markerów uruchomiony. '
            'Oczekiwanie na wykrycia markerów.'
        ))

    def _load_camera_params(self, config_path: str):
        """Wczytuje macierz kamery i współczynniki dystorsji z pliku YAML."""
        default_matrix = np.array([
            [600.0, 0.0, 320.0],
            [0.0, 600.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        default_dist = np.zeros((5, 1), dtype=np.float64)

        if not config_path:
            self.get_logger().warn(stamp(
                'Brak ścieżki do konfiguracji kamery – używam domyślnych parametrów. '
                'Podaj ścieżkę przez parametr: --ros-args -p camera_config_path:=<ścieżka>'
            ))
            return default_matrix, default_dist

        try:
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)
            cam = cfg.get('body_camera', {})
            cm = cam.get('camera_matrix', {})
            dc = cam.get('distortion_coeffs', {})
            matrix = np.array([
                [cm.get('fx', 600.0), 0.0, cm.get('cx', 320.0)],
                [0.0, cm.get('fy', 600.0), cm.get('cy', 240.0)],
                [0.0, 0.0, 1.0]
            ], dtype=np.float64)
            dist = np.array([
                [dc.get('k1', 0.0)],
                [dc.get('k2', 0.0)],
                [dc.get('p1', 0.0)],
                [dc.get('p2', 0.0)],
                [dc.get('k3', 0.0)]
            ], dtype=np.float64)
            if cfg.get('marker_size'):
                self._marker_size = float(cfg['marker_size'])
            return matrix, dist
        except Exception as e:
            self.get_logger().warn(stamp(
                f'Błąd wczytywania konfiguracji kamery: {e}. '
                'Sprawdź format pliku camera.yaml i wartości parametrów. '
                'Używam domyślnych parametrów kalibracji.'
            ))
            return default_matrix, default_dist

    def _on_detection(self, msg: MarkerDetection):
        """Oblicza pozę 3D markera i publikuje wyniki."""
        try:
            pose = self._estimate_pose(msg)
            self._pub_pose.publish(pose)

            offset = self._compute_offset(pose)
            self._pub_offset.publish(offset)

            self.get_logger().debug(stamp(
                f'Marker {msg.marker_id}: '
                f'pos=({pose.x:.3f}, {pose.y:.3f}, {pose.z:.3f}) m, '
                f'offset=({offset.dx:.3f}, {offset.dy:.3f}, {offset.dz:.3f}) m'
            ))
        except Exception as e:
            self.get_logger().error(stamp(
                f'Błąd estymacji pozy markera {msg.marker_id}: {e}. '
                'Sprawdź parametry kalibracji kamery i format wiadomości wykrycia.'
            ))

    def _estimate_pose(self, msg: MarkerDetection) -> MarkerPose:
        """Oblicza przybliżoną pozycję 3D markera na podstawie jego rozmiaru w obrazie.

        Używa modelu aparatu otworkowego: z = (f * rozmiar_rzeczywisty) / rozmiar_pikselowy.
        """
        import cv2

        pose = MarkerPose()
        pose.marker_id = msg.marker_id

        # Ogniskowa w pikselach (średnia fx, fy)
        fx = float(self._camera_matrix[0, 0])
        fy = float(self._camera_matrix[1, 1])
        cx = float(self._camera_matrix[0, 2])
        cy = float(self._camera_matrix[1, 2])
        f_avg = (fx + fy) / 2.0

        # Szacowanie głębokości z rozmiaru markera w obrazie
        if msg.size > 0:
            z = (f_avg * self._marker_size) / msg.size
        else:
            z = 1.0

        # Pozycja X, Y na podstawie odchylenia od centrum obrazu
        x = (msg.image_x - cx) * z / fx
        y = (msg.image_y - cy) * z / fy

        pose.x = float(x)
        pose.y = float(y)
        pose.z = float(z)

        # Poza markera: zakładamy marker równoległy do płaszczyzny obrazu
        # Orientacja: identyczny kwaternion (brak obrotu)
        pose.qx = 0.0
        pose.qy = 0.0
        pose.qz = 0.0
        pose.qw = 1.0

        # Dokładniejsza estymacja za pomocą solvePnP, gdy dostępne narożniki
        # (wymagałoby rozszerzenia wiadomości MarkerDetection o narożniki)

        # Korekta dystorsji punktu centralnego
        point = np.array([[[msg.image_x, msg.image_y]]], dtype=np.float32)
        undistorted = cv2.undistortPoints(
            point, self._camera_matrix, self._dist_coeffs, P=self._camera_matrix
        )
        u = float(undistorted[0, 0, 0])
        v = float(undistorted[0, 0, 1])
        pose.x = float((u - cx) * z / fx)
        pose.y = float((v - cy) * z / fy)

        return pose

    def _compute_offset(self, pose: MarkerPose) -> Offset:
        """Oblicza offset korekcji między aktualną a oczekiwaną pozycją markera.

        Zakładamy, że oczekiwana pozycja to (0, 0, z) - marker na wprost.
        """
        offset = Offset()
        # Korekcja boczna: odchylenie w osi X
        offset.dx = -pose.x
        # Korekcja przód/tył: zerowa (głębokość kontroluje sekwencja ruchu)
        offset.dy = 0.0
        # Korekcja pionowa: odchylenie w osi Y kamery
        offset.dz = -pose.y
        return offset

    def _rvec_to_quaternion(self, rvec: np.ndarray):
        """Konwertuje wektor obrotu Rodriguesa na kwaternion.

        TODO: Użyć tej metody gdy _estimate_pose zostanie rozszerzone
        o pełną estymację pozy przez cv2.solvePnP z narożnikami markera.
        """
        angle = float(np.linalg.norm(rvec))
        if angle < 1e-10:
            return 0.0, 0.0, 0.0, 1.0
        axis = rvec.flatten() / angle
        s = math.sin(angle / 2.0)
        qx = float(axis[0] * s)
        qy = float(axis[1] * s)
        qz = float(axis[2] * s)
        qw = float(math.cos(angle / 2.0))
        return qx, qy, qz, qw


def main(args=None):
    """Punkt wejścia węzła marker_pose_estimator."""
    rclpy.init(args=args)
    node = MarkerPoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
