#!/usr/bin/env python3
"""Moduł detekcji markerów wizualnych dla systemu RoboMVP.

Subskrybuje obrazy z obu kamer i wykrywa markery AprilTag lub QR.
Publikuje wykrycia jako wiadomości MarkerDetection.
"""

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from robomvp.logger_utils import stamp
from robomvp.msg import MarkerDetection


class MarkerDetectionNode(Node):
    """Węzeł ROS2 wykrywający markery wizualne w obrazach kamer."""

    def __init__(self):
        super().__init__('marker_detection')

        self.declare_parameter('marker_type', 'apriltag')
        self._marker_type = (
            self.get_parameter('marker_type').get_parameter_value().string_value
        )

        self._bridge = CvBridge()
        self._detector = self._init_detector()

        # Subskrypcje tematów kamer
        self._sub_body = self.create_subscription(
            Image, '/camera/body/image_raw', self._on_body_image, 10
        )
        self._sub_head = self.create_subscription(
            Image, '/camera/head/image_raw', self._on_head_image, 10
        )

        # Wydawca wykryć markerów
        self._pub_detections = self.create_publisher(
            MarkerDetection, '/robomvp/marker_detections', 10
        )

        self.get_logger().info(stamp(
            f'Węzeł detekcji markerów uruchomiony (typ: {self._marker_type}). '
            'Oczekiwanie na obrazy z kamer.'
        ))

    def _init_detector(self):
        """Inicjalizuje detektor markerów AprilTag lub QR."""
        if self._marker_type == 'apriltag':
            try:
                import apriltag
                options = apriltag.DetectorOptions(families='tag36h11')
                return apriltag.Detector(options)
            except ImportError:
                self.get_logger().warn(stamp(
                    'Biblioteka apriltag niedostępna. '
                    'Używam detektora QR jako zastępczego. '
                    'Aby zainstalować: pip install apriltag'
                ))
                return self._init_qr_detector()
        return self._init_qr_detector()

    def _init_qr_detector(self):
        """Inicjalizuje detektor kodów QR z OpenCV."""
        import cv2
        return cv2.QRCodeDetector()

    def _on_body_image(self, msg: Image):
        """Przetwarza obraz z kamery ciała."""
        self._process_image(msg, source='body')

    def _on_head_image(self, msg: Image):
        """Przetwarza obraz z kamery głowy."""
        self._process_image(msg, source='head')

    def _process_image(self, msg: Image, source: str):
        """Wykrywa markery w obrazie i publikuje wyniki."""
        try:
            import cv2
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            detections = self._detect_markers(gray, cv_image)
            for detection in detections:
                self._pub_detections.publish(detection)
                self.get_logger().debug(stamp(
                    f'[{source}] Wykryto marker ID={detection.marker_id} '
                    f'pos=({detection.image_x:.1f}, {detection.image_y:.1f})'
                ))
        except Exception as e:
            self.get_logger().error(stamp(
                f'Błąd przetwarzania obrazu [{source}]: {e}. '
                'Sprawdź poprawność wiadomości obrazu i instalację biblioteki cv_bridge.'
            ))

    def _detect_markers(self, gray: np.ndarray, color: np.ndarray) -> list:
        """Wykrywa markery i zwraca listę wiadomości MarkerDetection."""
        results = []

        # Prefer AprilTag detection when configured and the detector supports it.
        if self._marker_type == 'apriltag' and hasattr(self._detector, 'detect'):
            results = self._detect_apriltags(gray)
        # Fallback to QR detection when the detector has the appropriate API
        # (e.g. when AprilTag import failed and a QRCodeDetector was created).
        elif hasattr(self._detector, 'detectAndDecode'):
            results = self._detect_qr(color)
        else:
            self.get_logger().error(
                'Brak obsługi detektora markerów: nieobsługiwany typ detektora.'
            )
            results = []

        return results

    def _detect_apriltags(self, gray: np.ndarray) -> list:
        """Wykrywa markery AprilTag i zwraca listę MarkerDetection."""
        results = []
        try:
            detections = self._detector.detect(gray)
            for d in detections:
                msg = MarkerDetection()
                msg.marker_id = int(d.tag_id)
                msg.image_x = float(d.center[0])
                msg.image_y = float(d.center[1])
                # Rozmiar znacznika jako średnia przekątna ramki
                corners = d.corners
                size = float(np.mean([
                    np.linalg.norm(corners[0] - corners[2]),
                    np.linalg.norm(corners[1] - corners[3])
                ]))
                msg.size = size
                results.append(msg)
        except Exception as e:
            self.get_logger().warn(stamp(
                f'Błąd detekcji AprilTag: {e}. '
                'Sprawdź, czy biblioteka apriltag jest poprawnie zainstalowana '
                'i czy obraz ma właściwy format.'
            ))
        return results

    def _detect_qr(self, color: np.ndarray) -> list:
        """Wykrywa kody QR i zwraca listę MarkerDetection."""
        results = []
        try:
            data, points, _ = self._detector.detectAndDecode(color)
            if points is not None and data:
                points = points[0]
                center_x = float(np.mean(points[:, 0]))
                center_y = float(np.mean(points[:, 1]))
                size = float(np.mean([
                    np.linalg.norm(points[0] - points[2]),
                    np.linalg.norm(points[1] - points[3])
                ]))
                marker_id = self._parse_qr_id(data)
                msg = MarkerDetection()
                msg.marker_id = marker_id
                msg.image_x = center_x
                msg.image_y = center_y
                msg.size = size
                results.append(msg)
        except Exception as e:
            self.get_logger().warn(stamp(
                f'Błąd detekcji QR: {e}. '
                'Sprawdź, czy OpenCV jest poprawnie zainstalowane '
                'i czy marker QR jest wyraźnie widoczny w obrazie.'
            ))
        return results

    def _parse_qr_id(self, data: str) -> int:
        """Parsuje identyfikator z danych QR (zakłada format 'ID:N')."""
        try:
            if data.startswith('ID:'):
                return int(data.split(':')[1])
            return int(data)
        except (ValueError, IndexError):
            return 0


def main(args=None):
    """Punkt wejścia węzła marker_detection."""
    rclpy.init(args=args)
    node = MarkerDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
