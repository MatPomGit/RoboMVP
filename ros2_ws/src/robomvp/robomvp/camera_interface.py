#!/usr/bin/env python3
"""Interfejs kamer dla systemu RoboMVP.

Węzeł ROS2 publikujący obrazy z obu kamer robota Unitree G1 EDU.
Obsługuje dwa tryby pracy:

- **demo_mode**: wczytuje obrazy PNG/JPG z katalogu ``data/test_images/``
  i publikuje je cyklicznie na tematy kamer. Nie wymaga sprzętu.
- **robot_mode**: otwiera kamery sprzętowe przez OpenCV VideoCapture
  korzystając z konfigurowalnych indeksów urządzeń V4L2
  (``body_camera_device``, ``head_camera_device``).

Publikowane tematy:
    /camera/body/image_raw  – obraz kamery ciała (manipulacja, bliskie markery)
    /camera/head/image_raw  – obraz kamery głowy (nawigacja, dalekie markery)

Parametry ROS2:
    mode               – tryb pracy: ``demo_mode`` lub ``robot_mode``
    test_images_path   – ścieżka do katalogu z obrazami testowymi (tryb demo)
    publish_rate       – częstotliwość publikacji [Hz], domyślnie 10.0
    body_camera_device – indeks urządzenia V4L2 kamery ciała, domyślnie 0
    head_camera_device – indeks urządzenia V4L2 kamery głowy, domyślnie 1
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraInterface(Node):
    """Węzeł ROS2 publikujący obrazy z kamer robota."""

    def __init__(self):
        super().__init__('camera_interface')

        # Deklaracja parametrów węzła
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('body_camera_device', 0)
        self.declare_parameter('head_camera_device', -1)

        self._publish_rate = (
            self.get_parameter('publish_rate').get_parameter_value().double_value
        )
        self._body_camera_device = self._read_device_parameter('body_camera_device')
        self._head_camera_device = self._read_device_parameter('head_camera_device')

        self._bridge = CvBridge()

        # Wydawcy tematów kamer
        self._pub_body = self.create_publisher(Image, '/camera/body/image_raw', 10)
        self._pub_head = self.create_publisher(Image, '/camera/head/image_raw', 10)

        # Kamera ciała jest wymagana
        self._body_cap = self._open_capture(self._body_camera_device, 'ciała')
        if self._body_cap is None:
            raise RuntimeError(
                'Nie udało się otworzyć wymaganej kamery ciała. '
                'Aplikacja działa wyłącznie na rzeczywistej kamerze robota. '
                'Sprawdź parametr body_camera_device i /dev/video*.'
            )

        # Kamera głowy jest opcjonalna
        self._head_cap = None
        self._use_body_as_head = False
        if self._head_camera_device < 0:
            self._use_body_as_head = True
            self.get_logger().warning(
                'Kamera głowy wyłączona (head_camera_device < 0). '
                'Publikuję obraz z kamery ciała również na /camera/head/image_raw.'
            )
        else:
            self._head_cap = self._open_capture(self._head_camera_device, 'głowy')
            if self._head_cap is None:
                self._use_body_as_head = True
                self.get_logger().warning(
                    'Nie można otworzyć kamery głowy - przełączam się na obraz '
                    'z kamery ciała dla /camera/head/image_raw.'
                )

        # Timer publikowania obrazów
        self._timer = self.create_timer(1.0 / self._publish_rate, self._publish_images)

        head_description = (
            'body_fallback'
            if self._use_body_as_head
            else f'/dev/video{self._head_camera_device}'
        )
        self.get_logger().info(
            'Kamery sprzętowe zainicjalizowane. '
            f'body=/dev/video{self._body_camera_device}, '
            f'head={head_description}.'
        )

    def _read_device_parameter(self, name: str) -> int:
        """Odczytuje indeks urządzenia kamery i wymusza poprawny typ int."""
        param = self.get_parameter(name).value
        try:
            return int(param)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f'Parametr {name} musi być liczbą całkowitą, otrzymano: {param}'
            ) from exc

    def _open_capture(self, device_index: int, camera_name: str):
        """Otwiera pojedyncze urządzenie kamerowe przez OpenCV VideoCapture."""
        cap = cv2.VideoCapture(device_index)
        if cap.isOpened():
            self.get_logger().info(
                f'Kamera {camera_name} otwarta pomyślnie '
                f'(urządzenie /dev/video{device_index}).'
            )
            return cap
        cap.release()
        self.get_logger().error(
            f'Nie można otworzyć kamery {camera_name} '
            f'(urządzenie /dev/video{device_index}). '
            'Sprawdź, czy urządzenie istnieje i ma odpowiednie uprawnienia.'
        )
        return None

    def _publish_images(self):
        """Publikuje aktualny obraz na tematy kamer."""
        stamp = self.get_clock().now().to_msg()

        body_img = self._read_capture(self._body_cap, 'ciała')
        if self._use_body_as_head:
            head_img = body_img
        else:
            head_img = self._read_capture(self._head_cap, 'głowy')

        body_msg = self._bridge.cv2_to_imgmsg(body_img, encoding='bgr8')
        body_msg.header.stamp = stamp
        body_msg.header.frame_id = 'body_camera_frame'
        self._pub_body.publish(body_msg)

        head_msg = self._bridge.cv2_to_imgmsg(head_img, encoding='bgr8')
        head_msg.header.stamp = stamp
        head_msg.header.frame_id = 'head_camera_frame'
        self._pub_head.publish(head_msg)

    def _read_capture(self, cap, camera_name: str):
        """Odczytuje jedną klatkę z urządzenia VideoCapture."""
        if cap is not None and cap.isOpened():
            ret, frame = cap.read()
            if ret:
                return frame
        raise RuntimeError(
            f'Błąd odczytu klatki z kamery {camera_name}. '
            'Aplikacja nie może kontynuować bez rzeczywistych kamer.'
        )

    def destroy_node(self):
        """Zwalnia zasoby kamer i niszczy węzeł."""
        if self._body_cap is not None:
            self._body_cap.release()
            self._body_cap = None
        if self._head_cap is not None:
            self._head_cap.release()
            self._head_cap = None
        super().destroy_node()


def main(args=None):
    """Punkt wejścia węzła camera_interface."""
    rclpy.init(args=args)
    node = None
    try:
        node = CameraInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f'Błąd krytyczny camera_interface: {exc}')
        else:
            print(f'Błąd krytyczny camera_interface: {exc}')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
