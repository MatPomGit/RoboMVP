#!/usr/bin/env python3
"""Interfejs kamer dla systemu RoboMVP.

Obsługuje dwie kamery robota:
- kamera ciała (manipulacja, bliskie markery)
- kamera głowy (nawigacja, dalekie markery)

W trybie demo wczytuje obrazy z katalogu data/test_images/.
W trybie robot otwiera kamery sprzętowe przez OpenCV VideoCapture
korzystając z konfigurowalnych indeksów urządzeń V4L2
(body_camera_device, head_camera_device).
"""

import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraInterface(Node):
    """Węzeł ROS2 publikujący obrazy z obu kamer robota."""

    def __init__(self):
        super().__init__('camera_interface')

        # Deklaracja parametrów węzła
        self.declare_parameter('mode', 'demo_mode')
        self.declare_parameter('test_images_path', '')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('body_camera_device', 0)
        self.declare_parameter('head_camera_device', 1)

        self._mode = self.get_parameter('mode').get_parameter_value().string_value
        self._publish_rate = (
            self.get_parameter('publish_rate').get_parameter_value().double_value
        )
        self._test_images_path = (
            self.get_parameter('test_images_path').get_parameter_value().string_value
        )
        self._body_camera_device = (
            self.get_parameter('body_camera_device').get_parameter_value().integer_value
        )
        self._head_camera_device = (
            self.get_parameter('head_camera_device').get_parameter_value().integer_value
        )

        self._bridge = CvBridge()

        # Wydawcy tematów kamer
        self._pub_body = self.create_publisher(Image, '/camera/body/image_raw', 10)
        self._pub_head = self.create_publisher(Image, '/camera/head/image_raw', 10)

        # Timer publikowania obrazów
        self._timer = self.create_timer(1.0 / self._publish_rate, self._publish_images)

        # Indeks aktualnego obrazu testowego (tryb demo)
        self._demo_index = 0
        self._demo_images = []

        # Uchwyty VideoCapture dla trybu robot
        self._body_cap = None
        self._head_cap = None

        if self._mode == 'demo_mode':
            self._load_demo_images()
            self.get_logger().info(
                f'Tryb demo: załadowano {len(self._demo_images)} obrazów testowych. '
                'Obrazy będą publikowane cyklicznie na tematy kamer.'
            )
        else:
            self._init_robot_cameras()
            self.get_logger().info(
                'Tryb robot: inicjalizacja kamer sprzętowych. '
                'Upewnij się, że robot jest podłączony i SDK jest dostępne.'
            )

    def _load_demo_images(self):
        """Wczytuje obrazy testowe z katalogu data/test_images/."""
        if not self._test_images_path:
            # Domyślna ścieżka względna do katalogu roboczego
            self._test_images_path = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                '..', '..', '..', '..', '..', 'data', 'test_images'
            )
        path = os.path.realpath(self._test_images_path)
        if os.path.isdir(path):
            extensions = ('.png', '.jpg', '.jpeg')
            for fname in sorted(os.listdir(path)):
                if fname.lower().endswith(extensions):
                    img_path = os.path.join(path, fname)
                    img = cv2.imread(img_path)
                    if img is not None:
                        self._demo_images.append(img)
                    else:
                        self.get_logger().warn(
                            f'Nie można wczytać obrazu: {img_path}. '
                            'Sprawdź, czy plik nie jest uszkodzony.'
                        )
        else:
            self.get_logger().warn(
                f'Katalog obrazów testowych nie istnieje: {path}. '
                'Ustaw poprawną ścieżkę przez parametr test_images_path '
                'lub utwórz katalog data/test_images/ z obrazami PNG/JPG.'
            )
        if not self._demo_images:
            # Jeśli brak obrazów, utwórz czarny obraz zastępczy
            blank = self._create_blank_image()
            self._demo_images.append(blank)
            self.get_logger().warn(
                'Brak obrazów testowych – używam czarnego obrazu zastępczego. '
                'Dodaj obrazy PNG/JPG do katalogu data/test_images/.'
            )

    def _create_blank_image(self):
        """Tworzy czarny obraz zastępczy 640x480."""
        import numpy as np
        return np.zeros((480, 640, 3), dtype='uint8')

    def _init_robot_cameras(self):
        """Inicjalizuje kamery sprzętowe robota przez OpenCV VideoCapture.

        Otwiera urządzenia V4L2 dla kamery ciała i kamery głowy.
        Indeksy urządzeń konfigurowane są przez parametry ROS2:
            body_camera_device (domyślnie 0)
            head_camera_device (domyślnie 1)

        Jeśli urządzenie kamery jest niedostępne, węzeł używa czarnego
        obrazu zastępczego i loguje ostrzeżenie.
        """
        self._body_cap = self._open_capture(self._body_camera_device, 'ciała')
        self._head_cap = self._open_capture(self._head_camera_device, 'głowy')

        if self._body_cap is None and self._head_cap is None:
            self.get_logger().warn(
                'Żadna kamera sprzętowa nie jest dostępna. '
                'Używam czarnego obrazu zastępczego. '
                'Sprawdź indeksy urządzeń (body_camera_device, head_camera_device) '
                'i uprawnienia do /dev/video*.'
            )
            self._demo_images = [self._create_blank_image()]

    def _open_capture(self, device_index: int, camera_name: str):
        """Otwiera pojedyncze urządzenie kamerowe przez OpenCV VideoCapture.

        Args:
            device_index: Indeks urządzenia V4L2 (np. 0 dla /dev/video0).
            camera_name: Nazwa kamery do logów (np. 'ciała').

        Returns:
            cv2.VideoCapture jeśli urządzenie jest dostępne, None w przeciwnym razie.
        """
        cap = cv2.VideoCapture(device_index)
        if cap.isOpened():
            self.get_logger().info(
                f'Kamera {camera_name} otwarta pomyślnie '
                f'(urządzenie /dev/video{device_index}).'
            )
            return cap
        cap.release()
        self.get_logger().warn(
            f'Nie można otworzyć kamery {camera_name} '
            f'(urządzenie /dev/video{device_index}). '
            'Sprawdź, czy urządzenie istnieje i ma odpowiednie uprawnienia.'
        )
        return None

    def _publish_images(self):
        """Publikuje aktualny obraz na tematy kamer.

        W trybie demo: publikuje kolejne obrazy z listy _demo_images cyklicznie.
        W trybie robot: przechwytuje klatki z otwartych urządzeń VideoCapture.
        """
        stamp = self.get_clock().now().to_msg()

        if self._mode == 'robot_mode' and (
            self._body_cap is not None or self._head_cap is not None
        ):
            body_img = self._read_capture(self._body_cap, 'ciała')
            head_img = self._read_capture(self._head_cap, 'głowy')
        else:
            # Tryb demo lub brak kamer sprzętowych – użyj obrazów z listy
            if not self._demo_images:
                return
            img = self._demo_images[self._demo_index % len(self._demo_images)]
            self._demo_index += 1
            body_img = img
            head_img = img

        # Publikacja na temat kamery ciała
        body_msg = self._bridge.cv2_to_imgmsg(body_img, encoding='bgr8')
        body_msg.header.stamp = stamp
        body_msg.header.frame_id = 'body_camera_frame'
        self._pub_body.publish(body_msg)

        # Publikacja na temat kamery głowy
        head_msg = self._bridge.cv2_to_imgmsg(head_img, encoding='bgr8')
        head_msg.header.stamp = stamp
        head_msg.header.frame_id = 'head_camera_frame'
        self._pub_head.publish(head_msg)

    def _read_capture(self, cap, camera_name: str):
        """Odczytuje jedną klatkę z urządzenia VideoCapture.

        Zwraca czarny obraz zastępczy jeśli kamera jest niedostępna
        lub odczyt się nie powiedzie.

        Args:
            cap: cv2.VideoCapture lub None.
            camera_name: Nazwa kamery do logów.

        Returns:
            Obraz BGR jako numpy array.
        """
        if cap is not None and cap.isOpened():
            ret, frame = cap.read()
            if ret:
                return frame
            self.get_logger().warn(
                f'Błąd odczytu klatki z kamery {camera_name}. '
                'Używam czarnego obrazu zastępczego.'
            )
        return self._create_blank_image()

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
    node = CameraInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
