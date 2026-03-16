#!/usr/bin/env python3
"""Interfejs kamer dla systemu RoboMVP.

Obsługuje dwie kamery robota:
- kamera ciała (manipulacja, bliskie markery)
- kamera głowy (nawigacja, dalekie markery)

W trybie demo wczytuje obrazy z katalogu data/test_images/.
W trybie robot używa prawdziwych kamer sprzętowych.
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

        self._mode = self.get_parameter('mode').get_parameter_value().string_value
        self._publish_rate = (
            self.get_parameter('publish_rate').get_parameter_value().double_value
        )
        self._test_images_path = (
            self.get_parameter('test_images_path').get_parameter_value().string_value
        )

        self._bridge = CvBridge()

        # Wydawcy tematów kamer
        self._pub_body = self.create_publisher(Image, '/camera/body/image_raw', 10)
        self._pub_head = self.create_publisher(Image, '/camera/head/image_raw', 10)

        # Timer publikowania obrazów
        self._timer = self.create_timer(1.0 / self._publish_rate, self._publish_images)

        # Indeks aktualnego obrazu testowego
        self._demo_index = 0
        self._demo_images = []

        if self._mode == 'demo_mode':
            self._load_demo_images()
            self.get_logger().info(
                f'Tryb demo: załadowano {len(self._demo_images)} obrazów testowych'
            )
        else:
            self._init_robot_cameras()
            self.get_logger().info('Tryb robot: inicjalizacja kamer sprzętowych')

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
        if not self._demo_images:
            # Jeśli brak obrazów, utwórz czarny obraz zastępczy
            blank = self._create_blank_image()
            self._demo_images.append(blank)

    def _create_blank_image(self):
        """Tworzy czarny obraz zastępczy 640x480."""
        import numpy as np
        return np.zeros((480, 640, 3), dtype='uint8')

    def _init_robot_cameras(self):
        """Inicjalizuje kamery sprzętowe robota (stub dla Unitree SDK)."""
        # TODO: Integracja z Unitree SDK dla prawdziwych kamer
        self.get_logger().warn(
            'Inicjalizacja kamer sprzętowych nie jest jeszcze zaimplementowana. '
            'Używam zastępczego obrazu.'
        )
        self._demo_images = [self._create_blank_image()]

    def _publish_images(self):
        """Publikuje aktualny obraz na tematy kamer."""
        if not self._demo_images:
            return

        img = self._demo_images[self._demo_index % len(self._demo_images)]
        self._demo_index += 1

        stamp = self.get_clock().now().to_msg()

        # Publikacja na temat kamery ciała
        body_msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
        body_msg.header.stamp = stamp
        body_msg.header.frame_id = 'body_camera_frame'
        self._pub_body.publish(body_msg)

        # Publikacja na temat kamery głowy
        head_msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
        head_msg.header.stamp = stamp
        head_msg.header.frame_id = 'head_camera_frame'
        self._pub_head.publish(head_msg)


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
