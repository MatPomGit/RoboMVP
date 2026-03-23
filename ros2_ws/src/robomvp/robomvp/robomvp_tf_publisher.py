#!/usr/bin/env python3
"""Węzeł publikujący statyczne transformacje TF dla kamer robota.

CZYM JEST DRZEWO TF2 I PO CO NAM TRANSFORMACJE?
================================================
TF2 (Transform Library 2) to biblioteka ROS2 do zarządzania układami
współrzędnych (frame). Każdy sensor, kończyna i fragment robota ma swój
własny układ współrzędnych, a TF2 przechowuje jak jeden układ ma się do drugiego.

Przykładowe drzewo TF dla RoboMVP:
    odom
      └─ base_link          (środek robota, środek masy)
           ├─ body_camera_frame  (kamera ciała, ~0.2 m przed base_link, ~1.0 m w górę)
           └─ head_camera_frame  (kamera głowy, ~0.0 m przed base_link, ~1.5 m w górę)

Dlaczego to ważne?
Węzeł marker_pose_estimator oblicza pozycję markera w układzie KAMERY
(x = prawo, y = dół, z = do przodu). Ale automat stanowy chce wiedzieć
gdzie marker jest względem BASE_LINK robota (żeby wiedzieć jak się ruszyć).
TF2 umożliwia tę konwersję jednym wywołaniem tf_buffer.transform().

TRANSFORMACJE STATYCZNE vs DYNAMICZNE:
StaticTransformBroadcaster wysyła transformację raz i latczy ją jako stałą.
Kamera jest przymocowana do robota – jej pozycja względem base_link NIE zmienia
się w czasie. Używamy static, nie dynamic (który wymagałby publikowania co klatkę).

KONWENCJA UKŁADU KAMERY (REP-102):
W ROS2 kamera ma własną konwencję osi (Camera Frame Convention):
    x → prawo
    y → dół
    z → do przodu (głębokość)
To jest INNE niż standardowy układ ROS (REP-103): x=przód, y=lewo, z=góra.
Transformacja TF uwzględnia ten obrót.

Publikowane transformacje TF:
    base_link → body_camera_frame  (statyczna)
    base_link → head_camera_frame  (statyczna)
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class TFPublisher(Node):
    """Węzeł publikujący statyczne transformacje TF dla kamer robota G1 EDU."""

    def __init__(self):
        super().__init__('robomvp_tf_publisher')

        # Parametry pozycji kamer – dostosuj do fizycznej konfiguracji G1 EDU.
        # Wartości domyślne są przybliżone dla standardowego G1 EDU.
        self.declare_parameter('body_camera_x', 0.20)   # 20 cm przed base_link
        self.declare_parameter('body_camera_y', 0.00)
        self.declare_parameter('body_camera_z', 1.00)   # 1 m nad podłożem

        self.declare_parameter('head_camera_x', 0.00)
        self.declare_parameter('head_camera_y', 0.00)
        self.declare_parameter('head_camera_z', 1.50)   # 1.5 m – wysokość głowy

        # Nachylenie kamery głowy w dół (pitch) – typowo ~10-15 stopni
        self.declare_parameter('head_camera_pitch_deg', 10.0)

        self._broadcaster = StaticTransformBroadcaster(self)
        self._publish_transforms()

        self.get_logger().info(
            'Statyczne transformacje TF opublikowane: '
            'base_link → body_camera_frame, base_link → head_camera_frame'
        )

    def _publish_transforms(self):
        """Buduje i publikuje obie statyczne transformacje TF."""
        transforms = [
            self._build_body_camera_tf(),
            self._build_head_camera_tf(),
        ]
        self._broadcaster.sendTransform(transforms)

    def _build_body_camera_tf(self) -> TransformStamped:
        """Transformacja base_link → body_camera_frame.

        Kamera ciała patrzy do przodu, zamontowana na wysokości ~1 m.
        Obrót: kamera jest obrócona o -90° wokół X i +90° wokół Z, żeby
        zmapować optyczną oś Z (do przodu) na standardową oś X (przód robota).
        """
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id  = 'body_camera_frame'

        t.transform.translation.x = self.get_parameter('body_camera_x').value
        t.transform.translation.y = self.get_parameter('body_camera_y').value
        t.transform.translation.z = self.get_parameter('body_camera_z').value

        # Kwaternion dla konwersji układu kamery optycznej → układ ROS:
        # Obrót: roll=−90°, pitch=0°, yaw=−90°
        # Ta konwencja jest standardowa dla kamer w ROS (REP-102 → REP-103).
        quat = self._rpy_to_quaternion(-math.pi / 2, 0.0, -math.pi / 2)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        return t

    def _build_head_camera_tf(self) -> TransformStamped:
        """Transformacja base_link → head_camera_frame.

        Kamera głowy jest zamontowana wyżej i lekko nachylona w dół
        (parametr head_camera_pitch_deg), żeby widzieć podłogę przed robotem.
        """
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id  = 'head_camera_frame'

        t.transform.translation.x = self.get_parameter('head_camera_x').value
        t.transform.translation.y = self.get_parameter('head_camera_y').value
        t.transform.translation.z = self.get_parameter('head_camera_z').value

        pitch_deg = self.get_parameter('head_camera_pitch_deg').value
        pitch_rad = math.radians(pitch_deg)

        # Kamera głowy: obrót kamery optycznej → układ ROS + nachylenie pitch
        quat = self._rpy_to_quaternion(-math.pi / 2, pitch_rad, -math.pi / 2)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        return t

    @staticmethod
    def _rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple:
        """Konwertuje kąty RPY (roll-pitch-yaw) na kwaternion.

        Konwencja ZYX (yaw najpierw, roll ostatni) – standard w robotyce.
        Wzory:
            cr = cos(roll/2),  sr = sin(roll/2)
            cp = cos(pitch/2), sp = sin(pitch/2)
            cy = cos(yaw/2),   sy = sin(yaw/2)

            w = cr*cp*cy + sr*sp*sy
            x = sr*cp*cy - cr*sp*sy
            y = cr*sp*cy + sr*cp*sy
            z = cr*cp*sy - sr*sp*cy
        """
        cr, sr = math.cos(roll / 2),  math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        cy, sy = math.cos(yaw / 2),   math.sin(yaw / 2)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return (x, y, z, w)


def main(args=None):
    rclpy.init(args=args)
    node = TFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
