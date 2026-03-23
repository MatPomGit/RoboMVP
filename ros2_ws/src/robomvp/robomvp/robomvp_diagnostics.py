#!/usr/bin/env python3
"""Węzeł diagnostyczny systemu RoboMVP.

CO TO SĄ DIAGNOSTYKI W ROS2?
==============================
diagnostic_msgs/DiagnosticArray to standardowy mechanizm ROS2 do raportowania
„zdrowia" komponentów systemu. Każdy komponent (kamera, połączenie z robotem,
automat stanowy) publikuje swój status jako DiagnosticStatus z jednym z poziomów:

    OK    (0) – komponent działa poprawnie
    WARN  (1) – coś niepokojącego, ale system działa
    ERROR (2) – poważny błąd, system może nie działać
    STALE (3) – brak danych (komponent przestał raportować)

Narzędzie `rqt_robot_monitor` (albo `ros2 topic echo /diagnostics`) pokazuje
te statusy w czytelnej formie – operator widzi jednym rzutem oka czy wszystko
działa, zamiast śledzić dziesiątki tematów.

DLACZEGO WARTO TO MIEĆ W MVP?
Podczas demonstracji w laboratorium coś zawsze idzie nie tak – kamera się
odłącza, bateria robota siada, SDK traci połączenie. Bez diagnostyk operator
musi analizować logi terminala. Z diagnostykami widzi czerwone pole od razu.

Publikowane tematy:
    /diagnostics  – DiagnosticArray co 1 sekundę (standard ROS2)
"""

from datetime import datetime

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from std_msgs.msg import String

from robomvp.msg import State as StateMsg


class RoboMVPDiagnostics(Node):
    """Węzeł zbierający i publikujący diagnostyki całego systemu RoboMVP."""

    # Czas (sekundy) po którym brak wiadomości uznajemy za STALE
    _STALE_THRESHOLD = 3.0

    def __init__(self):
        super().__init__('robomvp_diagnostics')

        self.declare_parameter('publish_rate', 1.0)
        rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self._pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Śledzenie ostatnich wiadomości z każdego monitowanego tematu
        self._last_state_msg: StateMsg | None = None
        self._last_state_time: float = 0.0
        self._last_motion_cmd: str = 'brak'
        self._last_motion_time: float = 0.0
        self._last_marker_time: float = 0.0  # czas ostatniego wykrycia markera

        # Subskrypcje do tematów, których zdrowie monitorujemy
        self.create_subscription(StateMsg, '/robomvp/state', self._on_state, 10)
        self.create_subscription(String, '/robomvp/motion_command', self._on_motion, 10)

        # Używamy wbudowanego timera ROS2 – diagnostyki co `rate` Hz (domyślnie 1 Hz)
        self.create_timer(1.0 / rate, self._publish_diagnostics)
        self.get_logger().info('Węzeł diagnostyczny uruchomiony.')

    # ------------------------------------------------------------------
    # Callbacki subskrypcji
    # ------------------------------------------------------------------

    def _on_state(self, msg: StateMsg):
        self._last_state_msg = msg
        self._last_state_time = self.get_clock().now().nanoseconds * 1e-9

    def _on_motion(self, msg: String):
        self._last_motion_cmd = msg.data
        self._last_motion_time = self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------
    # Publikowanie diagnostyk
    # ------------------------------------------------------------------

    def _publish_diagnostics(self):
        """Buduje i publikuje DiagnosticArray ze statusem każdego komponentu."""
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        stamp = self.get_clock().now().to_msg()

        array = DiagnosticArray()
        array.header.stamp = stamp

        array.status.append(self._check_state_machine(now_sec))
        array.status.append(self._check_motion_pipeline(now_sec))
        array.status.append(self._check_system_time())

        self._pub.publish(array)

    def _check_state_machine(self, now_sec: float) -> DiagnosticStatus:
        """Sprawdza czy automat stanowy publikuje stan i czy nie utkwił."""
        status = DiagnosticStatus()
        status.hardware_id = 'robomvp_state_machine'
        status.name = 'RoboMVP / Automat stanowy'

        if self._last_state_time == 0.0:
            status.level = DiagnosticStatus.WARN
            status.message = 'Brak wiadomości /robomvp/state – węzeł main jeszcze nie startował?'
            return status

        age = now_sec - self._last_state_time
        if age > self._STALE_THRESHOLD:
            status.level = DiagnosticStatus.STALE
            status.message = f'Brak wiadomości od {age:.1f}s (próg: {self._STALE_THRESHOLD}s)'
            return status

        state_name = self._last_state_msg.state_name if self._last_state_msg else 'nieznany'
        state_id   = self._last_state_msg.state_id   if self._last_state_msg else -1

        if state_name == 'FINISHED':
            status.level = DiagnosticStatus.OK
            status.message = 'Scenariusz zakończony pomyślnie'
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'Aktywny: {state_name}'

        status.values = [
            KeyValue(key='stan', value=state_name),
            KeyValue(key='id_stanu', value=str(state_id)),
            KeyValue(key='wiek_wiadomosci_s', value=f'{age:.2f}'),
        ]
        return status

    def _check_motion_pipeline(self, now_sec: float) -> DiagnosticStatus:
        """Sprawdza czy pipeline ruchu wysyła komendy."""
        status = DiagnosticStatus()
        status.hardware_id = 'robomvp_motion'
        status.name = 'RoboMVP / Pipeline ruchu'

        if self._last_motion_time == 0.0:
            status.level = DiagnosticStatus.OK
            status.message = 'Oczekiwanie na pierwszą komendę ruchu'
            return status

        age = now_sec - self._last_motion_time
        status.level = DiagnosticStatus.OK
        status.message = f'Ostatnia komenda: {self._last_motion_cmd}'
        status.values = [
            KeyValue(key='ostatnia_komenda', value=self._last_motion_cmd),
            KeyValue(key='wiek_s', value=f'{age:.1f}'),
        ]
        return status

    def _check_system_time(self) -> DiagnosticStatus:
        """Publikuje informację o czasie systemowym (przydatne przy synchronizacji NTP)."""
        status = DiagnosticStatus()
        status.hardware_id = 'host'
        status.name = 'RoboMVP / Czas systemowy'
        status.level = DiagnosticStatus.OK
        status.message = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        return status


def main(args=None):
    rclpy.init(args=args)
    node = RoboMVPDiagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
