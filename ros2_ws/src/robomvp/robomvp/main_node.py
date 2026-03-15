#!/usr/bin/env python3
"""Główny węzeł RoboMVP.

Ładuje konfigurację, uruchamia automat stanowy,
subskrybuje tematy percepcji i wykonuje sekwencje ruchów.
Orkiestruje cały pipeline demonstracyjny.
"""

import os

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String

from robomvp.motion_sequences import (
    apply_offset_to_sequence,
    execute_sequence,
    get_approach_table,
    get_pick_box,
    get_place_box,
    get_rotate_180,
    get_walk_to_second_table,
)
from robomvp.msg import MarkerPose, Offset
from robomvp.msg import State as StateMsg
from robomvp.offset_corrector import OffsetCorrector
from robomvp.state_machine import State, StateMachine


class RoboMVPMain(Node):
    """Węzeł główny systemu RoboMVP.

    Koordynuje wszystkie komponenty:
    - Automat stanowy
    - Korektor offsetu
    - Sekwencje ruchów
    - Komunikacja ROS2
    """

    def __init__(self):
        super().__init__('robomvp_main')

        # Deklaracja parametrów
        self.declare_parameter('scene_config_path', '')
        self.declare_parameter('mode', 'demo_mode')
        self.declare_parameter('step_period', 1.0)

        scene_config_path = (
            self.get_parameter('scene_config_path')
            .get_parameter_value()
            .string_value
        )
        self._mode = self.get_parameter('mode').get_parameter_value().string_value
        step_period = (
            self.get_parameter('step_period').get_parameter_value().double_value
        )

        # Wczytanie konfiguracji sceny
        self._config = self._load_config(scene_config_path)

        # Inicjalizacja komponentów
        offset_scale = self._config.get('offset_scale', {})
        self._offset_corrector = OffsetCorrector(
            scale_dx=offset_scale.get('dx', 1.0),
            scale_dy=offset_scale.get('dy', 1.0),
            scale_dz=offset_scale.get('dz', 1.0),
        )
        self._state_machine = StateMachine(self._config, logger=self.get_logger())

        # Subskrypcje: poza markera i offset korekcji
        self._sub_pose = self.create_subscription(
            MarkerPose, '/robomvp/marker_pose', self._on_marker_pose, 10
        )
        self._sub_offset = self.create_subscription(
            Offset, '/robomvp/offset', self._on_offset, 10
        )

        # Wydawcy: stan automatu i komendy ruchu
        self._pub_state = self.create_publisher(StateMsg, '/robomvp/state', 10)
        self._pub_motion = self.create_publisher(String, '/robomvp/motion_command', 10)

        # Timer kroku automatu stanowego
        self._timer = self.create_timer(step_period, self._step)

        # Aktualny offset korekcji
        self._current_offset = (0.0, 0.0, 0.0)

        self.get_logger().info(
            f'Węzeł główny RoboMVP uruchomiony (tryb: {self._mode})'
        )

    def _load_config(self, config_path: str) -> dict:
        """Wczytuje konfigurację sceny z pliku YAML."""
        if not config_path:
            # Szukaj pliku konfiguracji względem pakietu
            possible_paths = [
                os.path.join(
                    os.path.dirname(os.path.abspath(__file__)),
                    '..', '..', '..', '..', '..', '..', 'config', 'scene.yaml'
                ),
                '/opt/ros2_ws/config/scene.yaml',
            ]
            for p in possible_paths:
                real_p = os.path.realpath(p)
                if os.path.isfile(real_p):
                    config_path = real_p
                    break

        if config_path and os.path.isfile(config_path):
            try:
                with open(config_path, 'r') as f:
                    cfg = yaml.safe_load(f)
                self.get_logger().info(f'Załadowano konfigurację: {config_path}')
                return cfg or {}
            except Exception as e:
                self.get_logger().warn(f'Błąd wczytywania konfiguracji: {e}')

        self.get_logger().warn('Używam domyślnej konfiguracji sceny')
        return {
            'box_marker_id': 10,
            'table_markers': {'pickup_table': 21, 'place_table': 22},
            'target_marker': 30,
            'stop_distance_threshold': 0.3,
            'alignment_threshold': 0.05,
            'offset_scale': {'dx': 1.0, 'dy': 1.0, 'dz': 1.0},
        }

    def _on_marker_pose(self, msg: MarkerPose):
        """Odbiera pozę markera i aktualizuje automat stanowy."""
        self._state_machine.update_marker(msg.marker_id, msg.x, msg.y, msg.z)

    def _on_offset(self, msg: Offset):
        """Odbiera offset korekcji i aktualizuje automat stanowy."""
        self._current_offset = (msg.dx, msg.dy, msg.dz)
        self._state_machine.update_offset(msg.dx, msg.dy, msg.dz)

    def _step(self):
        """Wykonuje jeden krok automatu stanowego i reaguje na zmiany stanu."""
        previous_state = self._state_machine.current_state
        new_state = self._state_machine.step()

        # Publikuj aktualny stan
        state_msg = StateMsg()
        state_msg.state_name = self._state_machine.current_state_name
        state_msg.state_id = int(new_state)
        self._pub_state.publish(state_msg)

        # Wykonaj akcję jeśli nastąpiło przejście stanu
        if new_state != previous_state:
            self._execute_state_action(new_state)

        # Sprawdź zakończenie
        if new_state == State.FINISHED:
            self.get_logger().info('Scenariusz demonstracyjny zakończony pomyślnie!')
            self._timer.cancel()

    def _execute_state_action(self, state: State):
        """Wykonuje akcję odpowiednią dla nowego stanu."""
        dx, dy, dz = self._current_offset

        if state == State.DETECT_MARKER:
            self._publish_motion('approach_table')
            sequence = apply_offset_to_sequence(get_approach_table(), dx, dy, dz)
            execute_sequence(sequence, robot_api=None, logger=self.get_logger())

        elif state == State.ALIGN_WITH_BOX:
            self._publish_motion('align_with_box')

        elif state == State.PICK_BOX:
            self._publish_motion('pick_box')
            sequence = apply_offset_to_sequence(get_pick_box(), dx, dy, dz)
            execute_sequence(sequence, robot_api=None, logger=self.get_logger())

        elif state == State.ROTATE_180:
            self._publish_motion('rotate_180')
            sequence = get_rotate_180()
            execute_sequence(sequence, robot_api=None, logger=self.get_logger())

        elif state == State.NAVIGATE_TO_TARGET_MARKER:
            self._publish_motion('walk_to_second_table')
            sequence = get_walk_to_second_table()
            execute_sequence(sequence, robot_api=None, logger=self.get_logger())

        elif state == State.PLACE_BOX:
            self._publish_motion('place_box')
            sequence = apply_offset_to_sequence(get_place_box(), dx, dy, dz)
            execute_sequence(sequence, robot_api=None, logger=self.get_logger())

        elif state == State.FINISHED:
            self._publish_motion('finished')

    def _publish_motion(self, command: str):
        """Publikuje komendę ruchu na temat /robomvp/motion_command."""
        msg = String()
        msg.data = command
        self._pub_motion.publish(msg)
        self.get_logger().info(f'Komenda ruchu: {command}')


def main(args=None):
    """Punkt wejścia węzła robomvp_main."""
    rclpy.init(args=args)
    node = RoboMVPMain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
