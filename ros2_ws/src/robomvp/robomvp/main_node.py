#!/usr/bin/env python3
"""Główny węzeł RoboMVP.

Orkiestruje cały pipeline demonstracyjny:
- wczytuje konfigurację sceny z pliku ``scene.yaml``,
- inicjalizuje automat stanowy (``StateMachine``),
- w trybie robot nawiązuje połączenie z robotem przez ``UnitreeRobotAPI``,
- subskrybuje tematy percepcji (poza markera, offset korekcji),
- na każdym kroku timera wywołuje krok automatu stanowego i uruchamia
  odpowiednie sekwencje ruchów z modułu ``motion_sequences``,
- publikuje aktualny stan automatu i komendy ruchu.

Subskrybowane tematy:
    /robomvp/marker_pose  – poza 3D markera (robomvp/MarkerPose)
    /robomvp/offset       – offset korekcji pozycji (robomvp/Offset)

Publikowane tematy:
    /robomvp/state          – aktualny stan automatu (robomvp/State)
    /robomvp/motion_command – nazwa wykonywanej sekwencji (std_msgs/String)

Parametry ROS2:
    scene_config_path  – ścieżka do pliku ``scene.yaml``
    mode               – tryb pracy: ``demo_mode`` lub ``robot_mode``
    step_period        – okres kroku automatu [s], domyślnie 1.0
    network_interface  – interfejs Ethernet do robota, domyślnie ``eth0``
"""

from pathlib import Path

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
from robomvp.state_machine import State, StateMachine
from robomvp.unitree_robot_api import UnitreeRobotAPI


class RoboMVPMain(Node):
    """Węzeł główny systemu RoboMVP."""

    def __init__(self):
        super().__init__('robomvp_main')

        self.declare_parameter('scene_config_path', '')
        self.declare_parameter('step_period', 1.0)
        self.declare_parameter('network_interface', 'eth0')
        self.declare_parameter('require_robot_connection', False)

        scene_config_path = self.get_parameter('scene_config_path').value
        step_period = float(self.get_parameter('step_period').value)
        network_interface = str(self.get_parameter('network_interface').value)
        self._require_robot_connection = bool(
            self.get_parameter('require_robot_connection').value
        )

        self._config = self._load_config(scene_config_path)

        self._robot_api = UnitreeRobotAPI(network_interface=network_interface)
        self._robot_connected = False
        try:
            self._robot_api.connect(logger=self.get_logger())
            self._robot_connected = True
            self.get_logger().info('Połączenie z robotem aktywne - ruch będzie wykonywany na sprzęcie.')
        except RuntimeError as exc:
            if self._require_robot_connection:
                self.get_logger().error(
                    f'Nie można połączyć z robotem: {exc}. '
                    'Parametr require_robot_connection=True wymaga aktywnego połączenia.'
                )
                raise
            self.get_logger().warn(
                f'Nie można połączyć z robotem: {exc}. '
                'Kontynuuję bez połączenia sprzętowego (symulacja komend ruchu w logach).'
            )
            self._robot_api = None

        offset_scale = self._config.get('offset_scale', {})
        self._scale_dx = float(offset_scale.get('dx', 1.0))
        self._scale_dy = float(offset_scale.get('dy', 1.0))
        self._scale_dz = float(offset_scale.get('dz', 1.0))
        self._state_machine = StateMachine(self._config, logger=self.get_logger())

        motion_timeouts = self._config.get('motion_timeouts', {})
        self._motion_total_timeout_s = float(motion_timeouts.get('total', 30.0))
        self._motion_step_timeout_s = float(motion_timeouts.get('step', 5.0))

        self._sub_pose = self.create_subscription(
            MarkerPose, '/robomvp/marker_pose', self._on_marker_pose, 10
        )
        self._sub_offset = self.create_subscription(
            Offset, '/robomvp/offset', self._on_offset, 10
        )

        self._pub_state = self.create_publisher(StateMsg, '/robomvp/state', 10)
        self._pub_motion = self.create_publisher(String, '/robomvp/motion_command', 10)

        self._timer = self.create_timer(step_period, self._step)
        self._current_offset = (0.0, 0.0, 0.0)

        self.get_logger().info(
            'Węzeł główny RoboMVP uruchomiony. '
            f'Połączenie z robotem: {"aktywne" if self._robot_connected else "brak"}. '
            'Oczekiwanie na dane z czujników.'
        )

    def _load_config(self, config_path: str) -> dict:
        """Wczytuje konfigurację sceny z pliku YAML."""
        if not config_path:
            current = Path(__file__).resolve()
            possible_paths = [
                *[parent / 'config' / 'scene.yaml' for parent in current.parents],
                Path('/opt/ros2_ws/config/scene.yaml'),
            ]
            for path in possible_paths:
                if path.is_file():
                    config_path = str(path)
                    break

        if config_path and Path(config_path).is_file():
            try:
                with open(config_path, 'r') as f:
                    cfg = yaml.safe_load(f)
                self.get_logger().info(f'Załadowano konfigurację sceny: {config_path}')
                return cfg or {}
            except Exception as e:
                self.get_logger().warn(
                    f'Błąd wczytywania konfiguracji: {e}. '
                    'Sprawdź poprawność pliku YAML i uprawnienia do odczytu. '
                    'Używam domyślnych wartości konfiguracji.'
                )

        self.get_logger().warn(
            'Nie znaleziono pliku konfiguracyjnego sceny. Używam domyślnej konfiguracji. '
            'Podaj ścieżkę przez parametr ROS2: --ros-args -p scene_config_path:=<ścieżka>'
        )
        return {
            'box_marker_id': 10,
            'table_markers': {'pickup_table': 21, 'place_table': 22},
            'target_marker': 30,
            'stop_distance_threshold': 0.3,
            'alignment_threshold': 0.05,
            'offset_scale': {'dx': 1.0, 'dy': 1.0, 'dz': 1.0},
            'state_timeouts': {
                'search_table': 20.0,
                'detect_marker': 20.0,
                'align_with_box': 10.0,
                'navigate_to_target_marker': 25.0,
            },
            'motion_timeouts': {'total': 30.0, 'step': 5.0},
        }

    def _on_marker_pose(self, msg: MarkerPose):
        """Odbiera pozę markera i aktualizuje automat stanowy."""
        self.get_logger().debug(
            f'Odebrano marker_pose: id={msg.marker_id}, x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}'
        )
        self._state_machine.update_marker(msg.marker_id, msg.x, msg.y, msg.z)

    def _on_offset(self, msg: Offset):
        """Odbiera offset korekcji i aktualizuje automat stanowy."""
        dx = msg.dx * self._scale_dx
        dy = msg.dy * self._scale_dy
        dz = msg.dz * self._scale_dz
        self._current_offset = (dx, dy, dz)
        self.get_logger().debug(
            f'Odebrano offset: dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f}'
        )
        self._state_machine.update_offset(dx, dy, dz)

    def _step(self):
        """Wykonuje jeden krok automatu stanowego i reaguje na zmiany stanu."""
        previous_state = self._state_machine.current_state
        new_state = self._state_machine.step()

        state_msg = StateMsg()
        state_msg.state_name = self._state_machine.current_state_name
        state_msg.state_id = int(new_state)
        self._pub_state.publish(state_msg)

        if new_state != previous_state:
            self.get_logger().info(
                f'Wykryto zmianę stanu: {previous_state.name} -> {new_state.name}. '
                'Uruchamiam akcję dla nowego stanu.'
            )
            self._execute_state_action(new_state)

        if new_state == State.FINISHED:
            self.get_logger().info(
                'Scenariusz zakończony pomyślnie. Zatrzymuję timer automatu.'
            )
            self._timer.cancel()

    def _execute_state_action(self, state: State):
        """Wykonuje akcję odpowiednią dla nowego stanu."""
        dx, dy, dz = self._current_offset

        if state == State.DETECT_MARKER:
            self.get_logger().info('Akcja: approach_table (podejście do stołu).')
            self._publish_motion('approach_table')
            sequence = apply_offset_to_sequence(get_approach_table(), dx, dy, dz)
            self._run_sequence(sequence, 'approach_table')

        elif state == State.ALIGN_WITH_BOX:
            self.get_logger().info('Akcja: align_with_box (wyrównanie do pudełka).')
            self._publish_motion('align_with_box')

        elif state == State.PICK_BOX:
            self.get_logger().info('Akcja: pick_box (podniesienie pudełka).')
            self._publish_motion('pick_box')
            sequence = apply_offset_to_sequence(get_pick_box(), dx, dy, dz)
            self._run_sequence(sequence, 'pick_box')

        elif state == State.ROTATE_180:
            self.get_logger().info('Akcja: rotate_180 (obrót robota).')
            self._publish_motion('rotate_180')
            sequence = get_rotate_180()
            self._run_sequence(sequence, 'rotate_180')

        elif state == State.NAVIGATE_TO_TARGET_MARKER:
            self.get_logger().info('Akcja: walk_to_second_table (nawigacja do celu).')
            self._publish_motion('walk_to_second_table')
            sequence = get_walk_to_second_table()
            self._run_sequence(sequence, 'walk_to_second_table')

        elif state == State.PLACE_BOX:
            self.get_logger().info('Akcja: place_box (odłożenie pudełka).')
            self._publish_motion('place_box')
            sequence = apply_offset_to_sequence(get_place_box(), dx, dy, dz)
            self._run_sequence(sequence, 'place_box')

        elif state == State.FINISHED:
            self.get_logger().info('Akcja: finished (publikacja zakończenia).')
            self._publish_motion('finished')

    def _run_sequence(self, sequence: list, sequence_name: str):
        """Uruchamia sekwencję ruchu z timeoutami i obsługą błędów."""
        self.get_logger().info(
            f'Start sekwencji: {sequence_name} (kroków: {len(sequence)}). '
            f'Tryb wykonania: {"robot" if self._robot_api is not None else "bez robota"}.'
        )
        ok = execute_sequence(
            sequence,
            robot_api=self._robot_api,
            logger=self.get_logger(),
            total_timeout_s=self._motion_total_timeout_s,
            step_timeout_s=self._motion_step_timeout_s,
        )
        if not ok:
            self.get_logger().error(
                f'Sekwencja {sequence_name} zakończyła się błędem/timeoutem - zatrzymuję timer'
            )
            self._timer.cancel()
        else:
            self.get_logger().info(f'Sekwencja {sequence_name} zakończona sukcesem.')

    def _publish_motion(self, command: str):
        """Publikuje komendę ruchu na temat /robomvp/motion_command."""
        msg = String()
        msg.data = command
        self._pub_motion.publish(msg)
        self.get_logger().info(f'Wysłano komendę ruchu: {command}')

    def destroy_node(self):
        """Rozłącza robota i niszczy węzeł."""
        if self._robot_api is not None:
            self._robot_api.disconnect()
            self._robot_api = None
        super().destroy_node()


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
