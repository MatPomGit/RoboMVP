#!/usr/bin/env python3
"""Główny węzeł RoboMVP – orkiestrator pipeline z pełną komunikacją ROS2.

KOMPLETNA MAPA KOMUNIKACJI ROS2 TEGO WĘZŁA:
============================================

Subskrypcje (dane wejściowe):
    /robomvp/marker_pose  [robomvp/MarkerPose]   – pozycja 3D markera z kamery
    /robomvp/offset       [robomvp/Offset]        – korekcja wyrównania

Publikacje (dane wyjściowe):
    /robomvp/state          [robomvp/State]        – bieżący stan automatu
    /robomvp/motion_command [std_msgs/String]       – nazwa aktywnej sekwencji

Serwisy (komendy synchroniczne od operatora):
    /robomvp/pause          [std_srvs/SetBool]     – wstrzymaj/wznów scenariusz
    /robomvp/emergency_stop [std_srvs/Trigger]     – natychmiastowe zatrzymanie
    /robomvp/reset          [std_srvs/Trigger]     – reset do stanu SEARCH_TABLE

Action Server (długotrwałe zadania z feedbackiem):
    /robomvp/manipulation_task [robomvp/ManipulationTask] – wykonaj sekwencję

DLACZEGO SERWISY + ACTION SERVER, NIE TYLKO TOPICI?
=====================================================
Topic (pub/sub) jest dobry do ciągłych strumieni danych (obraz, stan, czujniki).
Ale dla komend od operatora potrzebujemy potwierdzenia „komenda odebrana i wykonana".

Serwis daje potwierdzenie synchronicznie – operator wywołuje
`ros2 service call /robomvp/pause std_srvs/SetBool '{data: true}'`
i dostaje odpowiedź od razu, czy pauza zadziałała. Idealny dla krótkich akcji.

Action Server jest jak serwis ale dla długotrwałych zadań. Klient wysyła Goal
(„wykonaj pick_box"), serwer akceptuje i wysyła Feedback co krok
(„krok 2/4, postęp 50%"), a na końcu Result. Klient może anulować w dowolnym
momencie przez Cancel. Używamy go dla sekwencji ruchów, które mogą trwać 10-30s.

DLACZEGO MultiThreadedExecutor?
Domyślny SingleThreadedExecutor wykonuje callbacki jeden po drugim.
Gdy Action Server wykonuje długą sekwencję (blokuje), żaden serwis
ani timer nie może się odezwać. MultiThreadedExecutor z ReentrantCallbackGroup
pozwala na równoległe callbacki – timer działa, serwisy odpowiadają,
nawet gdy Action Server jest zajęty.
"""

import asyncio
import threading
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from robomvp.action import ManipulationTask
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


# Mapowanie nazwy sekwencji (z Action Goal) na funkcję generującą waypoints.
# Słownik zamiast if/elif – łatwiej rozszerzać, łatwiej testować.
_SEQUENCE_BUILDERS = {
    'approach_table':       get_approach_table,
    'pick_box':             get_pick_box,
    'rotate_180':           get_rotate_180,
    'walk_to_second_table': get_walk_to_second_table,
    'place_box':            get_place_box,
}


class RoboMVPMain(Node):
    """Węzeł główny systemu RoboMVP z pełną komunikacją ROS2."""

    def __init__(self):
        super().__init__('robomvp_main')

        # ReentrantCallbackGroup pozwala na równoległe wywołania callbacków.
        # Bez tego Action Server i timery blokują się wzajemnie (domyślna
        # MutuallyExclusiveCallbackGroup dopuszcza tylko jeden callback naraz).
        self._cbg = ReentrantCallbackGroup()

        self.declare_parameter('scene_config_path', '')
        self.declare_parameter('step_period', 1.0)
        self.declare_parameter('network_interface', 'eth0')
        self.declare_parameter('require_robot_connection', False)

        scene_config_path       = self.get_parameter('scene_config_path').value
        step_period             = float(self.get_parameter('step_period').value)
        network_interface       = str(self.get_parameter('network_interface').value)
        self._require_robot_conn = bool(self.get_parameter('require_robot_connection').value)

        self._config = self._load_config(scene_config_path)

        # ── Węzeł odometrii – tworzymy osobno żeby podłączyć callback ───────────
        # OdometryNode jest tworzony PRZED UnitreeRobotAPI, bo API potrzebuje
        # referencji do jego metody set_velocity() jako velocity_callback.
        # Dzięki temu odometria jest zsynchronizowana z faktycznymi komendami
        # prędkości bez potrzeby parsowania tematów ROS2.
        from robomvp.robomvp_odometry import OdometryNode  # import lokalny – unika cykli
        self._odometry_node = OdometryNode()

        # ── Połączenie z robotem ──────────────────────────────────────────────
        self._robot_api: UnitreeRobotAPI | None = UnitreeRobotAPI(
            network_interface=network_interface,
            velocity_callback=self._odometry_node.set_velocity,
            stop_callback=self._odometry_node.stop_velocity,
        )
        self._robot_connected = False
        try:
            self._robot_api.connect(logger=self.get_logger())
            self._robot_connected = True
            self.get_logger().info('Połączenie z robotem: aktywne.')
        except RuntimeError as exc:
            if self._require_robot_conn:
                raise
            self.get_logger().warning(
                f'Brak połączenia z robotem ({exc}). Tryb demo – komendy logowane.'
            )
            self._robot_api = None

        # ── Automat stanowy ───────────────────────────────────────────────────
        offset_scale     = self._config.get('offset_scale', {})
        self._scale_dx   = float(offset_scale.get('dx', 1.0))
        self._scale_dy   = float(offset_scale.get('dy', 1.0))
        self._scale_dz   = float(offset_scale.get('dz', 1.0))
        self._state_machine = StateMachine(self._config, logger=self.get_logger())

        motion_t = self._config.get('motion_timeouts', {})
        self._motion_total = float(motion_t.get('total', 30.0))
        self._motion_step  = float(motion_t.get('step', 5.0))

        # ── Sterowanie scenariuszem ───────────────────────────────────────────
        self._paused:           bool  = False
        self._sequence_running: bool  = False
        self._current_offset          = (0.0, 0.0, 0.0)
        # Mutex chroni _state_machine i _sequence_running przed race condition
        # gdy timer (_step) i wątek sekwencji działają jednocześnie.
        self._state_lock = threading.Lock()

        # ── Subskrypcje ───────────────────────────────────────────────────────
        self.create_subscription(
            MarkerPose, '/robomvp/marker_pose', self._on_marker_pose, 10,
            callback_group=self._cbg,
        )
        self.create_subscription(
            Offset, '/robomvp/offset', self._on_offset, 10,
            callback_group=self._cbg,
        )

        # ── Publikacje ────────────────────────────────────────────────────────
        self._pub_state  = self.create_publisher(StateMsg, '/robomvp/state',          10)
        self._pub_motion = self.create_publisher(String,   '/robomvp/motion_command', 10)

        # ── Serwisy ───────────────────────────────────────────────────────────
        # /robomvp/pause:  std_srvs/SetBool – True = wstrzymaj, False = wznów
        #   Przykład:  ros2 service call /robomvp/pause std_srvs/SetBool '{data: true}'
        self.create_service(
            SetBool, '/robomvp/pause', self._handle_pause,
            callback_group=self._cbg,
        )
        # /robomvp/emergency_stop: natychmiastowe zatrzymanie + rozłączenie SDK
        #   Przykład:  ros2 service call /robomvp/emergency_stop std_srvs/Trigger '{}'
        self.create_service(
            Trigger, '/robomvp/emergency_stop', self._handle_emergency_stop,
            callback_group=self._cbg,
        )
        # /robomvp/reset: reset automatu do SEARCH_TABLE bez restartu węzła
        #   Przykład:  ros2 service call /robomvp/reset std_srvs/Trigger '{}'
        self.create_service(
            Trigger, '/robomvp/reset', self._handle_reset,
            callback_group=self._cbg,
        )

        # ── Action Server ─────────────────────────────────────────────────────
        # /robomvp/manipulation_task: wykonaj pojedynczą sekwencję lub pełny scenariusz
        #   Przykład (ros2 action):
        #     ros2 action send_goal /robomvp/manipulation_task \
        #       robomvp/action/ManipulationTask \
        #       '{sequence_name: "pick_box", apply_offset: true}'
        self._action_server = ActionServer(
            self,
            ManipulationTask,
            '/robomvp/manipulation_task',
            execute_callback    = self._execute_action,
            goal_callback       = self._action_goal_cb,
            cancel_callback     = self._action_cancel_cb,
            callback_group      = self._cbg,
        )

        # ── Timer automatu stanowego ──────────────────────────────────────────
        self._timer = self.create_timer(
            step_period, self._step, callback_group=self._cbg,
        )

        self.get_logger().info(
            'Węzeł główny RoboMVP gotowy.\n'
            f'  Robot:   {"połączony" if self._robot_connected else "brak (tryb demo)"}\n'
            '  Serwisy: /robomvp/pause  /robomvp/emergency_stop  /robomvp/reset\n'
            '  Action:  /robomvp/manipulation_task'
        )

    # ======================================================================
    # Callbacki subskrypcji
    # ======================================================================

    def _on_marker_pose(self, msg: MarkerPose):
        with self._state_lock:
            self._state_machine.update_marker(msg.marker_id, msg.x, msg.y, msg.z)

    def _on_offset(self, msg: Offset):
        dx = msg.dx * self._scale_dx
        dy = msg.dy * self._scale_dy
        dz = msg.dz * self._scale_dz
        self._current_offset = (dx, dy, dz)
        with self._state_lock:
            self._state_machine.update_offset(dx, dy, dz)

    # ======================================================================
    # Handlery serwisów
    # ======================================================================

    def _handle_pause(self, request: SetBool.Request, response: SetBool.Response):
        """Wstrzymuje (True) lub wznawia (False) automat stanowy.

        Pauza zatrzymuje timer – robot nie dostaje nowych komend z automatu,
        ale jeśli sekwencja jest aktualnie wykonywana w wątku, nie jest
        przerywana. Dla natychmiastowego zatrzymania: /robomvp/emergency_stop.
        """
        self._paused = request.data
        label = 'WSTRZYMANY' if self._paused else 'WZNOWIONY'
        self.get_logger().info(f'Scenariusz {label} (serwis /robomvp/pause).')
        response.success = True
        response.message = f'Scenariusz {label.lower()}'
        return response

    def _handle_emergency_stop(
            self, request: Trigger.Request, response: Trigger.Response):
        """Natychmiastowe zatrzymanie: StopMove + Damp + blokada timera.

        Po emergency stop:
          1. Robot zatrzymuje się (StopMove → Damp).
          2. Scenariusz jest wstrzymany (paused=True).
          3. SDK jest rozłączone (bezpieczniejsze niż trzymanie połączenia).
        Aby wznowić, wywołaj /robomvp/reset, a następnie /robomvp/pause '{data: false}'.
        """
        self.get_logger().error('!!! EMERGENCY STOP – natychmiastowe zatrzymanie robota !!!')
        self._paused = True
        if self._robot_api is not None:
            try:
                self._robot_api.disconnect()
            except Exception:
                pass
            self._robot_api = None
        response.success = True
        response.message = 'Robot zatrzymany. Wywołaj /robomvp/reset aby wznowić.'
        return response

    def _handle_reset(self, request: Trigger.Request, response: Trigger.Response):
        """Resetuje automat stanowy do SEARCH_TABLE bez restartu węzła.

        Przydatne gdy scenariusz zakończył się błędem lub gdy chcemy powtórzyć
        demonstrację bez restartu całego systemu ROS2.
        Ponownie łączy z robotem jeśli połączenie zostało zerwane przez e-stop.
        """
        with self._state_lock:
            self._state_machine = StateMachine(self._config, logger=self.get_logger())
        self._paused           = False
        self._sequence_running = False
        self._current_offset   = (0.0, 0.0, 0.0)

        # Ponowne połączenie z robotem jeśli e-stop je zerwał
        if self._robot_api is None and not self._robot_connected:
            try:
                self._robot_api = UnitreeRobotAPI(
                    network_interface=self.get_parameter('network_interface').value
                )
                self._robot_api.connect(logger=self.get_logger())
                self._robot_connected = True
                self.get_logger().info('Reset: ponownie połączono z robotem.')
            except RuntimeError as exc:
                self.get_logger().warning(f'Reset: nie można połączyć z robotem: {exc}')

        self.get_logger().info('Automat stanowy zresetowany do SEARCH_TABLE.')
        response.success = True
        response.message = 'Reset wykonany. Scenariusz gotowy do ponownego uruchomienia.'
        return response

    # ======================================================================
    # Action Server – ManipulationTask
    # ======================================================================

    def _action_goal_cb(self, goal_request):
        """Decyduje czy zaakceptować Goal: sprawdza nazwę sekwencji i dostępność."""
        seq = goal_request.sequence_name
        valid = list(_SEQUENCE_BUILDERS.keys()) + ['full_scenario']
        if seq not in valid:
            self.get_logger().warn(f'Action: nieznana sekwencja {seq!r}. Dostępne: {valid}')
            return GoalResponse.REJECT
        if self._sequence_running:
            self.get_logger().warn('Action: sekwencja już w toku.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _action_cancel_cb(self, goal_handle):
        """Akceptuje żądanie anulowania – bieżący krok dokończy się naturalnie."""
        self.get_logger().info('Action: anulowanie zaakceptowane.')
        return CancelResponse.ACCEPT

    async def _execute_action(self, goal_handle):
        """Wykonuje sekwencję z feedbackiem po każdym kroku (async coroutine).

        Używamy asyncio.sleep(0) między krokami żeby oddać sterowanie event
        loop i umożliwić sprawdzenie is_cancel_requested między krokami.
        Samo move_to_pose jest synchroniczne i blokuje – uruchamiamy je
        przez asyncio.get_event_loop().run_in_executor() w osobnym wątku.
        """
        seq_name     = goal_handle.request.sequence_name
        apply_off    = goal_handle.request.apply_offset
        dx, dy, dz   = self._current_offset if apply_off else (0.0, 0.0, 0.0)

        result = ManipulationTask.Result()
        t0     = time.monotonic()

        if seq_name == 'full_scenario':
            ok = await self._full_scenario_action(goal_handle)
        else:
            seq = apply_offset_to_sequence(_SEQUENCE_BUILDERS[seq_name](), dx, dy, dz)
            ok  = await self._sequence_action(goal_handle, seq, seq_name)

        result.success         = ok
        result.elapsed_seconds = time.monotonic() - t0
        result.final_state     = self._state_machine.current_state_name
        result.message         = 'OK' if ok else 'Błąd lub anulowanie'

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    async def _sequence_action(self, goal_handle, sequence, name) -> bool:
        """Wykonuje sekwencję krok po kroku z Feedback po każdym kroku.

        run_in_executor przenosi blokujące move_to_pose do wątku z puli,
        a await pozwala event loop na sprawdzanie Cancel między krokami.
        """
        loop  = asyncio.get_event_loop()
        total = len(sequence)
        t_all = time.monotonic()

        for i, pose in enumerate(sequence):
            if goal_handle.is_cancel_requested:
                return False

            fb                    = ManipulationTask.Feedback()
            fb.current_step       = i + 1
            fb.total_steps        = total
            fb.progress           = (i + 1) / total
            fb.step_description   = (
                f'Krok {i+1}/{total}: x={pose.get("x",0):.2f} '
                f'y={pose.get("y",0):.2f} yaw={pose.get("yaw",0):.2f}'
            )
            fb.current_state = self._state_machine.current_state_name
            goal_handle.publish_feedback(fb)

            if self._robot_api is not None:
                try:
                    # Blokujące wywołanie w osobnym wątku – nie blokuje event loop
                    await loop.run_in_executor(
                        None,
                        lambda p=pose: self._robot_api.move_to_pose(
                            p, timeout_s=self._motion_step
                        )
                    )
                except Exception as exc:
                    self.get_logger().error(f'Action {name}: błąd SDK krok {i+1}: {exc}')
                    return False
            else:
                self.get_logger().info(f'[no_robot] Action {name}: {fb.step_description}')
                # Symulacja czasu wykonania kroku w trybie demo
                await asyncio.sleep(0.1)

            if time.monotonic() - t_all > self._motion_total:
                self.get_logger().error(f'Action {name}: timeout całkowitego czasu.')
                return False

        return True

    async def _full_scenario_action(self, goal_handle) -> bool:
        """Monitoruje pełny scenariusz (automat sterowany przez timer) do FINISHED."""
        fb           = ManipulationTask.Feedback()
        fb.total_steps = 7
        t0           = time.monotonic()

        while True:
            if goal_handle.is_cancel_requested:
                return False
            with self._state_lock:
                name = self._state_machine.current_state_name
                sid  = int(self._state_machine.current_state)
            fb.current_step      = sid
            fb.progress          = sid / 7.0
            fb.current_state     = name
            fb.step_description  = f'Stan: {name}'
            goal_handle.publish_feedback(fb)
            if name == 'FINISHED':
                return True
            if time.monotonic() - t0 > 300.0:
                return False
            await asyncio.sleep(0.5)

    # ======================================================================
    # Krok automatu stanowego (timer)
    # ======================================================================

    def _step(self):
        if self._paused:
            return

        with self._state_lock:
            prev = self._state_machine.current_state
            new  = self._state_machine.step()

        msg              = StateMsg()
        msg.state_name   = self._state_machine.current_state_name
        msg.state_id     = int(new)
        self._pub_state.publish(msg)

        if new != prev:
            self.get_logger().info(f'Stan: {prev.name} → {new.name}')
            self._execute_state_action(new)

        if new == State.FINISHED:
            self._timer.cancel()

    # ======================================================================
    # Akcje per-stan
    # ======================================================================

    def _execute_state_action(self, state: State):
        dx, dy, dz = self._current_offset
        mapping = {
            State.DETECT_MARKER: (
                'approach_table',
                apply_offset_to_sequence(get_approach_table(), dx, dy, dz),
                False,
            ),
            State.PICK_BOX: (
                'pick_box',
                apply_offset_to_sequence(get_pick_box(), dx, dy, dz),
                True,
            ),
            State.ROTATE_180: (
                'rotate_180',
                get_rotate_180(),
                True,
            ),
            State.NAVIGATE_TO_TARGET_MARKER: (
                'walk_to_second_table',
                get_walk_to_second_table(),
                False,
            ),
            State.PLACE_BOX: (
                'place_box',
                apply_offset_to_sequence(get_place_box(), dx, dy, dz),
                True,
            ),
        }
        if state == State.ALIGN_WITH_BOX:
            self._publish_motion('align_with_box')
            return
        if state == State.FINISHED:
            self._publish_motion('finished')
            return
        if state not in mapping:
            return

        name, seq, notify = mapping[state]
        self._publish_motion(name)
        self._launch_thread(seq, name, notify)

    def _launch_thread(self, sequence, name, notify_done):
        if self._sequence_running:
            self.get_logger().warning(
                f'Próba uruchomienia {name!r} gdy inna sekwencja trwa – pominięto.'
            )
            return
        self._sequence_running = True
        threading.Thread(
            target=self._run_in_thread,
            args=(sequence, name, notify_done),
            daemon=True,
            name=f'seq_{name}',
        ).start()

    def _run_in_thread(self, sequence, name, notify_done):
        try:
            ok = execute_sequence(
                sequence,
                robot_api        = self._robot_api,
                logger           = self.get_logger(),
                total_timeout_s  = self._motion_total,
                step_timeout_s   = self._motion_step,
            )
            if ok and notify_done:
                with self._state_lock:
                    self._state_machine.notify_sequence_done()
            elif not ok:
                self.get_logger().error(f'Sekwencja {name!r}: błąd lub timeout.')
        except Exception as exc:
            self.get_logger().error(f'Błąd wątku sekwencji {name!r}: {exc}')
        finally:
            self._sequence_running = False

    def _publish_motion(self, cmd: str):
        msg      = String()
        msg.data = cmd
        self._pub_motion.publish(msg)

    # ======================================================================
    # Wczytywanie konfiguracji
    # ======================================================================

    def _load_config(self, config_path: str) -> dict:
        if not config_path:
            for parent in Path(__file__).resolve().parents:
                candidate = parent / 'config' / 'scene.yaml'
                if candidate.is_file():
                    config_path = str(candidate)
                    break
        if config_path and Path(config_path).is_file():
            try:
                with open(config_path) as f:
                    cfg = yaml.safe_load(f)
                self.get_logger().info(f'Konfiguracja: {config_path}')
                return cfg or {}
            except Exception as e:
                self.get_logger().warning(f'Błąd konfiguracji: {e}')
        self.get_logger().warning('Używam domyślnej konfiguracji.')
        return {
            'stop_distance_threshold': 0.3,
            'alignment_threshold': 0.05,
            'offset_scale': {'dx': 1.0, 'dy': 1.0, 'dz': 1.0},
            'state_timeouts': {
                'search_table': 20.0, 'detect_marker': 20.0,
                'align_with_box': 10.0, 'navigate_to_target_marker': 25.0,
            },
            'motion_timeouts': {'total': 30.0, 'step': 5.0},
        }

    # ======================================================================
    # Czyszczenie zasobów
    # ======================================================================

    def destroy_node(self):
        if self._robot_api is not None:
            self._robot_api.disconnect()
            self._robot_api = None
        super().destroy_node()


def main(args=None):
    """Punkt wejścia – MultiThreadedExecutor dla Action Server + serwisów."""
    rclpy.init(args=args)
    node     = RoboMVPMain()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
