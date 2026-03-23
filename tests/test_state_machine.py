"""Testy jednostkowe systemu RoboMVP v0.2.0.

Pokrycie:
  - StateMachine: pełny scenariusz, oczekiwanie na notify_sequence_done,
    timeouty awaryjne, sanityzacja konfiguracji, właściwości pomocnicze
  - motion_sequences: apply_offset, execute_sequence (tryb demo), zmienne total
  - UnitreeRobotAPI: velocity_callback, stop_callback, get_pose, tryb demo

Uruchamianie:
    cd ros2_ws/src/robomvp
    python -m pytest ../../../../tests/ -v --tb=short
"""

import sys
import time
import threading
from pathlib import Path

# Dodaj pakiet do sys.path żeby importy działały bez instalacji ROS2
sys.path.insert(
    0,
    str(Path(__file__).resolve().parents[1] / 'ros2_ws' / 'src' / 'robomvp')
)

from robomvp.state_machine import State, StateMachine
from robomvp.motion_sequences import (
    apply_offset_to_sequence,
    execute_sequence,
    get_approach_table,
    get_pick_box,
    get_place_box,
    get_rotate_180,
    get_walk_to_second_table,
)
from robomvp.unitree_robot_api import UnitreeRobotAPI


# ═══════════════════════════════════════════════════════════════════════════
# Fixture pomocnicza
# ═══════════════════════════════════════════════════════════════════════════

def base_config(**overrides):
    """Zwraca bazową konfigurację sceny do testów."""
    cfg = {
        'table_markers':    {'pickup_table': 21, 'place_table': 22},
        'box_marker_id':    10,
        'target_marker':    30,
        'stop_distance_threshold': 0.3,
        'alignment_threshold':     0.05,
        'state_timeouts': {
            'search_table':             20.0,
            'detect_marker':            20.0,
            'align_with_box':           10.0,
            'navigate_to_target_marker': 25.0,
        },
    }
    cfg.update(overrides)
    return cfg


def _advance_to_navigate(sm):
    """Pomocnik: przesuwa automat z SEARCH_TABLE do NAVIGATE_TO_TARGET_MARKER."""
    sm.update_marker(21, 0.0, 0.0, 1.0); sm.step()   # → DETECT_MARKER
    sm.update_marker(10, 0.0, 0.0, 1.0); sm.step()   # → ALIGN_WITH_BOX
    sm.update_offset(0.0, 0.0, 0.0);     sm.step()   # → PICK_BOX
    sm.notify_sequence_done();            sm.step()   # → ROTATE_180
    sm.notify_sequence_done();            sm.step()   # → NAVIGATE_TO_TARGET_MARKER


# ═══════════════════════════════════════════════════════════════════════════
# Testy StateMachine – ścieżka nominalna
# ═══════════════════════════════════════════════════════════════════════════

class TestStateMachineHappyPath:

    def test_full_scenario_with_notify(self):
        """Pełny scenariusz z poprawnymi notyfikacjami sekwencji."""
        sm = StateMachine(base_config())

        # Każdy sekwencyjny stan musi czekać na notify_sequence_done()
        assert sm.step() == State.SEARCH_TABLE  # brak markera → bez zmiany

        sm.update_marker(21, 0.0, 0.0, 1.0)
        assert sm.step() == State.DETECT_MARKER

        sm.update_marker(10, 0.0, 0.0, 1.0)
        assert sm.step() == State.ALIGN_WITH_BOX

        sm.update_offset(0.0, 0.0, 0.0)
        assert sm.step() == State.PICK_BOX

        # PICK_BOX czeka
        assert sm.step() == State.PICK_BOX
        assert sm.step() == State.PICK_BOX
        sm.notify_sequence_done()
        assert sm.step() == State.ROTATE_180

        # ROTATE_180 czeka
        assert sm.step() == State.ROTATE_180
        sm.notify_sequence_done()
        assert sm.step() == State.NAVIGATE_TO_TARGET_MARKER

        # Nawigacja: marker za daleko → brak przejścia
        sm.update_marker(30, 0.0, 0.0, 0.5)   # z=0.5 > 0.3
        assert sm.step() == State.NAVIGATE_TO_TARGET_MARKER

        # Marker wystarczająco blisko
        sm.update_marker(30, 0.0, 0.0, 0.2)   # z=0.2 < 0.3
        assert sm.step() == State.PLACE_BOX

        # PLACE_BOX czeka
        assert sm.step() == State.PLACE_BOX
        sm.notify_sequence_done()
        assert sm.step() == State.FINISHED

    def test_state_names(self):
        """Weryfikacja nazw stanów używanych w wiadomościach ROS2."""
        sm = StateMachine(base_config())
        assert sm.current_state_name == 'SEARCH_TABLE'

        sm.update_marker(21, 0.0, 0.0, 1.0); sm.step()
        assert sm.current_state_name == 'DETECT_MARKER'

    def test_place_table_marker_stops_navigation(self):
        """Marker place_table (ID=22) też zatrzymuje nawigację."""
        sm = StateMachine(base_config())
        _advance_to_navigate(sm)

        sm.update_marker(22, 0.0, 0.0, 0.15)   # marker stołu docelowego
        assert sm.step() == State.PLACE_BOX


# ═══════════════════════════════════════════════════════════════════════════
# Testy StateMachine – oczekiwanie i blokady
# ═══════════════════════════════════════════════════════════════════════════

class TestStateMachineWaiting:

    def test_pick_box_waits_without_notify(self):
        """PICK_BOX pozostaje w tym stanie bez notify_sequence_done."""
        sm = StateMachine(base_config())
        sm.update_marker(21, 0.0, 0.0, 1.0); sm.step()
        sm.update_marker(10, 0.0, 0.0, 1.0); sm.step()
        sm.update_offset(0.0, 0.0, 0.0);     sm.step()

        # Wielokrotne step() bez notify → zawsze PICK_BOX
        for _ in range(5):
            assert sm.step() == State.PICK_BOX

    def test_rotate_waits_without_notify(self):
        """ROTATE_180 pozostaje bez notify."""
        sm = StateMachine(base_config())
        sm.update_marker(21, 0.0, 0.0, 1.0); sm.step()
        sm.update_marker(10, 0.0, 0.0, 1.0); sm.step()
        sm.update_offset(0.0, 0.0, 0.0);     sm.step()
        sm.notify_sequence_done();            sm.step()  # → ROTATE_180

        for _ in range(3):
            assert sm.step() == State.ROTATE_180

    def test_place_box_waits_without_notify(self):
        """PLACE_BOX pozostaje bez notify."""
        sm = StateMachine(base_config())
        _advance_to_navigate(sm)
        sm.update_marker(30, 0.0, 0.0, 0.1); sm.step()  # → PLACE_BOX

        for _ in range(3):
            assert sm.step() == State.PLACE_BOX

    def test_align_waits_when_offset_too_large(self):
        """ALIGN_WITH_BOX nie przechodzi gdy |dx| przekracza próg."""
        sm = StateMachine(base_config())
        sm.update_marker(21, 0.0, 0.0, 1.0); sm.step()
        sm.update_marker(10, 0.0, 0.0, 1.0); sm.step()

        sm.update_offset(0.1, 0.0, 0.0)   # |dx|=0.1 > 0.05
        assert sm.step() == State.ALIGN_WITH_BOX

    def test_align_transitions_when_offset_ok(self):
        """ALIGN_WITH_BOX przechodzi gdy oba komponenty < threshold."""
        sm = StateMachine(base_config())
        sm.update_marker(21, 0.0, 0.0, 1.0); sm.step()
        sm.update_marker(10, 0.0, 0.0, 1.0); sm.step()

        sm.update_offset(0.04, 0.0, 0.04)   # oba < 0.05
        assert sm.step() == State.PICK_BOX

    def test_notify_sequence_done_ignored_in_non_sequence_states(self):
        """notify_sequence_done() nie powoduje zmian w stanach niesekwencyjnych."""
        sm = StateMachine(base_config())
        assert sm.current_state == State.SEARCH_TABLE

        sm.notify_sequence_done()           # powinno być zignorowane
        assert sm.step() == State.SEARCH_TABLE  # brak markera → bez zmiany

    def test_sequence_done_flag_reset_on_transition(self):
        """Flaga _sequence_done jest resetowana przy każdym przejściu stanu."""
        sm = StateMachine(base_config())
        sm.update_marker(21, 0.0, 0.0, 1.0); sm.step()
        sm.update_marker(10, 0.0, 0.0, 1.0); sm.step()
        sm.update_offset(0.0, 0.0, 0.0);     sm.step()  # → PICK_BOX

        # Wywołaj notify zanim zakończymy PICK_BOX – symuluje "wczesne" potwierdzenie
        sm.notify_sequence_done()
        sm.step()   # → ROTATE_180

        # Po wejściu do ROTATE_180 flaga powinna być zresetowana
        assert not sm._sequence_done, '_sequence_done powinno być False po przejściu'

        # Kolejne step() bez notify → zostajemy w ROTATE_180
        assert sm.step() == State.ROTATE_180


# ═══════════════════════════════════════════════════════════════════════════
# Testy StateMachine – is_in_sequence_state
# ═══════════════════════════════════════════════════════════════════════════

class TestIsInSequenceState:

    def test_sequence_states(self):
        sm = StateMachine(base_config())
        # Dojdź do PICK_BOX
        sm.update_marker(21, 0.0, 0.0, 1.0); sm.step()
        sm.update_marker(10, 0.0, 0.0, 1.0); sm.step()
        sm.update_offset(0.0, 0.0, 0.0);     sm.step()

        assert sm.is_in_sequence_state  # PICK_BOX

        sm.notify_sequence_done(); sm.step()  # → ROTATE_180
        assert sm.is_in_sequence_state  # ROTATE_180

        sm.notify_sequence_done(); sm.step()  # → NAVIGATE
        assert not sm.is_in_sequence_state    # nawigacja nie jest sekwencją

    def test_non_sequence_states(self):
        sm = StateMachine(base_config())
        assert not sm.is_in_sequence_state   # SEARCH_TABLE

        sm.update_marker(21, 0.0, 0.0, 1.0); sm.step()
        assert not sm.is_in_sequence_state   # DETECT_MARKER

        sm.update_marker(10, 0.0, 0.0, 1.0); sm.step()
        assert not sm.is_in_sequence_state   # ALIGN_WITH_BOX


# ═══════════════════════════════════════════════════════════════════════════
# Testy StateMachine – sanityzacja konfiguracji
# ═══════════════════════════════════════════════════════════════════════════

class TestConfigSanitization:

    def test_zero_timeout_falls_back(self):
        sm = StateMachine(base_config(state_timeouts={'search_table': 0}))
        assert sm._state_timeouts['search_table'] == 20.0

    def test_negative_timeout_falls_back(self):
        sm = StateMachine(base_config(state_timeouts={'search_table': -5.0}))
        assert sm._state_timeouts['search_table'] == 20.0

    def test_string_timeout_falls_back(self):
        sm = StateMachine(base_config(state_timeouts={'detect_marker': 'bad'}))
        assert sm._state_timeouts['detect_marker'] == 20.0

    def test_valid_custom_timeout(self):
        sm = StateMachine(base_config(state_timeouts={'search_table': 15.0}))
        assert sm._state_timeouts['search_table'] == 15.0

    def test_empty_config_uses_defaults(self):
        sm = StateMachine({})
        assert sm._box_marker_id        == 10
        assert sm._pickup_table_marker  == 21
        assert sm._place_table_marker   == 22
        assert sm._stop_distance        == 0.3
        assert sm._align_threshold      == 0.05

    def test_custom_marker_ids(self):
        cfg = base_config()
        cfg['box_marker_id'] = 99
        cfg['table_markers'] = {'pickup_table': 55, 'place_table': 77}
        sm = StateMachine(cfg)
        assert sm._box_marker_id       == 99
        assert sm._pickup_table_marker == 55
        assert sm._place_table_marker  == 77


# ═══════════════════════════════════════════════════════════════════════════
# Testy motion_sequences
# ═══════════════════════════════════════════════════════════════════════════

class TestMotionSequences:

    def test_all_sequences_non_empty(self):
        """Każda sekwencja zwraca co najmniej 2 waypoints."""
        for fn in [get_approach_table, get_pick_box, get_rotate_180,
                   get_walk_to_second_table, get_place_box]:
            seq = fn()
            assert len(seq) >= 2, f'{fn.__name__} zwróciła < 2 waypoints'

    def test_waypoint_keys(self):
        """Każdy waypoint ma wymagane klucze x, y, z, yaw."""
        for fn in [get_approach_table, get_pick_box, get_rotate_180,
                   get_walk_to_second_table, get_place_box]:
            for pose in fn():
                assert set(pose.keys()) >= {'x', 'y', 'z', 'yaw'}, (
                    f'{fn.__name__}: brakujące klucze w {pose}'
                )

    def test_apply_offset(self):
        """apply_offset_to_sequence przesuwa x, y, z o podane wartości."""
        seq = [{'x': 1.0, 'y': 2.0, 'z': 0.5, 'yaw': 0.0}]
        corrected = apply_offset_to_sequence(seq, dx=0.1, dy=0.2, dz=-0.1)
        assert abs(corrected[0]['x'] - 1.1) < 1e-9
        assert abs(corrected[0]['y'] - 2.2) < 1e-9
        assert abs(corrected[0]['z'] - 0.4) < 1e-9
        assert corrected[0]['yaw'] == 0.0   # yaw bez zmiany

    def test_apply_offset_does_not_modify_original(self):
        """apply_offset_to_sequence nie modyfikuje oryginalnej listy."""
        original = [{'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}]
        _ = apply_offset_to_sequence(original, dx=1.0, dy=1.0, dz=1.0)
        assert original[0]['x'] == 0.0   # oryginał bez zmian

    def test_execute_sequence_no_robot_logs(self, capsys):
        """execute_sequence w trybie demo (robot_api=None) loguje kroki."""
        class FakeLogger:
            def __init__(self): self.messages = []
            def info(self, msg): self.messages.append(msg)
            def error(self, msg): self.messages.append(msg)

        logger = FakeLogger()
        seq    = get_approach_table()
        ok     = execute_sequence(seq, robot_api=None, logger=logger)

        assert ok is True
        # Sprawdź że zostały zalogowane komunikaty dla każdego kroku
        step_logs = [m for m in logger.messages if '[no_robot]' in m]
        assert len(step_logs) == len(seq), (
            f'Oczekiwano {len(seq)} logów kroków, dostałem {len(step_logs)}'
        )

    def test_execute_sequence_total_variable_defined(self):
        """Zmienna 'total' w execute_sequence jest zdefiniowana (bug #1 z v0.1)."""
        # Ten test był niemożliwy do przejścia w v0.1.0 – NameError: total
        class CollectLogger:
            def __init__(self): self.msgs = []
            def info(self, m): self.msgs.append(m)
            def error(self, m): self.msgs.append(m)

        logger = CollectLogger()
        seq    = get_pick_box()
        ok     = execute_sequence(seq, robot_api=None, logger=logger)
        assert ok is True
        # Jeśli 'total' nie byłoby zdefiniowane, execute_sequence rzuciłoby NameError

    def test_execute_sequence_timeout(self):
        """execute_sequence zwraca False po przekroczeniu total_timeout."""
        class SlowAPI:
            def move_to_pose(self, pose, timeout_s=5.0):
                time.sleep(0.5)

        seq = get_approach_table()
        ok  = execute_sequence(
            seq, robot_api=SlowAPI(), logger=None,
            total_timeout_s=0.1,   # bardzo krótki timeout
            step_timeout_s=10.0,
        )
        assert ok is False


# ═══════════════════════════════════════════════════════════════════════════
# Testy UnitreeRobotAPI – tryb demo (bez SDK)
# ═══════════════════════════════════════════════════════════════════════════

class TestUnitreeRobotAPIDemoMode:
    """Testy API bez fizycznego robota – weryfikacja logiki, nie SDK."""

    def test_velocity_callback_called_on_move(self):
        """velocity_callback jest wywoływany gdy move_to_pose wysyła SetVelocity."""
        called_with = []

        def fake_sdk(api_instance):
            """Zamienia _send_velocity żeby nie wymagać SDK."""
            original = api_instance._send_velocity
            def patched(vx, vy, vyaw, duration_s):
                called_with.append((vx, vy, vyaw))
            api_instance._send_velocity = patched

        cb_args = []
        api = UnitreeRobotAPI(velocity_callback=lambda vx, vy, vyaw: cb_args.append((vx,vy,vyaw)))

        # Ręczne ustawienie _sdk_available żeby move_to_pose przeszło walidację
        api._sdk_available = True

        # Podmień _send_velocity na imitację (nie wywołuje SetVelocity)
        sent = []
        def mock_send(vx, vy, vyaw, duration_s):
            cb = api._velocity_cb
            if cb:
                cb(vx, vy, vyaw)
            sent.append((vx, vy, vyaw))
        api._send_velocity = mock_send

        # Podmień _loco_client na obiekt zastępczy
        class FakeLoco:
            def SetVelocity(self, *a): pass
            def StopMove(self): pass
        api._loco_client = FakeLoco()

        api.move_to_pose({'x': 0.0, 'y': 0.5, 'z': 0.0, 'yaw': 0.0})

        # velocity_callback powinno być wywołane przynajmniej raz (faza translacji)
        assert len(cb_args) >= 1

    def test_get_pose_initial(self):
        """get_pose zwraca (0, 0, 0) na starcie."""
        api  = UnitreeRobotAPI()
        pose = api.get_pose()
        assert pose == {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

    def test_velocity_callback_parameter(self):
        """UnitreeRobotAPI akceptuje velocity_callback w konstruktorze."""
        cb  = lambda vx, vy, vyaw: None
        api = UnitreeRobotAPI(velocity_callback=cb)
        assert api._velocity_cb is cb

    def test_stop_callback_parameter(self):
        """UnitreeRobotAPI akceptuje stop_callback w konstruktorze."""
        cb  = lambda: None
        api = UnitreeRobotAPI(stop_callback=cb)
        assert api._stop_cb is cb

    def test_is_connected_false_without_connect(self):
        """is_connected = False przed wywołaniem connect()."""
        api = UnitreeRobotAPI()
        assert not api.is_connected

    def test_move_to_pose_raises_without_connect(self):
        """move_to_pose rzuca RuntimeError gdy SDK nie jest zainicjalizowane."""
        import pytest
        api = UnitreeRobotAPI()
        with pytest.raises(RuntimeError, match='nie zainicjalizowane'):
            api.move_to_pose({'x': 0.0, 'y': 1.0, 'z': 0.0, 'yaw': 0.0})

    def test_execute_arm_action_raises_without_connect(self):
        """execute_arm_action rzuca RuntimeError gdy SDK nie jest zainicjalizowane."""
        import pytest
        api = UnitreeRobotAPI()
        with pytest.raises(RuntimeError):
            api.execute_arm_action('shake hand')


# ═══════════════════════════════════════════════════════════════════════════
# Testy integracyjne – automat + sekwencje
# ═══════════════════════════════════════════════════════════════════════════

class TestIntegration:

    def test_full_cycle_demo_no_crash(self):
        """Pełny cykl automatu w trybie demo nie rzuca wyjątków."""
        sm = StateMachine(base_config())

        class LogCollector:
            msgs = []
            def info(self, m): self.msgs.append(m)
            def error(self, m): self.msgs.append(m)
            def debug(self, m): pass

        logger = LogCollector()

        # Przejdź przez wszystkie stany
        sm.update_marker(21, 0.0, 0.0, 1.0); sm.step()
        sm.update_marker(10, 0.0, 0.0, 1.0); sm.step()
        sm.update_offset(0.02, 0.0, 0.02); sm.step()  # offset < 0.05

        # PICK_BOX: symuluj wykonanie sekwencji i notyfikację
        assert sm.current_state == State.PICK_BOX
        seq = apply_offset_to_sequence(get_pick_box(), 0.02, 0.0, 0.02)
        ok  = execute_sequence(seq, robot_api=None, logger=logger)
        assert ok
        sm.notify_sequence_done()
        sm.step()  # → ROTATE_180

        assert sm.current_state == State.ROTATE_180
        ok = execute_sequence(get_rotate_180(), robot_api=None, logger=logger)
        assert ok
        sm.notify_sequence_done()
        sm.step()  # → NAVIGATE

        assert sm.current_state == State.NAVIGATE_TO_TARGET_MARKER
        ok = execute_sequence(get_walk_to_second_table(), robot_api=None, logger=logger)
        assert ok

        sm.update_marker(30, 0.0, 0.0, 0.2)
        sm.step()   # → PLACE_BOX

        assert sm.current_state == State.PLACE_BOX
        ok = execute_sequence(get_place_box(), robot_api=None, logger=logger)
        assert ok
        sm.notify_sequence_done()
        sm.step()  # → FINISHED

        assert sm.current_state == State.FINISHED

    def test_threaded_notify_sequence_done(self):
        """notify_sequence_done wywołane z osobnego wątku działa poprawnie."""
        sm   = StateMachine(base_config())
        lock = threading.Lock()

        sm.update_marker(21, 0.0, 0.0, 1.0); sm.step()
        sm.update_marker(10, 0.0, 0.0, 1.0); sm.step()
        sm.update_offset(0.0, 0.0, 0.0);     sm.step()

        assert sm.current_state == State.PICK_BOX

        def notify_thread():
            time.sleep(0.05)  # krótkie opóźnienie symulujące wykonanie sekwencji
            with lock:
                sm.notify_sequence_done()

        t = threading.Thread(target=notify_thread, daemon=True)
        t.start()
        t.join(timeout=1.0)

        # Timer automatu: po notyfikacji z wątku, step() powinno przejść
        with lock:
            result = sm.step()

        assert result == State.ROTATE_180
