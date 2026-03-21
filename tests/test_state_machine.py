from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'ros2_ws' / 'src' / 'robomvp'))

from robomvp.state_machine import State, StateMachine


def _base_config():
    return {
        'table_markers': {'pickup_table': 21, 'place_table': 22},
        'box_marker_id': 10,
        'target_marker': 30,
        'stop_distance_threshold': 0.3,
        'alignment_threshold': 0.05,
    }


def test_navigate_to_target_accepts_target_marker():
    sm = StateMachine(_base_config())

    sm.update_marker(21, 0.0, 0.0, 1.0)
    assert sm.step() == State.DETECT_MARKER

    sm.update_marker(10, 0.0, 0.0, 1.0)
    assert sm.step() == State.ALIGN_WITH_BOX

    sm.update_offset(0.0, 0.0, 0.0)
    assert sm.step() == State.PICK_BOX
    assert sm.step() == State.ROTATE_180
    assert sm.step() == State.NAVIGATE_TO_TARGET_MARKER

    sm.update_marker(30, 0.0, 0.0, 0.2)
    assert sm.step() == State.PLACE_BOX


def test_invalid_state_timeout_falls_back_to_default():
    sm = StateMachine(
        {
            **_base_config(),
            'state_timeouts': {
                'search_table': 0,
                'detect_marker': 'bad-value',
            },
        }
    )

    assert sm._state_timeouts['search_table'] == 20.0
    assert sm._state_timeouts['detect_marker'] == 20.0
