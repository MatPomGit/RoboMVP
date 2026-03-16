#!/usr/bin/env python3
"""Moduł predefiniowanych sekwencji ruchów dla robota Unitree G1 EDU.

Zawiera zakodowane na stałe sekwencje trajektorii
dla wszystkich etapów scenariusza manipulacji.
"""

import time


# Typ pozy: słownik z pozycją i orientacją (yaw w radianach)
# Format: {'x': float, 'y': float, 'z': float, 'yaw': float}


def get_approach_table() -> list:
    """Zwraca sekwencję podejścia do pierwszego stołu.

    Robot porusza się przed stół z pudełkiem.
    """
    return [
        {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0},   # pozycja startowa
        {'x': 0.0, 'y': 0.3, 'z': 0.0, 'yaw': 0.0},   # krok naprzód
        {'x': 0.0, 'y': 0.6, 'z': 0.0, 'yaw': 0.0},   # bliżej stołu
        {'x': 0.0, 'y': 0.8, 'z': 0.0, 'yaw': 0.0},   # pozycja robocza
    ]


def get_pick_box() -> list:
    """Zwraca sekwencję podniesienia pudełka.

    Ramię opuszcza się, chwyta pudełko i podnosi.
    """
    return [
        {'x': 0.0, 'y': 0.8, 'z': 0.0, 'yaw': 0.0},   # pozycja wyjściowa
        {'x': 0.0, 'y': 0.8, 'z': -0.3, 'yaw': 0.0},  # ramię w dół
        {'x': 0.0, 'y': 0.8, 'z': -0.5, 'yaw': 0.0},  # pozycja chwytu
        {'x': 0.0, 'y': 0.8, 'z': 0.0, 'yaw': 0.0},   # uniesienie pudełka
    ]


def get_rotate_180() -> list:
    """Zwraca sekwencję obrotu o 180 stopni.

    Robot obraca się w miejscu o pół obrotu.
    """
    import math
    return [
        {'x': 0.0, 'y': 0.8, 'z': 0.0, 'yaw': 0.0},           # orientacja startowa
        {'x': 0.0, 'y': 0.8, 'z': 0.0, 'yaw': math.pi / 2},   # 90 stopni
        {'x': 0.0, 'y': 0.8, 'z': 0.0, 'yaw': math.pi},       # 180 stopni
    ]


def get_walk_to_second_table() -> list:
    """Zwraca sekwencję chodzenia do drugiego stołu.

    Robot idzie do stołu docelowego z pudełkiem.
    """
    import math
    return [
        {'x': 0.0, 'y': 0.8, 'z': 0.0, 'yaw': math.pi},   # po obrocie
        {'x': 0.0, 'y': 1.5, 'z': 0.0, 'yaw': math.pi},   # ruch naprzód
        {'x': 0.0, 'y': 2.5, 'z': 0.0, 'yaw': math.pi},   # dalej
        {'x': 0.0, 'y': 3.2, 'z': 0.0, 'yaw': math.pi},   # przed drugim stołem
    ]


def get_place_box() -> list:
    """Zwraca sekwencję odłożenia pudełka na drugi stół.

    Robot opuszcza ramię z pudełkiem i zwalnia chwyt.
    """
    import math
    return [
        {'x': 0.0, 'y': 3.2, 'z': 0.0, 'yaw': math.pi},    # pozycja wyjściowa
        {'x': 0.0, 'y': 3.2, 'z': -0.3, 'yaw': math.pi},   # ramię w dół
        {'x': 0.0, 'y': 3.2, 'z': -0.5, 'yaw': math.pi},   # odłożenie pudełka
        {'x': 0.0, 'y': 3.2, 'z': 0.0, 'yaw': math.pi},    # ramię w górę
    ]


def apply_offset_to_sequence(sequence: list, dx: float, dy: float, dz: float) -> list:
    """Stosuje offset korekcji do całej sekwencji ruchów.

    Args:
        sequence: Lista poz do wykonania.
        dx: Korekcja boczna (metry).
        dy: Korekcja przód/tył (metry).
        dz: Korekcja pionowa (metry).

    Returns:
        Skorygowana sekwencja poz.
    """
    corrected = []
    for pose in sequence:
        corrected_pose = dict(pose)
        corrected_pose['x'] = pose.get('x', 0.0) + dx
        corrected_pose['y'] = pose.get('y', 0.0) + dy
        corrected_pose['z'] = pose.get('z', 0.0) + dz
        corrected.append(corrected_pose)
    return corrected


def execute_sequence(
    sequence: list,
    robot_api=None,
    logger=None,
    total_timeout_s: float = 30.0,
    step_timeout_s: float = 5.0,
) -> bool:
    """Wykonuje sekwencję ruchów przez Unitree SDK lub loguje w trybie demo.

    Args:
        sequence: Lista poz do wykonania.
        robot_api: Interfejs API robota Unitree (None w trybie demo).
        logger: Logger ROS2 do wypisywania komunikatów.
        total_timeout_s: Maksymalny czas wykonania całej sekwencji (sekundy).
        step_timeout_s: Maksymalny czas pojedynczego kroku (sekundy).

    Returns:
        True jeśli sekwencja wykonana pomyślnie, False w przypadku błędu.
    """
    total = len(sequence)
    if logger:
        logger.info(f'Rozpoczynam wykonanie sekwencji {total} kroków.')
    start = time.monotonic()
    for i, pose in enumerate(sequence):
        elapsed_total = time.monotonic() - start
        if elapsed_total > total_timeout_s:
            if logger:
                logger.error(
                    f'Timeout sekwencji po {elapsed_total:.2f}s (limit {total_timeout_s:.2f}s)'
                )
            return False

        if robot_api is not None:
            try:
                robot_api.move_to_pose(pose, timeout_s=step_timeout_s)
            except Exception as e:
                if logger:
                    logger.error(
                        f'Błąd wykonania kroku {i + 1}/{total}: {e}. '
                        'Sprawdź połączenie z robotem i stan interfejsu SDK. '
                        'Wykonanie sekwencji zakończone błędem.'
                    )
                return False
        else:
            if logger:
                logger.info(
                    f'[demo] Krok {i + 1}/{total}: '
                    f'x={pose.get("x", 0):.2f}, '
                    f'y={pose.get("y", 0):.2f}, '
                    f'z={pose.get("z", 0):.2f}, '
                    f'yaw={pose.get("yaw", 0):.2f}'
                )
    if logger:
        logger.info(f'Sekwencja {total} kroków wykonana pomyślnie.')
    return True
