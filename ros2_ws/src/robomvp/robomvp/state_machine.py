#!/usr/bin/env python3
"""Deterministyczny automat stanowy dla systemu RoboMVP.

Implementuje sekwencję stanów scenariusza manipulacji:

    SEARCH_TABLE  → wykryto marker stołu startowego (ID=21)
    DETECT_MARKER → wykryto marker pudełka (ID=10)
    ALIGN_WITH_BOX → offset korekcji poniżej progu
    PICK_BOX      → sekwencja podniesienia zakończona
    ROTATE_180    → sekwencja obrotu zakończona
    NAVIGATE_TO_TARGET_MARKER → marker docelowego stołu (ID=22) w zasięgu
    PLACE_BOX     → sekwencja odkładania zakończona
    FINISHED

Klasa ``StateMachine`` jest niezależna od ROS2 i może być testowana
bez uruchamiania węzłów. Wymagana jest jedynie konfiguracja sceny
(słownik z parametrami z pliku ``scene.yaml``) i opcjonalny logger.
"""

import time
from enum import IntEnum


class State(IntEnum):
    """Stany automatu stanowego scenariusza manipulacji."""
    SEARCH_TABLE = 0
    DETECT_MARKER = 1
    ALIGN_WITH_BOX = 2
    PICK_BOX = 3
    ROTATE_180 = 4
    NAVIGATE_TO_TARGET_MARKER = 5
    PLACE_BOX = 6
    FINISHED = 7


# Nazwy stanów do publikacji na tematy ROS2
STATE_NAMES = {
    State.SEARCH_TABLE: 'SEARCH_TABLE',
    State.DETECT_MARKER: 'DETECT_MARKER',
    State.ALIGN_WITH_BOX: 'ALIGN_WITH_BOX',
    State.PICK_BOX: 'PICK_BOX',
    State.ROTATE_180: 'ROTATE_180',
    State.NAVIGATE_TO_TARGET_MARKER: 'NAVIGATE_TO_TARGET_MARKER',
    State.PLACE_BOX: 'PLACE_BOX',
    State.FINISHED: 'FINISHED',
}


class StateMachine:
    """Deterministyczny automat stanowy scenariusza manipulacji.

    Zarządza przejściami między stanami na podstawie
    wykrytych markerów i zakończonych sekwencji ruchów.
    """

    def __init__(self, config: dict, logger=None):
        """Inicjalizuje automat stanowy z konfiguracją sceny.

        Args:
            config: Słownik z parametrami konfiguracji scene.yaml.
            logger: Logger ROS2.
        """
        self._config = config
        self._logger = logger
        self._state = State.SEARCH_TABLE
        self._last_marker_id = None
        self._last_pose = None
        self._last_offset = None

        # Identyfikatory markerów z konfiguracji
        self._box_marker_id = config.get('box_marker_id', 10)
        self._pickup_table_marker = config.get('table_markers', {}).get('pickup_table', 21)
        self._place_table_marker = config.get('table_markers', {}).get('place_table', 22)

        # Progi odległości
        self._stop_distance = config.get('stop_distance_threshold', 0.3)
        self._align_threshold = config.get('alignment_threshold', 0.05)

        # Timeouty stanów (sekundy)
        default_timeouts = {
            'search_table': 20.0,
            'detect_marker': 20.0,
            'align_with_box': 10.0,
            'navigate_to_target_marker': 25.0,
        }
        configured_timeouts = config.get('state_timeouts', {})
        merged = {**default_timeouts, **configured_timeouts}
        self._state_timeouts = {}
        for key, val in merged.items():
            try:
                fval = float(val)
            except (TypeError, ValueError):
                continue
            if fval >= 0.0:
                self._state_timeouts[key] = fval

        self._state_enter_time = time.monotonic()

    @property
    def current_state(self) -> State:
        """Zwraca aktualny stan automatu."""
        return self._state

    @property
    def current_state_name(self) -> str:
        """Zwraca nazwę aktualnego stanu."""
        return STATE_NAMES[self._state]

    def update_marker(self, marker_id: int, x: float, y: float, z: float):
        """Aktualizuje ostatnio wykryty marker.

        Args:
            marker_id: Identyfikator markera.
            x, y, z: Pozycja markera względem kamery.
        """
        self._last_marker_id = marker_id
        self._last_pose = (x, y, z)
        self._log(
            f'Wykryto marker ID={marker_id} na pozycji '
            f'({x:.3f}, {y:.3f}, {z:.3f}) m względem kamery.'
        )

    def update_offset(self, dx: float, dy: float, dz: float):
        """Aktualizuje ostatni obliczony offset korekcji.

        Args:
            dx, dy, dz: Wartości korekcji w metrach.
        """
        self._last_offset = (dx, dy, dz)
        self._log(
            f'Zaktualizowano offset korekcji: '
            f'dx={dx:.4f}, dy={dy:.4f}, dz={dz:.4f} m.'
        )

    def step(self) -> State:
        """Wykonuje jeden krok automatu stanowego.

        Sprawdza warunki przejścia i zmienia stan jeśli warunki są spełnione.

        Returns:
            Nowy (lub aktualny) stan po wykonaniu kroku.
        """
        if self._state == State.SEARCH_TABLE:
            self._handle_search_table()
        elif self._state == State.DETECT_MARKER:
            self._handle_detect_marker()
        elif self._state == State.ALIGN_WITH_BOX:
            self._handle_align_with_box()
        elif self._state == State.PICK_BOX:
            self._handle_pick_box()
        elif self._state == State.ROTATE_180:
            self._handle_rotate_180()
        elif self._state == State.NAVIGATE_TO_TARGET_MARKER:
            self._handle_navigate_to_target()
        elif self._state == State.PLACE_BOX:
            self._handle_place_box()

        return self._state

    def _transition_to(self, new_state: State):
        """Przechodzi do nowego stanu i loguje przejście."""
        old_name = STATE_NAMES[self._state]
        new_name = STATE_NAMES[new_state]
        self._state = new_state
        self._state_enter_time = time.monotonic()
        self._log(f'Zmiana stanu: {old_name} -> {new_name}')

    def _is_timeout(self, key: str) -> bool:
        """Sprawdza czy upłynął timeout dla bieżącego stanu."""
        timeout_s = self._state_timeouts.get(key)
        if timeout_s is None:
            return False
        elapsed = time.monotonic() - self._state_enter_time
        return elapsed >= timeout_s

    def _handle_search_table(self):
        """Stan: szukanie stołu z pudełkiem przez detekcję markera."""
        if self._last_marker_id == self._pickup_table_marker:
            self._log(
                f'Wykryto marker stołu startowego (ID={self._pickup_table_marker}). '
                'Przechodzę do detekcji markera na pudełku.'
            )
            self._transition_to(State.DETECT_MARKER)
        elif self._is_timeout('search_table'):
            self._log(
                'Timeout w stanie SEARCH_TABLE. Wymuszam przejście do DETECT_MARKER '
                'aby kontynuować scenariusz.'
            )
            self._transition_to(State.DETECT_MARKER)
        else:
            expected = self._pickup_table_marker
            current = self._last_marker_id
            self._log(
                f'Szukam stołu startowego (oczekiwany marker ID={expected}, '
                f'ostatnio wykryty: {current}). '
                'Upewnij się, że marker stołu jest widoczny dla kamery.'
            )

    def _handle_detect_marker(self):
        """Stan: wykrywanie markera na pudełku."""
        if self._last_marker_id == self._box_marker_id and self._last_pose is not None:
            self._log(
                f'Wykryto marker pudełka (ID={self._box_marker_id}). '
                'Przechodzę do wyrównywania pozycji.'
            )
            self._transition_to(State.ALIGN_WITH_BOX)
        elif self._is_timeout('detect_marker'):
            self._log(
                'Timeout w stanie DETECT_MARKER. Wymuszam przejście do ALIGN_WITH_BOX.'
            )
            self._transition_to(State.ALIGN_WITH_BOX)
        else:
            self._log(
                f'Oczekuję na marker pudełka (ID={self._box_marker_id}). '
                'Upewnij się, że marker na pudełku jest widoczny dla kamery ciała.'
            )

    def _handle_align_with_box(self):
        """Stan: wyrównanie pozycji z pudełkiem używając korekcji offsetu."""
        if self._last_offset is None:
            self._log(
                'Oczekuję na dane offsetu korekcji. '
                'Sprawdź, czy węzeł estymacji pozy markerów jest uruchomiony.'
            )
            return
        dx, dy, dz = self._last_offset
        # Wyrównanie zakończone gdy offset mieści się w progu
        if abs(dx) < self._align_threshold and abs(dz) < self._align_threshold:
            self._log(
                f'Wyrównanie zakończone (|dx|={abs(dx):.4f}, |dz|={abs(dz):.4f} m, '
                f'próg={self._align_threshold:.4f} m). '
                'Przechodzę do podniesienia pudełka.'
            )
            self._transition_to(State.PICK_BOX)
        elif self._is_timeout('align_with_box'):
            self._log(
                'Timeout w stanie ALIGN_WITH_BOX. Wymuszam przejście do PICK_BOX.'
            )
            self._transition_to(State.PICK_BOX)
        else:
            self._log(
                f'Trwa wyrównywanie: dx={dx:.4f}, dz={dz:.4f} m '
                f'(wymagany próg: {self._align_threshold:.4f} m). '
                'Czekam na dalszą korektę pozycji.'
            )

    def _handle_pick_box(self):
        """Stan: podniesienie pudełka (sekwencja zakończona)."""
        # W pełnej implementacji: czekamy na zakończenie sekwencji pick_box
        # W MVP: przejście natychmiastowe po wykonaniu sekwencji
        self._log('Sekwencja podniesienia pudełka wykonana. Przechodzę do obrotu.')
        self._transition_to(State.ROTATE_180)

    def _handle_rotate_180(self):
        """Stan: obrót o 180 stopni (sekwencja zakończona)."""
        # W pełnej implementacji: czekamy na zakończenie sekwencji rotate_180
        self._log('Obrót o 180° wykonany. Przechodzę do nawigacji do celu.')
        self._transition_to(State.NAVIGATE_TO_TARGET_MARKER)

    def _handle_navigate_to_target(self):
        """Stan: nawigacja do drugiego stołu przez marker docelowy."""
        if self._last_marker_id == self._place_table_marker and self._last_pose:
            _, _, z = self._last_pose
            # Zatrzymaj gdy marker jest wystarczająco blisko
            if z < self._stop_distance:
                self._log(
                    f'Dotarłem do docelowego stołu '
                    f'(odległość z={z:.3f} m < próg={self._stop_distance:.3f} m). '
                    'Przechodzę do odkładania pudełka.'
                )
                self._transition_to(State.PLACE_BOX)
            else:
                self._log(
                    f'Zbliżam się do docelowego stołu: '
                    f'z={z:.3f} m (wymagane < {self._stop_distance:.3f} m).'
                )
        elif self._is_timeout('navigate_to_target_marker'):
            self._log(
                'Timeout w stanie NAVIGATE_TO_TARGET_MARKER. '
                'Wymuszam przejście do PLACE_BOX.'
            )
            self._transition_to(State.PLACE_BOX)
        else:
            self._log(
                f'Szukam markera docelowego stołu (ID={self._place_table_marker}). '
                'Upewnij się, że marker docelowego stołu jest widoczny dla kamery głowy.'
            )

    def _handle_place_box(self):
        """Stan: odłożenie pudełka na drugi stół."""
        self._log('Sekwencja odkładania pudełka wykonana. Scenariusz zakończony.')
        self._transition_to(State.FINISHED)

    def _log(self, message: str):
        """Loguje wiadomość przez logger ROS2 lub stdout."""
        if self._logger:
            self._logger.info(f'[StateMachine] {message}')
        else:
            print(f'[StateMachine] {message}')
