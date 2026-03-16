#!/usr/bin/env python3
"""Interfejs sprzętowy robota Unitree G1 EDU.

Zapewnia integrację z Unitree SDK 2 (unitree_sdk2py):
- Sterowanie lokomocją (chodzenie, obroty) przez LocoClient
- Inicjalizację połączenia DDS z robotem

Wymaga zainstalowanego pakietu unitree_sdk2py oraz podłączonego
i zasilonego robota przez sieć Ethernet.

Instalacja SDK:
    pip install unitree_sdk2py

Przykład użycia:
    api = UnitreeRobotAPI(network_interface='eth0')
    api.connect()
    api.move_to_pose({'x': 0.0, 'y': 0.5, 'z': 0.0, 'yaw': 0.0})
    api.disconnect()
"""

import math
import time


class UnitreeRobotAPI:
    """Interfejs sprzętowy dla robota Unitree G1 EDU.

    Zarządza połączeniem z robotem przez Unitree SDK 2 i udostępnia
    metody do sterowania lokomocją.  Realizuje sterowanie oparte na
    komendach prędkości (Move/StopMove) LocoClient.

    Konwencja osi waypoints:
        x  – ruch boczny (metry, dodatni = lewo)
        y  – ruch do przodu (metry, dodatni = przód)
        z  – wysokość ramienia (metry, używana przez wyższe warstwy)
        yaw – orientacja (radiany, dodatni = obrót w lewo)
    """

    # Parametry sterowania
    _LINEAR_SPEED: float = 0.3   # m/s – prędkość translacyjna
    _YAW_SPEED: float = 0.5      # rad/s – prędkość obrotowa
    _POSITION_TOLERANCE: float = 0.02  # m – tolerancja osiągnięcia pozycji
    _YAW_TOLERANCE: float = 0.05       # rad – tolerancja osiągnięcia orientacji
    # Ułamki budżetu czasu timeout_s przeznaczone na poszczególne fazy ruchu.
    # Faza obrotu otrzymuje 40%, pozostała część (≤90% reszty) na translację.
    # 10% rezerwy na StopMove i opóźnienia sieci.
    _YAW_TIMEOUT_FRACTION: float = 0.4
    _TRANSLATION_TIMEOUT_FRACTION: float = 0.9

    def __init__(self, network_interface: str = 'eth0') -> None:
        """Inicjalizuje interfejs robota.

        Args:
            network_interface: Nazwa interfejsu sieciowego Ethernet podłączonego
                do robota (np. 'eth0', 'enp3s0').
        """
        self._network_interface = network_interface
        self._loco_client = None
        self._sdk_available = False

        # Śledzona aktualna poza robota (aktualizowana po każdym ruchu)
        self._current_x: float = 0.0
        self._current_y: float = 0.0
        self._current_yaw: float = 0.0

    # ------------------------------------------------------------------
    # Publiczny interfejs
    # ------------------------------------------------------------------

    def connect(self, logger=None) -> None:
        """Nawiązuje połączenie z robotem przez Unitree SDK.

        Inicjalizuje transport DDS i klienta lokomocji LocoClient.

        Args:
            logger: Logger ROS2 (opcjonalny).  Jeśli podany, komunikaty
                są wysyłane przez rclpy; w przeciwnym razie przez print().

        Raises:
            RuntimeError: Jeśli pakiet unitree_sdk2py nie jest zainstalowany
                lub jeśli połączenie z robotem się nie powiedzie.
        """
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # noqa: PLC0415
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient      # noqa: PLC0415
        except ImportError as exc:
            msg = (
                f'Brak modułu Unitree SDK: {exc}. '
                'Zainstaluj pakiet: pip install unitree_sdk2py. '
                'Bez SDK robot nie może działać w trybie sprzętowym.'
            )
            self._log(logger, 'error', msg)
            raise RuntimeError(msg) from exc

        try:
            ChannelFactoryInitialize(0, self._network_interface)
            self._loco_client = LocoClient()
            self._loco_client.SetTimeout(10.0)
            self._loco_client.Init()
            self._sdk_available = True
            self._log(
                logger,
                'info',
                f'Połączono z robotem przez interfejs sieciowy: {self._network_interface}.',
            )
        except Exception as exc:
            msg = (
                f'Błąd inicjalizacji SDK Unitree na interfejsie {self._network_interface}: '
                f'{exc}. Sprawdź połączenie sieciowe i stan robota.'
            )
            self._log(logger, 'error', msg)
            raise RuntimeError(msg) from exc

    def disconnect(self) -> None:
        """Rozłącza połączenie i zatrzymuje robota.

        Bezpieczna do wywołania nawet gdy robot nie jest połączony.
        """
        if self._loco_client is not None:
            try:
                self._loco_client.StopMove()
            except Exception:
                pass
            self._loco_client = None
        self._sdk_available = False

    def move_to_pose(self, pose: dict, timeout_s: float = 5.0) -> None:
        """Przemieszcza robota do podanej bezwzględnej pozy.

        Realizacja:
        1. Obrót w miejscu do żądanej orientacji (yaw).
        2. Translacja do żądanej pozycji (x, y).

        Czas przeznaczony na każdą fazę jest proporcjonalny do odległości /
        kąta, lecz nie przekracza odpowiedniego ułamka ``timeout_s``.

        Komponent ``z`` (wysokość ramienia) nie jest obsługiwany przez ten
        moduł – należy go zrealizować przez dedykowany klient API ramienia.

        Args:
            pose: Słownik z kluczami 'x', 'y', 'z', 'yaw'.
                  Brakujące klucze są traktowane jako 0.0 /
                  aktualna wartość dla yaw.
            timeout_s: Maksymalny łączny czas wykonania kroku (sekundy).

        Raises:
            RuntimeError: Jeśli SDK nie zostało zainicjalizowane przez connect().
        """
        if not self._sdk_available or self._loco_client is None:
            raise RuntimeError(
                'SDK Unitree nie jest zainicjalizowane. '
                'Wywołaj connect() przed move_to_pose().'
            )

        target_x = float(pose.get('x', 0.0))
        target_y = float(pose.get('y', 0.0))
        target_yaw = float(pose.get('yaw', self._current_yaw))

        start = time.monotonic()

        # Faza 1: Obrót do żądanej orientacji
        dyaw = target_yaw - self._current_yaw
        # Normalizacja do [-π, π]
        dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi
        if abs(dyaw) > self._YAW_TOLERANCE:
            rotation_time = abs(dyaw) / self._YAW_SPEED
            allocated_rot = min(rotation_time, timeout_s * self._YAW_TIMEOUT_FRACTION)
            self._loco_client.Move(0.0, 0.0, math.copysign(self._YAW_SPEED, dyaw))
            time.sleep(allocated_rot)
            self._loco_client.StopMove()
            self._current_yaw = target_yaw

        # Faza 2: Translacja do żądanej pozycji
        dx = target_x - self._current_x
        dy = target_y - self._current_y
        distance = math.hypot(dx, dy)

        if distance > self._POSITION_TOLERANCE:
            elapsed = time.monotonic() - start
            remaining = max(0.0, timeout_s - elapsed)
            move_time = distance / self._LINEAR_SPEED
            # Mapowanie: y → vx (przód/tył), x → vy (bok)
            # Unitree LocoClient.Move(vx, vy, vyaw):
            #   vx – prędkość do przodu [m/s]
            #   vy – prędkość boczna [m/s] (+ = lewo)
            #   vyaw – prędkość kątowa [rad/s]
            vx = (dy / distance) * self._LINEAR_SPEED
            vy = (dx / distance) * self._LINEAR_SPEED
            self._loco_client.Move(vx, vy, 0.0)
            time.sleep(min(move_time, remaining * self._TRANSLATION_TIMEOUT_FRACTION))
            self._loco_client.StopMove()

        self._current_x = target_x
        self._current_y = target_y

    @property
    def is_connected(self) -> bool:
        """Zwraca True, jeśli SDK jest zainicjalizowane i połączone."""
        return self._sdk_available and self._loco_client is not None

    # ------------------------------------------------------------------
    # Metody wewnętrzne
    # ------------------------------------------------------------------

    @staticmethod
    def _log(logger, level: str, message: str) -> None:
        """Loguje wiadomość przez logger ROS2 lub stdout."""
        if logger is not None:
            getattr(logger, level)(message)
        else:
            print(f'[UnitreeRobotAPI/{level.upper()}] {message}')
