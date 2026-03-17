#!/usr/bin/env python3
"""Interfejs sprzętowy robota Unitree G1 EDU.

Zapewnia integrację z Unitree SDK 2 (``unitree_sdk2py``):
- inicjalizację połączenia DDS (``ChannelFactoryInitialize``)
- sterowanie lokomocją przez ``LocoClient`` (chodzenie, obroty)
- sterowanie ramionami przez ``G1ArmActionClient`` (predefiniowane gesty)

Klasa ``UnitreeRobotAPI`` jest używana wyłącznie w trybie ``robot_mode``.
W trybie ``demo_mode`` przekazuje się ``None`` jako ``robot_api``
do ``execute_sequence``, dzięki czemu sekwencje są tylko logowane.

Wymagania (tylko tryb robot):
    pip install unitree_sdk2py

Przykład użycia:
    api = UnitreeRobotAPI(network_interface='eth0')
    api.connect()
    api.move_to_pose({'x': 0.0, 'y': 0.5, 'z': 0.0, 'yaw': 0.0})
    api.execute_arm_action('wave hand')
    api.disconnect()
"""

import math
import time


class UnitreeRobotAPI:
    """Interfejs sprzętowy dla robota Unitree G1 EDU.

    Zarządza połączeniem z robotem przez Unitree SDK 2 i udostępnia
    metody do sterowania lokomocją i ramionami.  Realizuje sterowanie oparte na:
    - komendach prędkości ``LocoClient.SetVelocity()`` z parametrem ``duration``
    - predefiniowanych akcjach ramion ``G1ArmActionClient.ExecuteAction()``

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
        self._arm_action_client = None
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

        Inicjalizuje transport DDS, klienta lokomocji LocoClient
        oraz klienta akcji ramion G1ArmActionClient.
        Po inicjalizacji przełącza robota w tryb chodzenia (Start).

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
            from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient  # noqa: PLC0415
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

            self._arm_action_client = G1ArmActionClient()
            self._arm_action_client.SetTimeout(10.0)
            self._arm_action_client.Init()

            # Przełącz robota w tryb chodzenia (FSM ID = 200)
            self._loco_client.Start()
            self._sdk_available = True
            self._log(
                logger,
                'info',
                f'Połączono z robotem przez interfejs sieciowy: {self._network_interface}. '
                'Robot w trybie chodzenia (Start).',
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

        Najpierw zatrzymuje ruch (StopMove), potem przełącza robota
        w tryb tłumienia (Damp) i zwalnia ramiona (release arm).
        Bezpieczna do wywołania nawet gdy robot nie jest połączony.
        """
        if self._loco_client is not None:
            try:
                self._loco_client.StopMove()
            except Exception:
                pass
            try:
                self._loco_client.Damp()
            except Exception:
                pass
            self._loco_client = None

        if self._arm_action_client is not None:
            try:
                from unitree_sdk2py.g1.arm.g1_arm_action_client import action_map  # noqa: PLC0415
                release_id = action_map.get('release arm')
                if release_id is not None:
                    self._arm_action_client.ExecuteAction(release_id)
            except Exception:
                pass
            self._arm_action_client = None

        self._sdk_available = False

    def move_to_pose(self, pose: dict, timeout_s: float = 5.0) -> None:
        """Przemieszcza robota do podanej bezwzględnej pozy.

        Realizacja:
        1. Obrót w miejscu do żądanej orientacji (yaw).
        2. Translacja do żądanej pozycji (x, y).

        Komendy prędkości są wysyłane przez ``LocoClient.SetVelocity()``
        z parametrem ``duration`` odpowiadającym czasowi trwania ruchu.
        SDK obsługuje czas trwania po stronie serwera – robot wykonuje
        ruch przez zadany czas bez konieczności wysyłania poleceń w pętli.

        Komponent ``z`` (wysokość ramienia) nie jest obsługiwany przez
        lokomocję – należy go zrealizować przez ``execute_arm_action()``
        lub niskopoziomowe API ramienia SDK (arm_sdk DDS).

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
            self._send_velocity(
                vx=0.0,
                vy=0.0,
                vyaw=math.copysign(self._YAW_SPEED, dyaw),
                duration_s=allocated_rot,
            )
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
            # Unitree LocoClient.SetVelocity(vx, vy, vyaw, duration):
            #   vx – prędkość do przodu [m/s]
            #   vy – prędkość boczna [m/s] (+ = lewo)
            #   vyaw – prędkość kątowa [rad/s]
            vx = (dy / distance) * self._LINEAR_SPEED
            vy = (dx / distance) * self._LINEAR_SPEED
            self._send_velocity(
                vx=vx,
                vy=vy,
                vyaw=0.0,
                duration_s=min(move_time, remaining * self._TRANSLATION_TIMEOUT_FRACTION),
            )

        self._current_x = target_x
        self._current_y = target_y

    def execute_arm_action(self, action_name: str) -> int:
        """Wykonuje predefiniowaną akcję ramion robota.

        Dostępne akcje (z ``G1ArmActionClient.action_map``):
            'release arm', 'two-hand kiss', 'left kiss', 'right kiss',
            'hands up', 'clap', 'high five', 'hug', 'heart',
            'right heart', 'reject', 'right hand up', 'x-ray',
            'face wave', 'high wave', 'shake hand'

        Args:
            action_name: Nazwa akcji do wykonania.

        Returns:
            Kod błędu z SDK (0 = sukces).

        Raises:
            RuntimeError: Jeśli SDK nie jest zainicjalizowane.
            ValueError: Jeśli nazwa akcji nie jest rozpoznana.
        """
        if not self._sdk_available or self._arm_action_client is None:
            raise RuntimeError(
                'SDK Unitree nie jest zainicjalizowane. '
                'Wywołaj connect() przed execute_arm_action().'
            )

        try:
            from unitree_sdk2py.g1.arm.g1_arm_action_client import action_map  # noqa: PLC0415
        except ImportError as exc:
            raise RuntimeError(f'Brak modułu arm_action_client: {exc}') from exc

        action_id = action_map.get(action_name)
        if action_id is None:
            raise ValueError(
                f'Nieznana akcja ramion: {action_name!r}. '
                f'Dostępne akcje: {list(action_map.keys())}'
            )

        return self._arm_action_client.ExecuteAction(action_id)

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

    def _send_velocity(
        self,
        vx: float,
        vy: float,
        vyaw: float,
        duration_s: float,
    ) -> None:
        """Wysyła komendę prędkości ``LocoClient.SetVelocity`` z zadanym czasem trwania.

        SDK Unitree obsługuje parametr ``duration`` po stronie serwera –
        robot wykonuje ruch przez zadany czas. Dodatkowo, po upływie czasu
        wywołujemy ``StopMove()`` aby wyzerować prędkość (środek bezpieczeństwa).

        Args:
            vx: Prędkość do przodu/tyłu [m/s].
            vy: Prędkość boczna [m/s].
            vyaw: Prędkość kątowa [rad/s].
            duration_s: Czas trwania komendy [s].
        """
        if duration_s <= 0.0:
            return

        self._loco_client.SetVelocity(vx, vy, vyaw, duration_s)
        time.sleep(duration_s)
        self._loco_client.StopMove()
