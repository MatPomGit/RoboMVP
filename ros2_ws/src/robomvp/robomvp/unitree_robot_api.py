#!/usr/bin/env python3
"""Interfejs sprzętowy robota Unitree G1 EDU.

MIEJSCE TEGO MODUŁU W ARCHITEKTURZE:
=====================================
Ten plik to „tłumacz" między abstrakcyjnym językiem RoboMVP
(„przesuń się do pozy {'x': 0.5, 'y': 1.0, 'yaw': 0}") a konkretnym
protokołem DDS Unitree SDK 2 (SetVelocity z prędkością i czasem).

INTEGRACJA Z ODOMETRIĄ (velocity_callback):
============================================
Węzeł robomvp_odometry.py całkuje prędkości po czasie (dead reckoning),
ale żeby całkowanie było zsynchronizowane z rzeczywistymi komendami SDK,
potrzebuje wiedzieć dokładnie kiedy prędkości się zmieniają.

UnitreeRobotAPI akceptuje opcjonalny ``velocity_callback`` – callable
wywoływany przy każdej zmianie prędkości (przed SetVelocity i po StopMove).

Podłączenie:
    api = UnitreeRobotAPI(velocity_callback=odometry_node.set_velocity)

Przepływ przy move_to_pose():
    velocity_callback(vx, vy, vyaw)   ← OdometryNode.set_velocity() [start całkowania]
    SetVelocity(vx, vy, vyaw, dur)     ← robot zaczyna ruch
    time.sleep(dur)
    StopMove()                         ← robot zatrzymuje się
    velocity_callback(0, 0, 0)         ← OdometryNode zatrzymuje całkowanie

Dzięki temu odometria jest precyzyjnie zsynchronizowana z ruchem robota
bez konieczności subskrybowania żadnego dodatkowego tematu ROS2.

KONWENCJA OSI WAYPOINTS:
    x   – ruch boczny [m]       (+ = lewo)
    y   – ruch do przodu [m]    (+ = przód)
    z   – wysokość ramienia [m] (dla G1ArmActionClient)
    yaw – orientacja [rad]      (+ = obrót w lewo)
"""

import math
import time
from typing import Callable, Optional


class UnitreeRobotAPI:
    """Interfejs sprzętowy dla robota Unitree G1 EDU."""

    _LINEAR_SPEED:               float = 0.3    # m/s
    _YAW_SPEED:                  float = 0.5    # rad/s
    _POSITION_TOLERANCE:         float = 0.02   # m
    _YAW_TOLERANCE:              float = 0.05   # rad
    _YAW_TIMEOUT_FRACTION:       float = 0.4
    _TRANSLATION_TIMEOUT_FRACTION: float = 0.9

    def __init__(
        self,
        network_interface: str = 'eth0',
        velocity_callback: Optional[Callable[[float, float, float], None]] = None,
        stop_callback:     Optional[Callable[[], None]] = None,
    ) -> None:
        """Inicjalizuje interfejs robota.

        Args:
            network_interface: Nazwa interfejsu Ethernet (np. 'eth0').
                Sprawdź: ip link show
            velocity_callback: Wywoływany przy każdej SetVelocity z (vx, vy, vyaw).
                Używany przez OdometryNode.set_velocity() do śledzenia prędkości.
            stop_callback: Wywoływany przy StopMove (zerowanie prędkości).
                Jeśli None, wywołuje velocity_callback(0, 0, 0).
        """
        self._network_interface = network_interface
        self._velocity_cb       = velocity_callback
        self._stop_cb           = stop_callback
        self._loco_client       = None
        self._arm_action_client = None
        self._sdk_available     = False

        # Dead-reckoning pozycja (aktualizowana po każdym ruchu)
        self._current_x:   float = 0.0
        self._current_y:   float = 0.0
        self._current_yaw: float = 0.0

    # ------------------------------------------------------------------
    # Połączenie i rozłączenie
    # ------------------------------------------------------------------

    def connect(self, logger=None) -> None:
        """Nawiązuje połączenie z robotem przez Unitree SDK 2.

        WAŻNE: ChannelFactoryInitialize() musi być wywołane DOKŁADNIE RAZ
        na cały proces (nie per-węzeł, nie per-wątek). Wielokrotne wywołanie
        powoduje błąd inicjalizacji DDS.

        Po inicjalizacji wywołuje Start() – przełącza G1 w Sport Mode
        (FSM ID = 200), który jest wymagany do SetVelocity.

        Raises:
            RuntimeError: gdy unitree_sdk2py nie jest zainstalowane
                lub gdy inicjalizacja DDS się nie powiedzie.
        """
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient
        except ImportError as exc:
            msg = (
                f'Brak modułu Unitree SDK: {exc}. '
                'Zainstaluj: pip install unitree_sdk2py'
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

            self._loco_client.Start()
            self._sdk_available = True
            self._log(
                logger, 'info',
                f'Połączono z robotem ({self._network_interface}). Sport Mode aktywny.'
            )
        except Exception as exc:
            msg = (
                f'Błąd inicjalizacji SDK ({self._network_interface}): {exc}. '
                'Sprawdź kabel Ethernet i stan robota.'
            )
            self._log(logger, 'error', msg)
            raise RuntimeError(msg) from exc

    def disconnect(self) -> None:
        """Zatrzymuje robota i rozłącza SDK.

        Sekwencja bezpiecznego zatrzymania:
        1. StopMove()   – zerowanie prędkości (robot staje w miejscu)
        2. Damp()       – tryb tłumiony (serwomechanizmy rozluźnione)
        3. release arm  – ramiona w pozycji spoczynkowej

        Bezpieczna do wywołania gdy robot nie jest połączony (wszystkie try/except).
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
                from unitree_sdk2py.g1.arm.g1_arm_action_client import action_map
                release_id = action_map.get('release arm')
                if release_id is not None:
                    self._arm_action_client.ExecuteAction(release_id)
            except Exception:
                pass
            self._arm_action_client = None

        self._sdk_available = False

    # ------------------------------------------------------------------
    # Sterowanie lokomocją
    # ------------------------------------------------------------------

    def move_to_pose(self, pose: dict, timeout_s: float = 5.0) -> None:
        """Przemieszcza robota do podanej pozycji absolutnej.

        ALGORYTM (dwufazowy):
        1. Obrót w miejscu do żądanego yaw.
           Delta yaw normalizowana do [-π, π] – wybiera krótszą ścieżkę.
        2. Translacja do (x, y) – wektor dx/dy dekompozycja na vx, vy.

        Każda faza wywołuje velocity_callback przed i po SetVelocity,
        co pozwala OdometryNode na precyzyjne śledzenie prędkości.

        Args:
            pose:      Słownik z kluczami 'x', 'y', 'z', 'yaw' [m, rad].
            timeout_s: Łączny limit czasu dla tego kroku [s].

        Raises:
            RuntimeError: gdy connect() nie było wywołane.
        """
        if not self._sdk_available or self._loco_client is None:
            raise RuntimeError(
                'SDK nie zainicjalizowane – wywołaj connect() przed move_to_pose().'
            )

        target_x   = float(pose.get('x',   0.0))
        target_y   = float(pose.get('y',   0.0))
        target_yaw = float(pose.get('yaw', self._current_yaw))

        start = time.monotonic()

        # ── Faza 1: Obrót ────────────────────────────────────────────────
        dyaw = target_yaw - self._current_yaw
        dyaw = (dyaw + math.pi) % (2.0 * math.pi) - math.pi  # [-π, π]

        if abs(dyaw) > self._YAW_TOLERANCE:
            rot_time      = abs(dyaw) / self._YAW_SPEED
            allocated_rot = min(rot_time, timeout_s * self._YAW_TIMEOUT_FRACTION)
            self._send_velocity(
                vx=0.0, vy=0.0,
                vyaw=math.copysign(self._YAW_SPEED, dyaw),
                duration_s=allocated_rot,
            )
            self._current_yaw = target_yaw

        # ── Faza 2: Translacja ───────────────────────────────────────────
        dx       = target_x - self._current_x
        dy       = target_y - self._current_y
        distance = math.hypot(dx, dy)

        if distance > self._POSITION_TOLERANCE:
            elapsed   = time.monotonic() - start
            remaining = max(0.0, timeout_s - elapsed)
            move_time = distance / self._LINEAR_SPEED

            # Układ waypoints: y=przód, x=bok
            # LocoClient:     vx=przód, vy=bok
            vx = (dy / distance) * self._LINEAR_SPEED
            vy = (dx / distance) * self._LINEAR_SPEED
            self._send_velocity(
                vx=vx, vy=vy, vyaw=0.0,
                duration_s=min(move_time, remaining * self._TRANSLATION_TIMEOUT_FRACTION),
            )

        self._current_x = target_x
        self._current_y = target_y

    # ------------------------------------------------------------------
    # Sterowanie ramionami
    # ------------------------------------------------------------------

    def execute_arm_action(self, action_name: str) -> int:
        """Wykonuje predefiniowaną akcję ramion przez G1ArmActionClient.

        Dostępne akcje (przykłady z action_map w SDK):
            'release arm', 'shake hand', 'hug', 'high five',
            'clap', 'heart', 'hands up', 'face wave', 'high wave'.
        Pełna lista: unitree_sdk2py.g1.arm.g1_arm_action_client.action_map

        Returns:
            Kod błędu z SDK (0 = sukces).

        Raises:
            RuntimeError: gdy SDK nie jest zainicjalizowane.
            ValueError:   gdy action_name nie ma w słowniku SDK.
        """
        if not self._sdk_available or self._arm_action_client is None:
            raise RuntimeError('SDK nie zainicjalizowane.')

        try:
            from unitree_sdk2py.g1.arm.g1_arm_action_client import action_map
        except ImportError as exc:
            raise RuntimeError(f'Brak modułu arm_action_client: {exc}') from exc

        action_id = action_map.get(action_name)
        if action_id is None:
            raise ValueError(
                f'Nieznana akcja: {action_name!r}. Dostępne: {list(action_map.keys())}'
            )
        return self._arm_action_client.ExecuteAction(action_id)

    @property
    def is_connected(self) -> bool:
        """True gdy SDK jest zainicjalizowane i połączone."""
        return self._sdk_available and self._loco_client is not None

    def get_pose(self) -> dict:
        """Zwraca bieżącą estymowaną pozycję (dead reckoning).

        Pozycja może dryfować relative do rzeczywistej – to ograniczenie
        dead reckoning bez zewnętrznego systemu lokalizacji.
        """
        return {'x': self._current_x, 'y': self._current_y, 'yaw': self._current_yaw}

    # ------------------------------------------------------------------
    # Metody wewnętrzne
    # ------------------------------------------------------------------

    @staticmethod
    def _log(logger, level: str, message: str) -> None:
        if logger is not None:
            getattr(logger, level)(message)
        else:
            print(f'[UnitreeRobotAPI/{level.upper()}] {message}')

    def _send_velocity(
        self,
        vx: float, vy: float, vyaw: float,
        duration_s: float,
    ) -> None:
        """Wysyła SetVelocity, czeka, następnie StopMove.

        Powiadamia velocity_callback PRZED i PO ruchu, żeby OdometryNode
        mógł dokładnie zsynchronizować całkowanie z ruchem robota.

        TRY/FINALLY: StopMove i zerowanie callbacku są gwarantowane nawet
        przy KeyboardInterrupt podczas time.sleep.
        """
        if duration_s <= 0.0:
            return

        # Powiadom odometrię o nowych prędkościach
        if self._velocity_cb is not None:
            try:
                self._velocity_cb(vx, vy, vyaw)
            except Exception:
                pass

        self._loco_client.SetVelocity(vx, vy, vyaw, duration_s)
        try:
            time.sleep(duration_s)
        finally:
            try:
                self._loco_client.StopMove()
            except Exception:
                pass
            # Zeruj prędkości w odometrii
            if self._stop_cb is not None:
                try:
                    self._stop_cb()
                except Exception:
                    pass
            elif self._velocity_cb is not None:
                try:
                    self._velocity_cb(0.0, 0.0, 0.0)
                except Exception:
                    pass
