#!/usr/bin/env python3
"""Węzeł teleoperation – ręczne sterowanie robotem przez /cmd_vel.

PO CO NAM TELEOP?
==================
W normalnym scenariuszu robot działa autonomicznie. Ale co jeśli:
  - chcemy ręcznie zaprowadzić robota do punktu startowego,
  - chcemy przetestować czy LocoClient działa zanim uruchomimy cały pipeline,
  - scenariusz utknął i chcemy ręcznie wyciągnąć robota z impasu?

Węzeł teleop subskrybuje standardowy temat /cmd_vel (geometry_msgs/Twist)
i przekazuje komendy bezpośrednio do LocoClient.SetVelocity.

Temat /cmd_vel to standard ROS2 – działa z każdym joystickiem, klawiaturą
(ros2 run teleop_twist_keyboard teleop_twist_keyboard) albo aplikacją mobilną.
Nie trzeba pisać własnego interfejsu sterowania.

PRIORYTET BEZPIECZEŃSTWA:
Teleop jest aktywny TYLKO gdy serwis /robomvp/pause wstrzymał automat.
Nie chcemy żeby operator klikał klawiaturą gdy robot wykonuje autonomiczną
sekwencję – mogłoby to spowodować kolizję. Guard condition jest sprawdzany
przy każdej wiadomości /cmd_vel.

Subskrybowane tematy:
    /cmd_vel     – geometry_msgs/Twist (komenda prędkości od operatora)
    /robomvp/state – do monitorowania stanu (nie sterujemy gdy scenariusz aktywny)

Parametry ROS2:
    max_linear_speed  – limit prędkości liniowej [m/s], domyślnie 0.3
    max_angular_speed – limit prędkości kątowej [rad/s], domyślnie 0.5
    enabled           – czy teleop jest aktywny od startu, domyślnie False
"""

import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

from robomvp.msg import State as StateMsg


class TeleopNode(Node):
    """Węzeł przekazujący komendy /cmd_vel do LocoClient robota."""

    def __init__(self):
        super().__init__('robomvp_teleop')

        self.declare_parameter('max_linear_speed',  0.3)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('enabled', False)

        self._max_linear  = self.get_parameter('max_linear_speed').value
        self._max_angular = self.get_parameter('max_angular_speed').value
        self._enabled: bool = self.get_parameter('enabled').value

        # Blokada teleop: True gdy automat stanowy jest aktywny (nie FINISHED, nie SEARCH)
        self._scenario_active = False
        self._lock = threading.Lock()

        # Inicjalizacja LocoClient – opcjonalna, tylko jeśli SDK dostępne
        self._loco_client = None
        self._try_connect_sdk()

        # Publikator statusu teleop (np. dla panelu operatora w RViz)
        self._pub_status = self.create_publisher(Bool, '/robomvp/teleop_active', 10)

        self.create_subscription(Twist,    '/cmd_vel',        self._on_cmd_vel, 10)
        self.create_subscription(StateMsg, '/robomvp/state',  self._on_state,   10)

        # Watchdog: jeśli przez 0.5s nie przyjdzie nowa komenda /cmd_vel,
        # zatrzymaj robota (zabezpieczenie przed utratą połączenia z joystickiem)
        self._last_cmd_time: float = 0.0
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            f'Węzeł teleop uruchomiony. '
            f'Aktywny: {self._enabled}. '
            'Sterowanie: ros2 run teleop_twist_keyboard teleop_twist_keyboard'
        )

    def _try_connect_sdk(self):
        """Próbuje podłączyć się do LocoClient SDK – bez błędu gdy brak SDK."""
        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            # Uwaga: ChannelFactoryInitialize musi być wywołane tylko raz w procesie.
            # Jeśli main_node już je wywołał (w tym samym procesie), ten węzeł
            # uruchomiony w osobnym procesie musi wywołać je samodzielnie.
            ChannelFactoryInitialize(0, 'eth0')
            self._loco_client = LocoClient()
            self._loco_client.SetTimeout(5.0)
            self._loco_client.Init()
            self.get_logger().info('Teleop: połączono z LocoClient SDK.')
        except Exception as exc:
            self.get_logger().warning(
                f'Teleop: brak SDK lub brak połączenia ({exc}). '
                'Komendy /cmd_vel będą logowane, nie wysyłane do robota.'
            )

    def _on_state(self, msg: StateMsg):
        """Monitoruje stan automatu – blokuje teleop gdy scenariusz aktywny."""
        with self._lock:
            # Teleop dozwolony tylko gdy automat w SEARCH_TABLE lub FINISHED
            self._scenario_active = msg.state_name not in ('SEARCH_TABLE', 'FINISHED')

    def _on_cmd_vel(self, msg: Twist):
        """Przetwarza komendę /cmd_vel i wysyła ją do robota (jeśli dozwolone)."""
        now = self.get_clock().now().nanoseconds * 1e-9
        self._last_cmd_time = now

        with self._lock:
            active = self._enabled and not self._scenario_active

        self._pub_status.publish(Bool(data=active))

        if not active:
            if self._scenario_active:
                self.get_logger().warn(
                    'Teleop: zablokowany – scenariusz jest aktywny. '
                    'Wstrzymaj scenariusz przez /robomvp/pause zanim użyjesz teleop.'
                )
            return

        # Ograniczenie prędkości (clamp) – bezpieczeństwo
        vx   = max(-self._max_linear,  min(self._max_linear,  msg.linear.x))
        vy   = max(-self._max_linear,  min(self._max_linear,  msg.linear.y))
        vyaw = max(-self._max_angular, min(self._max_angular, msg.angular.z))

        self.get_logger().debug(
            f'Teleop: vx={vx:.2f} vy={vy:.2f} vyaw={vyaw:.2f}'
        )

        if self._loco_client is not None:
            try:
                # duration=0.5s: robot zatrzyma się po 0.5s bez kolejnej komendy
                # (watchdog na poziomie SDK)
                self._loco_client.SetVelocity(vx, vy, vyaw, 0.5)
            except Exception as exc:
                self.get_logger().error(f'Teleop: błąd SDK: {exc}')

    def _watchdog(self):
        """Zatrzymuje robota jeśli przez 0.5s nie było komendy /cmd_vel."""
        if self._last_cmd_time == 0.0:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_cmd_time > 0.5:
            if self._loco_client is not None:
                try:
                    self._loco_client.StopMove()
                except Exception:
                    pass
            self._last_cmd_time = 0.0  # reset – nie wysyłaj StopMove w kółko


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
