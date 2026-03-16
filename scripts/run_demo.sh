#!/usr/bin/env bash
# Skrypt pomocniczy do uruchamiania RoboMVP na rzeczywistym sprzęcie
# Buduje pakiety ROS2, ładuje środowisko i uruchamia system

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/../ros2_ws"

echo "=== Budowanie pakietów ROS2 ==="
cd "$WS_DIR"
colcon build --symlink-install

echo "=== Ładowanie środowiska ==="
source "$WS_DIR/install/setup.bash"

echo "=== Uruchamianie RoboMVP (kamery rzeczywiste) ==="
ros2 launch robomvp demo.launch.py
