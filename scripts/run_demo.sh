#!/usr/bin/env bash
# Skrypt pomocniczy do uruchamiania demonstracji RoboMVP
# Buduje pakiety ROS2, ładuje środowisko i uruchamia demo

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/../ros2_ws"

echo "=== Budowanie pakietów ROS2 ==="
cd "$WS_DIR"
colcon build --symlink-install

echo "=== Ładowanie środowiska ==="
source "$WS_DIR/install/setup.bash"

echo "=== Uruchamianie demonstracji ==="
ros2 launch robomvp_bringup demo.launch.py
