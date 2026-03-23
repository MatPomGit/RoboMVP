#!/usr/bin/env bash
# ============================================================================
# RoboMVP – uruchomienie pełnego scenariusza
# Użycie: ./scripts/run_demo.sh [opcje]
#
# Opcje:
#   --robot          Połącz z fizycznym robotem (domyślnie: tryb demo)
#   --interface ETH  Interfejs Ethernet (domyślnie: eth0)
#   --body-cam N     Indeks /dev/videoN kamery ciała (domyślnie: 0)
#   --head-cam N     Indeks /dev/videoN kamery głowy; -1 = brak (domyślnie: -1)
#   --apriltag       Użyj AprilTag (domyślnie: tak)
#   --qr             Użyj QR zamiast AprilTag
#   --teleop         Aktywuj ręczne sterowanie /cmd_vel
#   --step T         Okres kroku automatu w sekundach (domyślnie: 1.0)
#   --rviz           Uruchom RViz2 z predefiniowaną konfiguracją
# ============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$REPO_DIR/ros2_ws"

# Domyślne parametry
ROBOT_MODE=false
NETWORK_INTERFACE="eth0"
BODY_CAM=0
HEAD_CAM=-1
MARKER_TYPE="apriltag"
TELEOP=false
STEP_PERIOD="1.0"
LAUNCH_RVIZ=false

# Parsowanie argumentów
while [[ $# -gt 0 ]]; do
    case "$1" in
        --robot)         ROBOT_MODE=true; shift ;;
        --interface)     NETWORK_INTERFACE="$2"; shift 2 ;;
        --body-cam)      BODY_CAM="$2"; shift 2 ;;
        --head-cam)      HEAD_CAM="$2"; shift 2 ;;
        --qr)            MARKER_TYPE="qr"; shift ;;
        --teleop)        TELEOP=true; shift ;;
        --step)          STEP_PERIOD="$2"; shift 2 ;;
        --rviz)          LAUNCH_RVIZ=true; shift ;;
        *) echo "Nieznana opcja: $1" && exit 1 ;;
    esac
done

echo "========================================"
echo "  RoboMVP v0.2.0"
echo "  Tryb: $([ "$ROBOT_MODE" = true ] && echo 'ROBOT' || echo 'DEMO')"
echo "  Interfejs: $NETWORK_INTERFACE"
echo "  Kamera ciała: /dev/video$BODY_CAM"
echo "  Kamera głowy: $([ "$HEAD_CAM" = "-1" ] && echo 'brak (używa ciała)' || echo "/dev/video$HEAD_CAM")"
echo "  Markery: $MARKER_TYPE"
echo "  Teleop: $TELEOP"
echo "  Krok automatu: ${STEP_PERIOD}s"
echo "========================================"

# Budowanie
echo ""
echo ">>> Budowanie pakietów ROS2..."
cd "$WS_DIR"
colcon build --symlink-install --packages-select robomvp 2>&1 | tail -5

echo ">>> Ładowanie środowiska..."
source "$WS_DIR/install/setup.bash"

# Opcjonalny RViz w tle
if [ "$LAUNCH_RVIZ" = true ]; then
    RVIZ_CFG="$REPO_DIR/rviz/robomvp.rviz"
    echo ">>> Uruchamianie RViz2..."
    ros2 run rviz2 rviz2 -d "$RVIZ_CFG" &
    RVIZ_PID=$!
    sleep 2
fi

echo ""
echo ">>> Uruchamianie systemu RoboMVP..."
ros2 launch robomvp demo.launch.py \
    network_interface:="$NETWORK_INTERFACE" \
    body_camera_device:="$BODY_CAM" \
    head_camera_device:="$HEAD_CAM" \
    marker_type:="$MARKER_TYPE" \
    teleop_enabled:="$TELEOP" \
    step_period:="$STEP_PERIOD" \
    require_robot_connection:="$ROBOT_MODE"

# Czyszczenie po zakończeniu
if [ "$LAUNCH_RVIZ" = true ] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill "$RVIZ_PID" 2>/dev/null || true
fi
