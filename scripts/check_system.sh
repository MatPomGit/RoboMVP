#!/usr/bin/env bash
# RoboMVP – weryfikacja prereqs przed uruchomieniem
# Użycie: ./scripts/check_system.sh
set -euo pipefail

OK=0; WARN=0; ERR=0

check() {
    local label="$1"; local cmd="$2"; local expected="$3"
    if eval "$cmd" &>/dev/null; then
        echo "  ✓ $label"
        ((OK++)) || true
    else
        echo "  ✗ $label  ← $expected"
        ((ERR++)) || true
    fi
}

warn() {
    local label="$1"; local cmd="$2"; local hint="$3"
    if eval "$cmd" &>/dev/null; then
        echo "  ✓ $label"
        ((OK++)) || true
    else
        echo "  ? $label  ← $hint (opcjonalne)"
        ((WARN++)) || true
    fi
}

echo "========================================"
echo "  RoboMVP – weryfikacja środowiska"
echo "========================================"

echo ""
echo "System:"
check "Ubuntu 22.04"        "grep -q 'Ubuntu 22' /etc/os-release"   "wymagany Ubuntu 22.04"
check "Python 3.10+"        "python3 -c 'import sys; assert sys.version_info >= (3,10)'" "python3.10+"
check "ROS2 Humble"         "source /opt/ros/humble/setup.bash && ros2 --version"  "zainstaluj ROS2 Humble"

echo ""
echo "Zależności Python:"
check "opencv-python"       "python3 -c 'import cv2'"       "pip install opencv-python"
check "numpy"               "python3 -c 'import numpy'"     "pip install numpy"
check "pyyaml"              "python3 -c 'import yaml'"      "pip install pyyaml"
warn  "apriltag"            "python3 -c 'import apriltag'"  "pip install apriltag (fallback: QR)"
warn  "unitree_sdk2py"      "python3 -c 'import unitree_sdk2py'" "pip install unitree_sdk2py (wymagane z robotem)"

echo ""
echo "ROS2 pakiety:"
check "cv_bridge"           "python3 -c 'from cv_bridge import CvBridge'"  "sudo apt install ros-humble-cv-bridge"
check "tf2_ros"             "python3 -c 'import tf2_ros'"   "sudo apt install ros-humble-tf2-ros"
check "diagnostic_msgs"     "ros2 interface show diagnostic_msgs/msg/DiagnosticArray" "sudo apt install ros-humble-common-interfaces"
check "nav_msgs"            "ros2 interface show nav_msgs/msg/Odometry"     "sudo apt install ros-humble-common-interfaces"
check "std_srvs"            "ros2 interface show std_srvs/srv/Trigger"      "sudo apt install ros-humble-common-interfaces"

echo ""
echo "Pakiet RoboMVP:"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")/ros2_ws"
warn  "colcon build"        "test -d '$WS_DIR/install/robomvp'"  "cd ros2_ws && colcon build"

echo ""
echo "Podsumowanie: ✓ $OK  ? $WARN  ✗ $ERR"
if [ "$ERR" -gt 0 ]; then
    echo "Rozwiąż błędy (✗) przed uruchomieniem systemu."
    exit 1
fi
echo "System gotowy do uruchomienia!"
