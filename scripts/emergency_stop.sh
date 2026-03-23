#!/usr/bin/env bash
# RoboMVP – natychmiastowe zatrzymanie robota przez serwis ROS2
# Użycie: ./scripts/emergency_stop.sh
set -euo pipefail

echo "!!! EMERGENCY STOP !!!"
echo "Wysyłanie komendy zatrzymania do /robomvp/emergency_stop..."

if ros2 service call /robomvp/emergency_stop std_srvs/srv/Trigger '{}' 2>&1; then
    echo ""
    echo "✓ Robot zatrzymany."
    echo "  Aby wznowić: ros2 service call /robomvp/reset std_srvs/srv/Trigger '{}'"
else
    echo ""
    echo "✗ Serwis niedostępny – czy system RoboMVP jest uruchomiony?"
    echo "  Awaryjne zatrzymanie: odłącz kabel Ethernet od robota."
    exit 1
fi
