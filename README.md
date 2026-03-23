# RoboMVP v0.2.0

System ROS2 do autonomicznego scenariusza manipulacji na robocie humanoidalnym **Unitree G1 EDU**.

> 📖 **Instrukcja obsługi (PDF/DOCX)**: `RoboMVP_Instrukcja_Obslugi.docx`
> — Instalacja, architektura, opis węzłów, przewodnik operatora, kalibracja, troubleshooting.
>
> 📖 **Dokumentacja techniczna SDK**: [docs/sterowanie_robotem.md](docs/sterowanie_robotem.md)
>
> 📖 **Historia poprawek**: [docs/poprawki_i_architektura.md](docs/poprawki_i_architektura.md)

---

## Scenariusz demonstracyjny

Robot wykonuje 7-etapową sekwencję:

1. Szuka markera stołu startowego (AprilTag ID=21)
2. Wykrywa marker pudełka (AprilTag ID=10)
3. Wyrównuje pozycję z pudełkiem (korekcja offsetu kamery)
4. Podnosi pudełko
5. Obraca się o 180°
6. Idzie do drugiego stołu (marker ID=22/30)
7. Odkłada pudełko

Ruch realizowany przez **predefiniowane sekwencje waypoints** — bez planowania, bez uczenia maszynowego.

---

## Architektura v0.2.0 — 8 węzłów ROS2

```
robomvp_tf_publisher  ──/tf_static──────────────────────────────────────────┐
camera_interface      ──/camera/body/image_raw──────────────────────────┐   │
                      ──/camera/head/image_raw──────────────────────┐   │   │
marker_detection      ──/robomvp/marker_detections──────────────┐   │   │   │
marker_pose_estimator ──/robomvp/marker_pose, /offset───────┐   │   │   │   │
robomvp_odometry      ──/odom, /tf──────────────────────┐   │   │   │   │   │
robomvp_diagnostics   ──/diagnostics───────────────┐    │   │   │   │   │   │
                                                   │    │   │   │   │   │   │
robomvp_main ─── subskrybuje: /marker_pose, /offset│    │   │   │   │   │   │
             ─── publikuje:   /state, /motion_cmd  │    │   │   │   │   │   │
             ─── serwisy:     /pause, /e_stop, /reset   │   │   │   │   │   │
             ─── action:      /manipulation_task        │   │   │   │   │   │
                                                        └───┴───┴───┴───┴───┘
robomvp_teleop ── /cmd_vel ──────────────────────────────────────────────────
```

### Wszystkie tematy ROS2

| Temat | Typ | Hz |
|-------|-----|----|
| `/camera/body/image_raw` | `sensor_msgs/Image` | 10 |
| `/camera/head/image_raw` | `sensor_msgs/Image` | 10 |
| `/robomvp/marker_detections` | `robomvp/MarkerDetection` | ~10 |
| `/robomvp/marker_pose` | `robomvp/MarkerPose` | ~10 |
| `/robomvp/offset` | `robomvp/Offset` | ~10 |
| `/robomvp/state` | `robomvp/State` | 1 |
| `/robomvp/motion_command` | `std_msgs/String` | zdarzeniowy |
| `/odom` | `nav_msgs/Odometry` | 50 |
| `/tf` | `tf2_msgs/TFMessage` | 50 |
| `/tf_static` | `tf2_msgs/TFMessage` | raz |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 |
| `/cmd_vel` | `geometry_msgs/Twist` | zmienny |
| `/robomvp/teleop_active` | `std_msgs/Bool` | zmienny |

### Serwisy ROS2

| Serwis | Typ | Opis |
|--------|-----|------|
| `/robomvp/pause` | `std_srvs/SetBool` | True = wstrzymaj, False = wznów |
| `/robomvp/emergency_stop` | `std_srvs/Trigger` | Natychmiastowe zatrzymanie |
| `/robomvp/reset` | `std_srvs/Trigger` | Reset do SEARCH_TABLE |

### Action Server

`/robomvp/manipulation_task` (`robomvp/ManipulationTask`)
— wykonuje sekwencję z feedbackiem co krok; obsługuje Cancel.

---

## Szybki start

### Wymagania

- Ubuntu 22.04, ROS2 Humble, Python 3.10+
- `pip install opencv-python apriltag numpy pyyaml`
- `sudo apt install ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-common-interfaces`
- `pip install unitree_sdk2py` (tylko z fizycznym robotem)

### Weryfikacja prereqs

```bash
./scripts/check_system.sh
```

### Uruchomienie (tryb demo, bez robota)

```bash
./scripts/run_demo.sh
```

### Uruchomienie z robotem

```bash
./scripts/run_demo.sh --robot --interface eth0 --body-cam 0 --head-cam 1
```

### Uruchomienie z RViz

```bash
./scripts/run_demo.sh --rviz
```

### Sterowanie ręczne (teleop)

```bash
./scripts/run_demo.sh --teleop
# W osobnym terminalu:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Sterowanie operatorskie

```bash
# Wstrzymaj scenariusz
ros2 service call /robomvp/pause std_srvs/srv/SetBool '{data: true}'

# Wznów
ros2 service call /robomvp/pause std_srvs/srv/SetBool '{data: false}'

# Emergency stop
./scripts/emergency_stop.sh

# Reset do SEARCH_TABLE
ros2 service call /robomvp/reset std_srvs/srv/Trigger '{}'

# Wykonaj sekwencję przez Action (z podglądem feedbacku)
ros2 action send_goal --feedback /robomvp/manipulation_task \
  robomvp/action/ManipulationTask \
  '{sequence_name: "pick_box", apply_offset: true}'
```

---

## Automat stanowy

```
SEARCH_TABLE  ──→ (marker stołu ID=21)
DETECT_MARKER ──→ (marker pudełka ID=10)  + sekwencja: approach_table
ALIGN_WITH_BOX──→ (|dx|<0.05 i |dz|<0.05)
PICK_BOX      ──→ (notify_sequence_done)  + sekwencja: pick_box
ROTATE_180    ──→ (notify_sequence_done)  + sekwencja: rotate_180
NAVIGATE_TO_TARGET_MARKER ──→ (marker ID=30/22, z<0.3m)  + sekwencja: walk_to_second_table
PLACE_BOX     ──→ (notify_sequence_done)  + sekwencja: place_box
FINISHED
```

Stany `PICK_BOX`, `ROTATE_180`, `PLACE_BOX` **czekają** na potwierdzenie `notify_sequence_done()` przed przejściem dalej. To kluczowa poprawka v0.2.0.

---

## Budowanie i testy

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Testy jednostkowe automatu stanowego (bez ROS2)
python -m pytest ../tests/ -v
```

---

## Struktura repozytorium

```
RoboMVP/
├── README.md
├── requirements.txt
├── config/
│   ├── scene.yaml          # ID markerów, progi, timeouty
│   └── camera.yaml         # Kalibracja kamer
├── docs/
│   ├── sterowanie_robotem.md       # SDK, Sport Mode, sterowanie ramionami
│   └── poprawki_i_architektura.md  # Historia błędów i poprawek
├── rviz/
│   └── robomvp.rviz        # Konfiguracja RViz2
├── scripts/
│   ├── run_demo.sh          # Główny skrypt uruchomienia
│   ├── emergency_stop.sh    # Szybkie zatrzymanie
│   └── check_system.sh      # Weryfikacja prereqs
├── tests/
│   └── test_state_machine.py
└── ros2_ws/src/robomvp/
    ├── action/
    │   └── ManipulationTask.action   # ROS2 Action (goal/feedback/result)
    ├── msg/
    │   ├── MarkerDetection.msg
    │   ├── MarkerPose.msg
    │   ├── Offset.msg
    │   └── State.msg
    ├── launch/
    │   └── demo.launch.py
    └── robomvp/
        ├── camera_interface.py
        ├── marker_detection.py
        ├── marker_pose_estimator.py
        ├── motion_sequences.py
        ├── state_machine.py
        ├── main_node.py              # Automat + serwisy + Action Server
        ├── unitree_robot_api.py      # Adapter Unitree SDK 2
        ├── offset_corrector.py
        ├── robomvp_diagnostics.py    # DiagnosticArray (1 Hz)
        ├── robomvp_odometry.py       # nav_msgs/Odometry dead reckoning (50 Hz)
        ├── robomvp_tf_publisher.py   # Statyczne TF kamer
        └── robomvp_teleop.py         # Sterowanie /cmd_vel
```

---

## Licencja

Apache 2.0 — patrz plik `LICENSE`.
