# RoboMVP

## Cel projektu

RoboMVP to minimalna aplikacja ROS2 demonstrująca scenariusz manipulacji dla robota humanoidalnego **Unitree G1 EDU**. Projekt realizuje **MVP (Minimum Viable Product)** — prosty, deterministyczny pipeline bez uczenia maszynowego, gotowy do uruchomienia w ciągu 2 tygodni.

---

## Scenariusz demonstracyjny

Robot wykonuje następującą sekwencję kroków:

1. Robot podchodzi do stołu.
2. Robot wykrywa pudełko na stole za pomocą markera AprilTag.
3. Robot wyrównuje się z pudełkiem (korekcja offsetu).
4. Robot podnosi pudełko.
5. Robot obraca się o 180 stopni.
6. Robot idzie do drugiego stołu.
7. Robot odkłada pudełko na drugi stół.

Ruch jest realizowany za pomocą **predefiniowanych sekwencji ruchów** — bez planowania i uczenia maszynowego.

---

## Konfiguracja sprzętowa

Robot: **Unitree G1 EDU**

### Kamera ciała
- Używana do manipulacji i wykrywania markerów na pudełku i stole
- Temat ROS2: `/camera/body/image_raw`

### Kamera głowy
- Używana do nawigacji i wykrywania dalekich markerów
- Temat ROS2: `/camera/head/image_raw`

---

## Lokalizacja oparta na markerach

Obiekty w środowisku oznaczone są markerami **AprilTag** (lub kodami QR):

| Obiekt | ID markera |
|--------|-----------|
| Pudełko | 10 |
| Stół startowy | 21 |
| Stół docelowy | 22 |
| Cel nawigacji | 30 |

System wykrywa markery i szacuje ich **pozycję 3D względem kamery**:

```json
{
  "marker_id": 12,
  "position": [x, y, z],
  "orientation": [qx, qy, qz, qw]
}
```

Korekcja pozycji wykorzystuje offset:

```
dx, dy, dz = oczekiwana_pozycja - zmierzona_pozycja
```

---

## Architektura systemu

```
                +-----------------------+
                |      Head Camera      |
                +-----------+-----------+
                            |
                    /camera/head/image_raw
                            |
                  +---------------------+
                  |  marker_detection   |
                  +----------+----------+
                             |
                   /robomvp/marker_detections
                             |
                  +----------------------+
                  | marker_pose_estimator|
                  +----------+-----------+
                             |
                /robomvp/marker_pose + /robomvp/offset
                             |
+-------------+       +-------------------+       +---------------------+
| Body Camera |-----> |   robomvp_main    |-----> | motion_sequences    |
+------+------+       |   state_machine   |       | execution           |
       |              +-------------------+       +---------------------+
/camera/body/image_raw     /robomvp/state               Unitree SDK
```

### Węzły ROS2

| Węzeł | Opis |
|-------|------|
| `/camera_interface` | Publikuje obrazy z obu kamer |
| `/marker_detection` | Wykrywa markery AprilTag/QR |
| `/marker_pose_estimator` | Oblicza pozycje 3D markerów |
| `/robomvp_main` | Automat stanowy i wykonanie ruchów |

### Tematy ROS2

| Temat | Typ | Opis |
|-------|-----|------|
| `/camera/body/image_raw` | `sensor_msgs/Image` | Obraz kamery ciała |
| `/camera/head/image_raw` | `sensor_msgs/Image` | Obraz kamery głowy |
| `/robomvp/marker_detections` | `robomvp/MarkerDetection` | Wykryte markery |
| `/robomvp/marker_pose` | `robomvp/MarkerPose` | Poza 3D markera |
| `/robomvp/offset` | `robomvp/Offset` | Offset korekcji |
| `/robomvp/state` | `robomvp/State` | Stan automatu |
| `/robomvp/motion_command` | `std_msgs/String` | Komenda ruchu |

---

## Automat stanowy

```
SEARCH_TABLE
      ↓  (wykryto marker stołu startowego)
DETECT_MARKER
      ↓  (wykryto marker pudełka)
ALIGN_WITH_BOX
      ↓  (offset < próg)
PICK_BOX
      ↓  (sekwencja zakończona)
ROTATE_180
      ↓  (sekwencja zakończona)
NAVIGATE_TO_TARGET_MARKER
      ↓  (marker stołu docelowego w zasięgu)
PLACE_BOX
      ↓  (sekwencja zakończona)
FINISHED
```

---

## Instalacja

### Wymagania

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10+
- OpenCV (`pip install opencv-python`)
- Biblioteka AprilTag (`pip install apriltag`)
- `cv_bridge` (ROS2)

### Instalacja zależności

```bash
sudo apt update
sudo apt install ros-humble-cv-bridge ros-humble-launch-ros
pip install opencv-python apriltag numpy pyyaml
```

### Budowanie pakietów

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Uruchomienie demonstracji

### Tryb demo (bez sprzętu)

```bash
bash scripts/run_demo.sh
```

lub ręcznie:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch robomvp_bringup demo.launch.py mode:=demo_mode
```

### Tryb robot (z prawdziwym sprzętem)

```bash
ros2 launch robomvp_bringup demo.launch.py mode:=robot_mode
```

### Sprawdzenie grafu węzłów

```bash
ros2 node list
# /camera_interface
# /marker_detection
# /marker_pose_estimator
# /robomvp_main

ros2 topic list
# /camera/body/image_raw
# /camera/head/image_raw
# /robomvp/marker_detections
# /robomvp/marker_pose
# /robomvp/offset
# /robomvp/state
# /robomvp/motion_command
```

---

## Struktura repozytorium

```
RoboMVP/
├── README.md
├── config/
│   ├── scene.yaml          # Konfiguracja sceny i markerów
│   └── camera.yaml         # Kalibracja kamer
├── data/
│   └── test_images/        # Obrazy testowe dla trybu demo
├── scripts/
│   └── run_demo.sh         # Skrypt uruchomienia
└── ros2_ws/
    └── src/
        ├── robomvp/
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   ├── setup.py
        │   ├── msg/
        │   │   ├── MarkerDetection.msg
        │   │   ├── MarkerPose.msg
        │   │   ├── Offset.msg
        │   │   └── State.msg
        │   └── robomvp/
        │       ├── camera_interface.py
        │       ├── marker_detection.py
        │       ├── marker_pose_estimator.py
        │       ├── offset_corrector.py
        │       ├── motion_sequences.py
        │       ├── state_machine.py
        │       └── main_node.py
        └── robomvp_bringup/
            ├── CMakeLists.txt
            ├── package.xml
            └── launch/
                └── demo.launch.py
```
