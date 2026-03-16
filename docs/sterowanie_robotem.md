# Sterowanie robotem Unitree G1 EDU: SDK, ROS2 i niezależne sterowanie kończynami

## Spis treści

1. [Jak przekazywane są komendy do robota](#1-jak-przekazywane-są-komendy-do-robota)
   - 1.1 [Ścieżka komendy przez ROS2](#11-ścieżka-komendy-przez-ros2)
   - 1.2 [Ścieżka komendy przez Unitree SDK 2](#12-ścieżka-komendy-przez-unitree-sdk-2)
   - 1.3 [Punkt styku: `unitree_robot_api.py`](#13-punkt-styku-unitree_robot_apipy)
2. [Czy SDK i ROS2 się gryzą?](#2-czy-sdk-i-ros2-się-gryzą)
3. [Czym jest Sport Mode w robocie?](#3-czym-jest-sport-mode-w-robocie)
   - 3.1 [Tryby sterowania w Unitree G1](#31-tryby-sterowania-w-unitree-g1)
   - 3.2 [Sport Mode – LocoClient](#32-sport-mode--lococlient)
   - 3.3 [Jak włączyć Sport Mode](#33-jak-włączyć-sport-mode)
4. [Niezależne sterowanie górną i dolną częścią ciała](#4-niezależne-sterowanie-górną-i-dolną-częścią-ciała)
   - 4.1 [Zasada współbieżnego sterowania](#41-zasada-współbieżnego-sterowania)
   - 4.2 [Dolna część ciała: LocoClient (nogi)](#42-dolna-część-ciała-lococlient-nogi)
   - 4.3 [Górna część ciała: ArmSDK (ramiona)](#43-górna-część-ciała-armsdk-ramiona)
   - 4.4 [Przykład: robot idzie do celu i trzyma pudełko](#44-przykład-robot-idzie-do-celu-i-trzyma-pudełko)
   - 4.5 [Przykład: robot idzie i macha ręką](#45-przykład-robot-idzie-i-macha-ręką)
   - 4.6 [Integracja z RoboMVP](#46-integracja-z-robomvp)
5. [Podsumowanie architektury sterowania](#5-podsumowanie-architektury-sterowania)

---

## 1. Jak przekazywane są komendy do robota

W systemie RoboMVP komendy do robota Unitree G1 EDU docierają dwiema równoległymi ścieżkami, które spotykają się w jednym punkcie integracji.

### 1.1 Ścieżka komendy przez ROS2

ROS2 odpowiada za całą warstwę **percepcji i koordynacji logicznej** – od obrazu kamery aż do decyzji o ruchu. Dane przepływają kolejno przez węzły:

```
Kamera (sprzęt)
      │
      ▼
/camera/body/image_raw        /camera/head/image_raw
      │                                │
      └──────────┬────────────────────┘
                 ▼
      [węzeł: marker_detection]
         Wykrywa markery AprilTag/QR
                 │
                 ▼
      /robomvp/marker_detections
                 │
                 ▼
      [węzeł: marker_pose_estimator]
         Oblicza pozycję 3D markera
                 │
      ┌──────────┴────────────┐
      ▼                       ▼
/robomvp/marker_pose    /robomvp/offset
      │                       │
      └──────────┬────────────┘
                 ▼
      [węzeł: robomvp_main]
         Automat stanowy
         Decyduje o ruchu
                 │
                 ▼
      /robomvp/motion_command   /robomvp/state
                 │
                 ▼
      [wywołanie: execute_sequence()]
         Tłumaczy komendę na
         parametry ruchu (x, y, yaw)
                 │
                 ▼
      UnitreeRobotAPI.move_to_pose()
```

W tej ścieżce **ROS2 nie steruje robotem bezpośrednio** – jego rolą jest:
- przetwarzanie obrazów z kamer,
- wykrywanie i lokalizowanie markerów,
- zarządzanie stanem automatu (co robot ma teraz robić),
- wywołanie odpowiedniej sekwencji ruchów.

### 1.2 Ścieżka komendy przez Unitree SDK 2

Unitree SDK 2 (`unitree_sdk2py`) to warstwa, która bezpośrednio komunikuje się z robotem. Używa protokołu **DDS (Data Distribution Service)** – tego samego, na którym opiera się ROS2 (szczegóły w sekcji 2).

```
UnitreeRobotAPI.move_to_pose(pose)
      │
      ├─── Faza 1: Obrót
      │    LocoClient.Move(0.0, 0.0, vyaw)
      │    time.sleep(allocated_rot)
      │    LocoClient.StopMove()
      │
      └─── Faza 2: Translacja
           vx = (dy / dist) * LINEAR_SPEED  # ruch do przodu [m/s]
           vy = (dx / dist) * LINEAR_SPEED  # ruch boczny [m/s]
           LocoClient.Move(vx, vy, 0.0)
           time.sleep(move_time)
           LocoClient.StopMove()
```

Sygnatura `LocoClient.Move` wygląda następująco:

```python
LocoClient.Move(vx: float, vy: float, vyaw: float)
# vx   – prędkość do przodu / tyłu [m/s]   (+ = przód)
# vy   – prędkość boczna [m/s]              (+ = lewo)
# vyaw – prędkość kątowa [rad/s]            (+ = obrót w lewo)
```

Komenda jest wysyłana **w sposób ciągły** (co kilka ms robot ją odświeża). Wywołanie `StopMove()` zeruje wszystkie prędkości.

### 1.3 Punkt styku: `unitree_robot_api.py`

Plik `ros2_ws/src/robomvp/robomvp/unitree_robot_api.py` jest **jedynym miejscem**, w którym logika ROS2 (automat stanowy, sekwencje ruchów) styka się z Unitree SDK 2. Główny węzeł `robomvp_main` wywołuje `robot_api.move_to_pose(pose)`, a ta metoda korzysta już wyłącznie z `LocoClient` – bez ROS2.

```
Warstwa ROS2           │   Warstwa SDK
(logika i percepcja)   │   (sprzęt)
                       │
state_machine          │
   ↓                   │
motion_sequences       │
   ↓                   │
main_node              │
   ↓                   │
UnitreeRobotAPI ───────┼──→ LocoClient.Move()
   move_to_pose()      │          ↓
                       │    Robot (DDS/Ethernet)
```

---

## 2. Czy SDK i ROS2 się gryzą?

**Krótka odpowiedź: Nie – jeśli korzysta się z nich w odpowiednich warstwach.**

Oba systemy korzystają z protokołu DDS pod spodem, jednak adresują zupełnie inne kanały komunikacji:

| | ROS2 | Unitree SDK 2 |
|---|---|---|
| **Protokół** | DDS (CycloneDDS / FastDDS) | DDS (własna implementacja) |
| **Kanały** | Tematy ROS2 (`/robomvp/*`, `/camera/*`) | Kanały SDK (`rt/loco/request`, itp.) |
| **Odbiorca** | Węzły ROS2 w systemie | Firmware robota |
| **Rola w RoboMVP** | Percepcja, koordynacja, logika | Sterowanie sprzętem |

Konflikty mogą wystąpić **tylko wtedy**, gdy dwie aplikacje jednocześnie wysyłają rozbieżne komendy przez `LocoClient` do tej samej instancji robota. W RoboMVP tak się nie dzieje, bo:

1. Tylko jeden węzeł (`robomvp_main`) wywołuje `UnitreeRobotAPI`.
2. Wywołania `move_to_pose()` są synchroniczne (blokują wątek do zakończenia ruchu).
3. ROS2 nie wysyła żadnych bezpośrednich komend ruchu – tylko aktualizuje stan automatu.

**Potencjalny konflikt**: gdyby równolegle uruchomić zewnętrzną aplikację (np. panel operatora lub teleop) korzystającą z tego samego `LocoClient`, komendy nakładałyby się. Rozwiązaniem jest wzajemne wykluczanie przez blokadę (mutex) lub użycie dedykowanego węzła arbitrażu.

---

## 3. Czym jest Sport Mode w robocie?

### 3.1 Tryby sterowania w Unitree G1

Robot Unitree G1 EDU obsługuje kilka trybów sterowania, różniących się poziomem abstrakcji:

| Tryb | API | Poziom abstrakcji | Opis |
|---|---|---|---|
| **Sport Mode** (tryb sportowy) | `LocoClient` | Wysoki | Sterowanie prędkością – robot sam oblicza ruchy nóg |
| **Low-Level Mode** (tryb niskopoziomowy) | `LowCmd` / `LowState` | Niski | Sterowanie momentem/kątem każdego stawu osobno |
| **Arm Mode** (tryb ramion) | `ArmSDK` / `H1ArmSDK` | Średni | Sterowanie stawami ramion (niezależne od nóg) |

### 3.2 Sport Mode – LocoClient

**Sport Mode** (zwany też *locomotion mode* lub *high-level mode*) to tryb, w którym robot:

- **Sam zarządza gaitem** (sekwencją kroków nóg) – programista tylko podaje prędkości,
- **Automatycznie utrzymuje równowagę** podczas chodzenia,
- **Reaguje na przeszkody** (jeśli oprogramowanie robota ma włączone odpowiednie moduły).

W tym trybie programista nie musi wiedzieć, jak zsynchronizować ruchy 12 stawów nóg. Wystarczy jedno wywołanie:

```python
loco_client.Move(vx=0.3, vy=0.0, vyaw=0.0)  # idź do przodu 0.3 m/s
```

Robot samodzielnie przeliczy to na trajektorie wszystkich kończyn dolnych.

**Sport Mode jest jedynym trybem używanym w RoboMVP** – cały plik `unitree_robot_api.py` opiera się na `LocoClient`, który jest interfejsem do Sport Mode.

**Dostępne komendy LocoClient (Sport Mode):**

```python
# Ruch
loco_client.Move(vx, vy, vyaw)  # prędkości liniowe i kątowa
loco_client.StopMove()           # natychmiastowe zatrzymanie

# Pozy statyczne
loco_client.StandUp()            # wstanie z pozycji siedzącej
loco_client.StandDown()          # usiadanie
loco_client.ZeroTorque()         # rozluźnienie stawów (ostrożnie!)
loco_client.Damp()               # tryb tłumiony – soft stop

# Gesty / ruchy specjalne
loco_client.WaveHand()           # machanie ręką (wbudowane)
loco_client.Dance1()             # wbudowany taniec (jeśli dostępny)
```

### 3.3 Jak włączyć Sport Mode

Sport Mode jest **domyślnym trybem** po uruchomieniu SDK. Inicjalizuje się go przez:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

# 1. Inicjalizacja transportu DDS (raz na proces)
ChannelFactoryInitialize(0, 'eth0')   # '0' = pierwsza karta, 'eth0' = interfejs

# 2. Utworzenie klienta lokomocji (= wejście w Sport Mode)
loco_client = LocoClient()
loco_client.SetTimeout(10.0)   # timeout odpowiedzi od robota [s]
loco_client.Init()              # nawiązanie połączenia

# 3. Robot jest teraz w Sport Mode – można wydawać komendy
loco_client.StandUp()
loco_client.Move(0.3, 0.0, 0.0)
```

**Ważne**: `ChannelFactoryInitialize` należy wywołać **dokładnie raz** na cały proces (nie per-wątek, nie per-węzeł). Wielokrotne wywołanie spowoduje błąd inicjalizacji DDS.

---

## 4. Niezależne sterowanie górną i dolną częścią ciała

Jedną z kluczowych możliwości robota humanoidalnego jest **jednoczesne, niezależne** sterowanie nogami (lokomocja) i ramionami (manipulacja). Przykłady zastosowań:
- robot idzie do celu, trzymając pudełko stabilnie na poziomie bioder,
- robot idzie i macha ręką (powitanie, sygnalizacja),
- robot obraca się i jednocześnie wyciąga rękę po przedmiot.

### 4.1 Zasada współbieżnego sterowania

Unitree SDK 2 udostępnia **dwa niezależne interfejsy**:

```
┌─────────────────────────────────────────────────────┐
│                  Robot Unitree G1 EDU               │
│                                                     │
│  ┌───────────────────┐   ┌──────────────────────┐   │
│  │   Dolna część     │   │    Górna część        │   │
│  │   (nogi + tułów)  │   │    (ramiona + dłonie) │   │
│  │                   │   │                      │   │
│  │   LocoClient      │   │   ArmSDK / ArmWriter  │   │
│  │   Move(vx,vy,yaw) │   │   set_joint_pos(...)  │   │
│  └───────────────────┘   └──────────────────────┘   │
│            ▲                          ▲             │
└────────────┼──────────────────────────┼─────────────┘
             │                          │
     Wątek lokomocji            Wątek sterowania
     (Python Thread 1)          ramionami (Thread 2)
```

Kluczowe zasady:
1. **Każdy interfejs działa niezależnie** – nie trzeba synchronizować `LocoClient` z `ArmSDK`.
2. **Oba wątki mogą działać jednocześnie** – SDK obsługuje to po stronie DDS.
3. **`ChannelFactoryInitialize` wywołuje się tylko raz** – współdzielony transport DDS.

### 4.2 Dolna część ciała: LocoClient (nogi)

Sterowanie nogami realizuje się w pętli wysyłającej komendy prędkości:

```python
import threading
import time
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

def locomotion_loop(loco_client: LocoClient, target_vx: float, stop_event: threading.Event):
    """Pętla sterowania lokomocją – wysyła komendy prędkości co 50 ms."""
    while not stop_event.is_set():
        loco_client.Move(target_vx, 0.0, 0.0)
        time.sleep(0.05)   # 20 Hz
    loco_client.StopMove()
```

Pętla musi być **aktywna** (wysyłać komendy cyklicznie), bo robot po braku świeżej komendy automatycznie zwalnia. Częstotliwość 20 Hz jest wystarczająca dla Sport Mode.

### 4.3 Górna część ciała: ArmSDK (ramiona)

Unitree SDK 2 udostępnia interfejs do sterowania stawami ramion przez `ArmWriter` lub dedykowany klient:

```python
import threading
import time
from unitree_sdk2py.g1.arm.g1_arm_sdk import ArmWriter   # import zależny od wersji SDK

# ⚠️  Numery stawów poniżej są PRZYKŁADOWE.
# Rzeczywiste indeksy różnią się w zależności od wersji robota i SDK.
# Sprawdź plik example/g1_arm_sdk.py w repozytorium unitree_sdk2_python
# lub dokumentację dla swojej wersji sprzętu.
#
# Schemat typowy dla G1 (może się różnić):
#   Lewe ramię:  stawy 15..20 (bark_yaw, bark_pitch, łokieć, nadgarstek x3)
#   Prawe ramię: stawy 21..26 (bark_yaw, bark_pitch, łokieć, nadgarstek x3)
LEFT_ARM_JOINTS  = list(range(15, 21))  # ZASTĄP rzeczywistymi indeksami z SDK
RIGHT_ARM_JOINTS = list(range(21, 27))  # ZASTĄP rzeczywistymi indeksami z SDK

def arm_hold_box(arm_writer: ArmWriter, stop_event: threading.Event):
    """
    Utrzymuje ramiona w pozycji 'trzymania pudełka' podczas chodzenia.
    Wysyła co 20 ms żądaną pozycję stawów (kąty w radianach).
    Kąty są PRZYKŁADOWE – dostosuj do fizycznej pozycji swojego robota.
    """
    # ⚠️  Docelowe kąty stawów – PRZYKŁADOWE wartości, wymagają kalibracji na sprzęcie
    hold_position = {
        15: 0.0,   # lewy bark – obrót      (PRZYKŁAD – zastąp wartością z SDK)
        16: 0.5,   # lewy bark – przód/tył  (PRZYKŁAD)
        17: -0.3,  # lewy łokieć            (PRZYKŁAD)
        18: 0.0,   # lewy nadgarstek yaw    (PRZYKŁAD)
        19: 0.2,   # lewy nadgarstek pitch  (PRZYKŁAD)
        20: 0.0,   # lewy nadgarstek roll   (PRZYKŁAD)
        21: 0.0,   # prawy bark – obrót     (PRZYKŁAD)
        22: 0.5,   # prawy bark – przód/tył (PRZYKŁAD)
        23: 0.3,   # prawy łokieć           (PRZYKŁAD)
        24: 0.0,   # prawy nadgarstek yaw   (PRZYKŁAD)
        25: 0.2,   # prawy nadgarstek pitch (PRZYKŁAD)
        26: 0.0,   # prawy nadgarstek roll  (PRZYKŁAD)
    }
    while not stop_event.is_set():
        arm_writer.set_joint_positions(hold_position)
        time.sleep(0.02)   # 50 Hz
```

> **Uwaga**: Dokładne indeksy stawów, nazwy klas i ścieżki importów zależą od wersji `unitree_sdk2py`
> (sprawdź zainstalowaną wersję poleceniem `pip show unitree_sdk2py`).
> Zawsze sprawdzaj dokumentację odpowiadającą **Twojej wersji SDK**, dostępną pod adresem:
> [https://github.com/unitreerobotics/unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)
>
> W szczególności:
> - Ścieżka importu (`unitree_sdk2py.g1.arm.*`) może się różnić między wersjami.
> - Numery stawów podane w przykładach poniżej są **przykładowymi wartościami** – muszą zostać
>   zastąpione wartościami z dokumentacji SDK dla posiadanej wersji robota.
> - Pliki przykładowe dołączone do SDK (katalog `example/`) zawierają zweryfikowane wartości.

### 4.4 Przykład: robot idzie do celu i trzyma pudełko

Poniższy przykład pokazuje kompletny schemat współbieżnego sterowania:

```python
import threading
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.g1.arm.g1_arm_sdk import ArmWriter  # import zależny od wersji SDK – sprawdź dokumentację


def walk_while_holding_box(
    network_interface: str = 'eth0',
    walk_speed: float = 0.3,
    walk_duration: float = 5.0,
):
    """
    Robot idzie do przodu przez `walk_duration` sekund,
    jednocześnie trzymając pudełko (ramiona w ustalonej pozycji).
    """
    # 1. Inicjalizacja DDS – TYLKO RAZ na cały proces
    ChannelFactoryInitialize(0, network_interface)

    # 2. Klient lokomocji (Sport Mode)
    loco = LocoClient()
    loco.SetTimeout(10.0)
    loco.Init()
    loco.StandUp()
    time.sleep(1.0)   # czas na ustabilizowanie postawy

    # 3. Interfejs ramion
    arm = ArmWriter()
    arm.Init()

    stop_event = threading.Event()

    # 4. Wątek lokomocji (nogi idą do przodu)
    def loco_thread():
        while not stop_event.is_set():
            loco.Move(walk_speed, 0.0, 0.0)
            time.sleep(0.05)
        loco.StopMove()

    # 5. Wątek ramion (trzymają pudełko)
    hold_angles = {
        15: 0.0, 16: 0.5, 17: -0.3, 18: 0.0, 19: 0.2, 20: 0.0,  # lewe ramię
        21: 0.0, 22: 0.5, 23:  0.3, 24: 0.0, 25: 0.2, 26: 0.0,  # prawe ramię
    }

    def arm_thread():
        while not stop_event.is_set():
            arm.set_joint_positions(hold_angles)
            time.sleep(0.02)

    # 6. Uruchomienie wątków
    t_loco = threading.Thread(target=loco_thread, daemon=True)
    t_arm  = threading.Thread(target=arm_thread,  daemon=True)
    t_loco.start()
    t_arm.start()

    # 7. Czekamy przez zadany czas
    time.sleep(walk_duration)

    # 8. Zatrzymanie obu wątków
    stop_event.set()
    t_loco.join(timeout=2.0)
    t_arm.join(timeout=2.0)

    print('Robot dotarł do celu i zatrzymał się.')
```

### 4.5 Przykład: robot idzie i macha ręką

Gdy zamiast trzymać pudełko robot ma machać ręką, wystarczy zmienić wątek ramion na animację:

```python
import math

def arm_wave_thread(arm_writer, stop_event: threading.Event, freq_hz: float = 0.5):
    """Macha prawą ręką w górę i w dół z zadaną częstotliwością."""
    t = 0.0
    dt = 0.02   # 50 Hz

    # Kąty bazowe (prawa ręka opuszczona wzdłuż ciała)
    base_angles = {
        21: 0.0,   # prawy bark – obrót
        22: 0.0,   # prawy bark – przód/tył
        23: 0.0,   # prawy łokieć
        24: 0.0,   # prawy nadgarstek yaw
        25: 0.0,   # prawy nadgarstek pitch
        26: 0.0,   # prawy nadgarstek roll
    }

    while not stop_event.is_set():
        wave_angles = dict(base_angles)
        # Sinusoidalne machanie barkiem (staw 22: przód/tył)
        wave_angles[22] = 0.6 * math.sin(2 * math.pi * freq_hz * t)
        arm_writer.set_joint_positions(wave_angles)
        t += dt
        time.sleep(dt)
```

### 4.6 Integracja z RoboMVP

Obecna implementacja RoboMVP używa wyłącznie `LocoClient` (Sport Mode) i nie steruje ramionami przez SDK – pole `z` w słownikach pozycji (`motion_sequences.py`) jest zarezerwowane na przyszłą integrację z `ArmSDK`.

Aby dodać niezależne sterowanie ramionami do RoboMVP, należy:

1. **Rozszerzyć `UnitreeRobotAPI`** o wątek ramion:

```python
# W pliku unitree_robot_api.py

import threading
from unitree_sdk2py.g1.arm.g1_arm_sdk import ArmWriter

class UnitreeRobotAPI:
    def connect(self, logger=None) -> None:
        # ... istniejący kod LocoClient ...

        # Inicjalizacja interfejsu ramion
        self._arm_writer = ArmWriter()
        self._arm_writer.Init()
        self._arm_stop_event = threading.Event()
        self._arm_thread = None

    def start_arm_hold(self, joint_angles: dict) -> None:
        """Uruchamia wątek trzymający ramiona w zadanej pozycji."""
        self._arm_stop_event.clear()

        def _loop():
            while not self._arm_stop_event.is_set():
                self._arm_writer.set_joint_positions(joint_angles)
                time.sleep(0.02)

        self._arm_thread = threading.Thread(target=_loop, daemon=True)
        self._arm_thread.start()

    def stop_arm_hold(self) -> None:
        """Zatrzymuje wątek sterowania ramionami."""
        self._arm_stop_event.set()
        if self._arm_thread is not None:
            self._arm_thread.join(timeout=1.0)
```

2. **W `motion_sequences.py`** przekazać kąty stawów jako opcjonalne pole `arm_pose` w słowniku waypoint:

```python
{'x': 0.0, 'y': 1.5, 'z': 0.0, 'yaw': 0.0,
 'arm_pose': {15: 0.0, 16: 0.5, ..., 26: 0.0}}
```

3. **W `execute_sequence()`** wywołać `robot_api.start_arm_hold(pose['arm_pose'])` przed ruchem i `stop_arm_hold()` po dotarciu do celu lub przy zmianie pozy ramion.

---

## 5. Podsumowanie architektury sterowania

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Aplikacja RoboMVP                            │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                     Warstwa ROS2                             │   │
│  │  camera_interface → marker_detection → marker_pose_estimator │   │
│  │                              ↓                               │   │
│  │                        robomvp_main                          │   │
│  │               (automat stanowy + sekwencje)                  │   │
│  └──────────────────────────┬───────────────────────────────────┘   │
│                             │                                       │
│                             ▼                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                  UnitreeRobotAPI                             │   │
│  │          (punkt integracji ROS2 ↔ SDK)                       │   │
│  └──────────┬──────────────────────────────┬────────────────────┘   │
│             │                              │                        │
│             ▼                              ▼                        │
│  ┌──────────────────┐         ┌───────────────────────┐             │
│  │   LocoClient     │         │   ArmSDK / ArmWriter  │             │
│  │  (Sport Mode)    │         │   (wątek ramion)      │             │
│  │  Wątek lokomocji │         │   (opcjonalne –       │             │
│  │                  │         │    rozszerzenie MVP)   │             │
│  └─────────┬────────┘         └──────────┬────────────┘             │
└────────────┼─────────────────────────────┼─────────────────────────┘
             │                             │
             └─────────────┬───────────────┘
                           ▼
              Robot Unitree G1 EDU (DDS / Ethernet)
              ┌──────────────────────────────┐
              │  Dolna część:  nogi (12 DOF) │
              │  Górna część: ramiona (12 DOF)│
              └──────────────────────────────┘
```

**Kluczowe wnioski:**

| Pytanie | Odpowiedź |
|---|---|
| Jak SDK wysyła komendy? | Przez `LocoClient.Move(vx, vy, vyaw)` po DDS/Ethernet |
| Jak ROS2 wysyła komendy? | ROS2 NIE wysyła komendy do sprzętu – koordynuje logikę i percepcję |
| Czy SDK i ROS2 się gryzą? | Nie, o ile tylko jeden klient wysyła komendy przez `LocoClient` |
| Czym jest Sport Mode? | Trybem wysokopoziomowym – programista podaje prędkości, robot sam chodzi |
| Jak sterować osobno górą i dołem? | Wątek 1: `LocoClient.Move()` (nogi); Wątek 2: `ArmSDK` (ramiona) |
