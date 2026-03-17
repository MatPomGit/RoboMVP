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
      │    LocoClient.SetVelocity(0.0, 0.0, vyaw, duration)
      │    time.sleep(duration)
      │    LocoClient.StopMove()
      │
      └─── Faza 2: Translacja
           vx = (dy / dist) * LINEAR_SPEED  # ruch do przodu [m/s]
           vy = (dx / dist) * LINEAR_SPEED  # ruch boczny [m/s]
           LocoClient.SetVelocity(vx, vy, 0.0, duration)
           time.sleep(duration)
           LocoClient.StopMove()
```

Sygnatura `LocoClient.SetVelocity` wygląda następująco:

```python
LocoClient.SetVelocity(vx: float, vy: float, omega: float, duration: float = 1.0)
# vx       – prędkość do przodu / tyłu [m/s]   (+ = przód)
# vy       – prędkość boczna [m/s]              (+ = lewo)
# omega    – prędkość kątowa [rad/s]            (+ = obrót w lewo)
# duration – czas trwania komendy [s]           (obsługiwany po stronie serwera)
```

SDK obsługuje parametr `duration` po stronie serwera – robot wykonuje ruch przez zadany czas. Dodatkowo `StopMove()` zeruje wszystkie prędkości (środek bezpieczeństwa).

Metoda `Move(vx, vy, vyaw, continous_move=False)` jest wrapperem na `SetVelocity` z `duration=1.0` (lub `864000.0` dla trybu ciągłego).

### 1.3 Punkt styku: `unitree_robot_api.py`

Plik `ros2_ws/src/robomvp/robomvp/unitree_robot_api.py` jest **jedynym miejscem**, w którym logika ROS2 (automat stanowy, sekwencje ruchów) styka się z Unitree SDK 2. Główny węzeł `robomvp_main` wywołuje `robot_api.move_to_pose(pose)`, a ta metoda korzysta z `LocoClient.SetVelocity()` (lokomocja) i `G1ArmActionClient.ExecuteAction()` (ramiona) – bez ROS2.

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
UnitreeRobotAPI ───────┼──→ LocoClient.SetVelocity()
   move_to_pose()      │    G1ArmActionClient.ExecuteAction()
   execute_arm_action() │          ↓
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

**Sport Mode jest jedynym trybem lokomocji używanym w RoboMVP** – plik `unitree_robot_api.py` opiera się na `LocoClient` (interfejs do Sport Mode) oraz `G1ArmActionClient` (predefiniowane akcje ramion).

**Dostępne komendy LocoClient (Sport Mode) – rzeczywiste funkcje z SDK:**

```python
# Ruch z parametrem czasu trwania (SetVelocity)
loco_client.SetVelocity(vx, vy, omega, duration=1.0)  # komenda prędkości z czasem trwania [s]
loco_client.Move(vx, vy, vyaw, continous_move=False)   # wrapper: duration=1s lub 864000s
loco_client.StopMove()                                  # zerowanie prędkości (SetVelocity(0,0,0))

# Zmiana trybu FSM (Finite State Machine)
loco_client.Start()              # wejście w tryb chodzenia (FSM ID = 200)
loco_client.Damp()               # tryb tłumiony – bezpieczne zatrzymanie (FSM ID = 1)
loco_client.Sit()                # siadanie (FSM ID = 3)
loco_client.ZeroTorque()         # rozluźnienie stawów (FSM ID = 0, ostrożnie!)
loco_client.Squat2StandUp()      # wstanie z przysiadu (FSM ID = 706)
loco_client.Lie2StandUp()        # wstanie z pozycji leżącej (FSM ID = 702)

# Wysokość postawy
loco_client.HighStand()          # wysoka postawa
loco_client.LowStand()           # niska postawa
loco_client.SetStandHeight(h)    # ustawienie konkretnej wysokości

# Gesty / ruchy specjalne (przez SetTaskId)
loco_client.WaveHand()           # machanie ręką
loco_client.WaveHand(True)       # machanie ręką z obrotem
loco_client.ShakeHand()          # podanie ręki

# Ustawienia lokomocji
loco_client.SetSwingHeight(h)    # wysokość unoszenia stóp
loco_client.SetBalanceMode(m)    # tryb balansu
loco_client.SetFsmId(id)         # bezpośrednie ustawienie trybu FSM
```

### 3.3 Jak włączyć Sport Mode

Sport Mode jest **domyślnym trybem** po uruchomieniu SDK. Inicjalizuje się go przez:

```python
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

# 1. Inicjalizacja transportu DDS (raz na proces)
ChannelFactoryInitialize(0, 'eth0')   # '0' = pierwsza karta, 'eth0' = interfejs

# 2. Utworzenie klienta lokomocji
loco_client = LocoClient()
loco_client.SetTimeout(10.0)   # timeout odpowiedzi od robota [s]
loco_client.Init()              # rejestracja API w kliencie RPC

# 3. Przełączenie w tryb chodzenia (FSM ID = 200)
loco_client.Start()

# 4. Robot jest teraz w Sport Mode – można wydawać komendy
loco_client.SetVelocity(0.3, 0.0, 0.0, 3.0)  # idź do przodu 0.3 m/s przez 3s
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

Sterowanie nogami realizuje się przez `SetVelocity()` z parametrem `duration`:

```python
import time
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

def walk_forward(loco_client: LocoClient, speed: float, duration: float):
    """Idzie do przodu z zadaną prędkością przez zadany czas."""
    loco_client.SetVelocity(speed, 0.0, 0.0, duration)
    time.sleep(duration)
    loco_client.StopMove()
```

SDK obsługuje `duration` po stronie serwera – robot wykonuje ruch przez zadany czas bez konieczności wysyłania poleceń w pętli. Wywołanie `StopMove()` po zakończeniu ruchu jest dodatkowym środkiem bezpieczeństwa.

### 4.3 Górna część ciała: ramiona

Unitree SDK 2 udostępnia dwa interfejsy do sterowania ramionami:

#### 4.3.1 Predefiniowane akcje – G1ArmActionClient (wysoki poziom)

`G1ArmActionClient` umożliwia wykonywanie gotowych gestów ramion:

```python
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map

arm_client = G1ArmActionClient()
arm_client.SetTimeout(10.0)
arm_client.Init()

# Dostępne akcje (action_map):
#   'release arm': 99   – zwolnienie ramion
#   'shake hand': 27    – podanie ręki
#   'high five': 18     – piątka
#   'hug': 19           – objęcie
#   'wave hand': ...    – machanie (przez LocoClient.WaveHand())
#   'clap': 17          – klaskanie
#   'heart': 20         – serce
#   'hands up': 15      – ręce do góry
#   i inne (patrz action_map w g1_arm_action_client.py)

arm_client.ExecuteAction(action_map['shake hand'])
time.sleep(2)
arm_client.ExecuteAction(action_map['release arm'])
```

#### 4.3.2 Bezpośrednie sterowanie stawami – ArmSDK DDS (niski poziom)

Do precyzyjnego sterowania pozycjami stawów służy niskopoziomowe API przez kanały DDS:

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
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient, action_map


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
    loco.Start()           # wejście w tryb chodzenia (FSM ID = 200)
    time.sleep(1.0)        # czas na ustabilizowanie postawy

    # 3. Klient akcji ramion
    arm = G1ArmActionClient()
    arm.SetTimeout(10.0)
    arm.Init()

    # 4. Ramiona w pozycji trzymania (predefiniowana akcja)
    arm.ExecuteAction(action_map['hug'])

    # 5. Ruch do przodu z zadanym czasem trwania (SetVelocity z duration)
    loco.SetVelocity(walk_speed, 0.0, 0.0, walk_duration)
    time.sleep(walk_duration)
    loco.StopMove()

    # 6. Zwolnienie ramion
    arm.ExecuteAction(action_map['release arm'])

    print('Robot dotarł do celu i zatrzymał się.')
```

### 4.5 Przykład: robot idzie i macha ręką

Gdy robot ma machać ręką podczas chodzenia, można użyć wbudowanej funkcji `WaveHand()`:

```python
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

ChannelFactoryInitialize(0, 'eth0')
loco = LocoClient()
loco.SetTimeout(10.0)
loco.Init()
loco.Start()

# Machanie ręką (wbudowana akcja LocoClient)
loco.WaveHand()           # machanie bez obrotu
# loco.WaveHand(True)     # machanie z obrotem

# Jednocześnie ruch do przodu
loco.SetVelocity(0.3, 0.0, 0.0, 5.0)
time.sleep(5.0)
loco.StopMove()
```

Dla bardziej zaawansowanych animacji ramion (np. sinusoidalne machanie) należy użyć niskopoziomowego API ramion (arm_sdk DDS) z kanałem `rt/arm_sdk`.

### 4.6 Integracja z RoboMVP

Obecna implementacja RoboMVP używa `LocoClient.SetVelocity()` do lokomocji oraz `G1ArmActionClient.ExecuteAction()` do sterowania ramionami (predefiniowane gesty). Pole `z` w słownikach pozycji (`motion_sequences.py`) jest przeznaczone do sterowania ramionami – w przyszłości może być zrealizowane przez niskopoziomowe API ramion (arm_sdk DDS) dla precyzyjnej kontroli pozycji stawów.

Dostępne metody `UnitreeRobotAPI`:

```python
# Lokomocja (LocoClient)
robot_api.move_to_pose({'x': 0.0, 'y': 0.5, 'z': 0.0, 'yaw': 0.0})

# Predefiniowane akcje ramion (G1ArmActionClient)
robot_api.execute_arm_action('shake hand')
robot_api.execute_arm_action('release arm')
```

Aby dodać niskopoziomowe sterowanie ramionami do RoboMVP (np. dla precyzyjnego chwytu), należy:

1. **Rozszerzyć `UnitreeRobotAPI`** o komunikację DDS z kanałem `rt/arm_sdk`:

```python
# W pliku unitree_robot_api.py
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

class UnitreeRobotAPI:
    def connect(self, logger=None) -> None:
        # ... istniejący kod LocoClient + G1ArmActionClient ...

        # Inicjalizacja niskopoziomowego sterowania ramionami (opcjonalnie)
        self._arm_publisher = ChannelPublisher('rt/arm_sdk', LowCmd_)
        self._arm_publisher.Init()
        self._arm_subscriber = ChannelSubscriber('rt/lowstate', LowState_)
        self._arm_subscriber.Init(self._on_low_state, 10)
```

2. **W `motion_sequences.py`** przekazać kąty stawów jako opcjonalne pole `arm_pose` w słowniku waypoint:

```python
{'x': 0.0, 'y': 1.5, 'z': 0.0, 'yaw': 0.0,
 'arm_pose': {15: 0.0, 16: 0.5, ..., 26: 0.0}}
```

3. **W `execute_sequence()`** wywołać odpowiednie akcje ramion przy zmianie pozy.

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
│  │   LocoClient     │         │  G1ArmActionClient    │             │
│  │  (Sport Mode)    │         │  (predefiniowane      │             │
│  │  SetVelocity()   │         │   akcje ramion)       │             │
│  │  Start() / Damp()│         │  ExecuteAction()      │             │
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
| Jak SDK wysyła komendy ruchu? | Przez `LocoClient.SetVelocity(vx, vy, vyaw, duration)` po DDS/Ethernet |
| Jak SDK steruje ramionami? | Przez `G1ArmActionClient.ExecuteAction(action_id)` (predefiniowane gesty) |
| Jak ROS2 wysyła komendy? | ROS2 NIE wysyła komendy do sprzętu – koordynuje logikę i percepcję |
| Czy SDK i ROS2 się gryzą? | Nie, o ile tylko jeden klient wysyła komendy przez `LocoClient` |
| Czym jest Sport Mode? | Trybem wysokopoziomowym – programista podaje prędkości, robot sam chodzi |
| Jak sterować osobno górą i dołem? | `LocoClient.SetVelocity()` (nogi) + `G1ArmActionClient` (ramiona) |
