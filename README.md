# ros2_ws1809 — гусеничный робот: движение по GPS-маршруту с объездом препятствий

ROS 2 **Jazzy**, Raspberry Pi 5, Ubuntu 24.04.

Главная функция: робот едет по заранее заданному маршруту
(`waypoints.yaml`, точки `lat/lon`), объезжает препятствия по 2D-лидару
(Nav2), а оператор в любой момент перехватывает управление пультом ELRS.

**Одометрия (принципиально):**

| Данные | Источник |
|---|---|
| линейное перемещение | `kolesa_control` → `/odom/vesc`, только `twist.linear.x` (тахометры VESC) |
| повороты (курс) | **только** IMU, из готового кватерниона `/imu/data` |
| колёсный yaw | **не используется нигде** (гусеницы в повороте буксуют) |

## Пакеты

| Пакет | Назначение |
|---|---|
| `kolesa_control` | 2×VESC (FS75100) по UART; публикует `/odom/vesc` (только линейную скорость) |
| `imu_stm32_bridge` | STM32 (MPU6050 + QMC5883L) → `/imu/data` (кватернион ENU), `/imu/mag`, калибровки |
| `elrs_receiver` | пульт ELRS → `/cmd_vel/manual` |
| `robot_odom` | эталонная одометрия: путь VESC + курс IMU → `/odom` (для калибровки и режима без Nav2) |
| `gps_navigator` | **навигационный слой**: конфиги Nav2 и robot_localization, launch-файлы, предполётная проверка, документация |
| `gps_mission` | миссия: `waypoints.yaml` → Nav2 `follow_gps_waypoints`, статус, запись точек |
| `cmd_switcher` | приоритеты команд: пульт → приложение → «домой» → автопилот |
| `relay_reliable` | `/scan` (BEST_EFFORT) → `/scan_reliable` (RELIABLE) для RViz/Wi-Fi |
| `project_start` | слой железа: launch и параметры драйверов |
| `tracked_robot_description` | URDF и TF датчиков |
| `ydlidar_ros2_driver` | драйвер лидара |

## Быстрый старт

```bash
cd ~/ros2_ws1809
sudo apt install -y ros-jazzy-robot-localization ros-jazzy-nav2-bringup \
     ros-jazzy-nav2-waypoint-follower ros-jazzy-nav2-navfn-planner \
     ros-jazzy-nav2-dwb-controller ros-jazzy-nmea-navsat-driver \
     ros-jazzy-geographic-msgs python3-serial python3-yaml
colcon build --symlink-install
source install/setup.bash

# 1) только железо (калибровки, проверки)
ros2 launch project_start start.launch.py

# 2) предполётная проверка
ros2 run gps_navigator preflight_check --ros-args \
  -p waypoints_file:=~/ros2_ws1809/src/project_start/config/waypoints.yaml

# 3) весь стек для маршрута
ros2 launch gps_navigator route.launch.py

# 4) старт миссии
ros2 service call /gps_mission/start std_srvs/srv/Trigger
```

## Документация

* **`src/gps_navigator/docs/PREFLIGHT.md` — как подготовить робота к движению
  по маршруту: что проверить, какой командой и каким результатом.**
* `src/gps_navigator/docs/OUTDOOR_NAV.md` — архитектура, одометрия, объезд,
  какие параметры править.
* `src/gps_navigator/docs/CPU_RPI5.md` — как не перегрузить CPU Pi 5, чем
  измерять, что снижать в первую очередь.
* `src/robot_odom/README.md`, `src/kolesa_control/README.md`,
  `src/imu_stm32_bridge/docs/*` — по отдельным узлам.

## Тесты

```bash
python3 -m pytest src/robot_odom/test src/gps_mission/test src/gps_navigator/test
```

52 теста: математика и поведение узла `robot_odom` (исполняется настоящий код
на заглушках rclpy, если ROS 2 не установлена), чтение/запись маршрута,
соответствие конфигов EKF/Nav2 требованиям (только `vx` от колёс, только
кватернион для курса, частоты под Pi 5, `Spin` выключен), связность
launch-файлов.
