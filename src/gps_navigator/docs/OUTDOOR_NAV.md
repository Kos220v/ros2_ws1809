# Движение по GPS-маршруту с объездом препятствий (Nav2)

## Архитектура

```
                         ┌──────────────── kolesa_control ────────────────┐
                         │ 2×VESC: /odom/vesc = ТОЛЬКО twist.linear.x     │
                         └───────────────────────┬───────────────────────┘
                                                 │ vx
imu_stm32_bridge ──/imu/data (кватернион ENU)───►├──► ekf_odom ──► TF odom→base_link
                                                 │              └► /odometry/filtered
                                                 └──► ekf_map ──► TF map→odom
nmea_navsat_driver ─/gps/fix─► navsat_transform ─/odometry/gps─┘  │
                                    └─ сервисы /datum /fromLL /toLL
ydlidar ─/scan─► Nav2: global+local costmap → NavFn → DWB → BT ─/cmd_vel/auto
waypoints.yaml ─► gps_mission ─► action follow_gps_waypoints (nav2_waypoint_follower)
/cmd_vel/auto ─► cmd_switcher ─► /cmd_vel ─► kolesa_control
elrs_receiver ─/control_mode (тумблер)─► gps_mission: AUTO=старт/продолжение,
                                        прочие положения=отмена миссии
```

### Одометрия (требование ТЗ)

| Источник | Что берём | Что НЕ берём |
|---|---|---|
| `kolesa_control` → `/odom/vesc` | `twist.twist.linear.x` — линейная скорость центра по тахометрам VESC | позу, `twist.angular.z`, разность бортов (в `odom0_config` включён **только** индекс `vx`) |
| `imu_stm32_bridge` → `/imu/data` | `orientation` — **готовый кватернион** (в `imu0_config` включена только строка ориентации) | гироскоп и акселерометр (не интегрируются, нет двойного учёта) |
| GPS | только `x, y` в `ekf_map` | курс (`yaw`) из GPS не берётся нигде |

Итог: позиция = ∫(скорость VESC) в системе координат, ориентированной по
кватерниону IMU; абсолютная привязка — от GPS. Гусеницы в повороте буксуют,
поэтому колёсный yaw не используется принципиально.

Параллельно работает узел `robot_odom` (`/odom`) — та же математика без
фильтра Калмана. Он нужен как **прозрачный эталон** для калибровки и для
режима без Nav2; TF он не публикует (`odom_publish_tf:=false`), и Nav2 его
не использует.

### Объезд препятствий

Лидар `/scan` → костмапы Nav2 (`voxel_layer`/`obstacle_layer`, окно 10×10 м
локальная и 24×24 м глобальная, разрешение 0.1 м) → планировщик NavFn
перестраивает путь вокруг препятствий (1 Гц) → контроллер DWB (10 Гц) ведёт
робота, отталкиваясь от раздутого слоя (`inflation_radius`).

Лучи без препятствия приходят как `+inf` (`invalid_range_is_inf: true` в
`ydlidar_params.yaml`) и **чистят** костмап — иначе однажды увиденное
препятствие остаётся на карте навсегда.

Дерево поведения (`config/behavior_tree.xml`): при неудаче — `Wait` 5 с и
`BackUp` 30 см; **`Spin` намеренно отсутствует** (гусеницы на грунте при
вращении роют ямы).

### Безопасность

* Команды Nav2 идут в `/cmd_vel/auto`, где `cmd_switcher` уступает пульту
  ELRS: оператор перехватывает управление мгновенно, без переключения режимов.
* `kolesa_control` останавливает гусеницы через 0.5 с после пропажи команд
  (плюс штатный таймаут VESC).
* При потере GPS-фикса `ekf_map` перестаёт обновлять `map→odom`, Nav2
  теряет TF и останавливает робота; при возврате фикса продолжает.
* `xy_goal_tolerance: 2.0 м` согласован с точностью бытового GPS.

## Состав запуска

| Слой | Файл | Что поднимает |
|---|---|---|
| железо | `project_start/launch/start.launch.py` | ELRS, kolesa_control, IMU, GNSS, лидар, robot_state_publisher, cmd_switcher, relay_reliable, robot_odom |
| навигация | `gps_navigator/launch/navigation.launch.py` | ekf_odom, ekf_map, navsat_transform, controller/planner/behavior/bt_navigator/waypoint_follower, lifecycle_manager, gps_mission |
| всё вместе | `gps_navigator/launch/route.launch.py` | оба слоя |

## Параметры, которые чаще всего нужно править

| Файл | Параметр | Зачем |
|---|---|---|
| `ekf_localization.yaml` | `imu0_yaw_offset` (в обоих EKF) | угол установки платы IMU |
| `ekf_localization.yaml` | `magnetic_declination_radians` | склонение (по умолчанию 0 — его уже учла прошивка STM32) |
| `nav2_params.yaml` | `robot_radius`, `inflation_radius` | габарит робота и запас |
| `nav2_params.yaml` | `max_vel_x`, `max_vel_theta`, `acc_lim_*` | динамика (согласована с `max_linear_velocity` в kolesa_control) |
| `nav2_params.yaml` | `xy_goal_tolerance` | радиус «прибыл» (≥ точности GPS) |
| `nav2_params.yaml` | `obstacle_max_range`, `raytrace_max_range` | дальность реакции на препятствия |
| `waypoints.yaml` | точки маршрута | сам маршрут |

## Тесты

```bash
cd ~/ros2_ws1809
python3 -m pytest src/robot_odom/test src/gps_mission/test src/gps_navigator/test
```

Тесты проверяют в том числе:
* `robot_odom`: путь интегрируется из `twist.linear.x`, курс — из кватерниона,
  `twist.angular.z` от колёс игнорируется, watchdog обнуляет скорость при
  пропаже VESC (исполняется настоящий код узла на заглушках rclpy);
* конфиги: в EKF включён **только** `vx` из `/odom/vesc` и **только** строка
  ориентации из IMU, ориентация из GPS выключена, TF делят два EKF, частоты
  Nav2 ограничены под Pi 5, `Spin` выключен, допуск цели ≥ точности GPS;
* launch-файлы: аргументы связаны, команды Nav2 идут в `/cmd_vel/auto`,
  `odom_publish_tf:=false`.
