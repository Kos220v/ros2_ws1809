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

## Курс по GPS (антидрейф)

Узел `gps_heading` (запускается в `navigation.launch.py`) пока робот едет —
сместился минимум на `min_dist_m` (2 м) за окно `window_s` (3 с) — публикует
в `/gps/heading` направление фактического перемещения между фиксами
(`PoseWithCovarianceStamped`, frame `map`, заполнен только yaw; дисперсия
yaw растёт на малой скорости). `ekf_map` берёт его как `pose0` (только yaw).

Зачем: курс из кватерниона IMU дрейфует (на текущей плате — десятки °/мин),
а курс по перемещению GPS — независимое абсолютное наблюдение. Пока робот
едет, ошибка курса в `map` не накапливается; на стоянке оценки нет — там
дрейф и не мешает (позиция не интегрируется). Точность «якоря» — единицы
градусов (шум GPS), поэтому кватернион IMU остаётся основным источником
между GPS-обновлениями: EKF их просто взвешивает.

Требования согласованы: оси `map` = восток/север без поворота (datum
ставится с единичной ориентацией), курс GNSS — истинный, магнитное
склонение узлу не нужно. Защиты: отброс «прыжков» GPS (> max_speed_mps
между фиксами), сброс окна при откате времени (NTP), отсутствие публикации
на стоянке и при плохом статусе фикса.

## Перед заездом: убедиться, что сборка свежая

Три выстрела себе в ногу из одной пушки: код обновлён в git, но на роботе
запущена старая сборка. Проверка перед каждым заездом:

```bash
cd ~/ros2_ws
git log --oneline -1                       # ждём последний коммит из репозитория
colcon build --packages-select gps_mission gps_navigator
source install/setup.bash
# установленная копия содержит последний фикс? (>0 — да)
grep -c probe_geo_serialization install/gps_mission/lib/python3.12/site-packages/gps_mission/gps_mission_node.py
```

## Известная проблема: abort «geo_pose convert_from_py Assertion failed»

Симптом: при старте маршрута процесс `gps_mission_node` падает с
`Assertion 'strncmp("geographic_msgs.msg._geo_pose.GeoPose", ...)' failed`
(обычно сразу после строки «datum -> ...»). Это падение **внутри
генерированных ROS-привязок** (rosidl_generator_py) при сериализации
запроса `SetDatum` — в окружении найдена вторая, устаревшая копия
`geographic_msgs` (например, остатки старой сборки в underlay или
`~/.local/lib/python3.12/site-packages`). Код gps_mission скалярные поля
только заполняет — причина всегда снаружи.

Что сделано в коде: вызов `SetDatum` вынесен в одноразовый подпроцесс
(`ros2 service call`), падение такого процесса узел gps_mission не
затрагивает; если datum не установится, navsat_transform возьмёт ноль
`map` по первому GPS-фиксу — маршрут поедет и так.

Найти виновника в окружении:

```bash
python3 -c "import geographic_msgs.msg as g; print(g.__file__); print(g.GeoPose)"
python3 -c "from robot_localization.srv import SetDatum; r=SetDatum.Request(); print(type(r.geo_pose))"
```

Ожидается `<class 'geographic_msgs.msg._geo_pose.GeoPose'>` и путь в
`/opt/ros/jazzy/...`. Если класс печатается как
`geographic_msgs.msg.GeoPose` (без `_geo_pose`) или файл лежит вне
`/opt/ros/jazzy` — вот эта копия и ломает сериализацию: уберите её из
PYTHONPATH (часто `~/.local/...` или старый `install/` в underlay),
затем `ros2 daemon stop` и перезапуск стека.

Если ОБЕ проверки чистые, а abort всё равно повторяется — конвертация
Python→C сломана на уровне установки ROS (импорт проходит, а
сериализация падает; классика — смешанные версии apt-пакетов после
частичного обновления). **Определяющий тест** (падает только битая
установка; безопасен — уронит лишь собственный процесс зонда):

```bash
python3 -c "from geographic_msgs.msg import GeoPose; from rclpy.serialization import serialize_message; serialize_message(GeoPose())" && echo OK
```

Лечение:

```bash
sudo apt update && sudo apt full-upgrade
sudo apt install --reinstall ros-jazzy-geographic-msgs ros-jazzy-rosidl-generator-py
ros2 daemon stop
# перезапустить стек
```

ВАЖНО: пока конвертация не починена, отправить GeoPose не может НИ один
Python-процесс — ни gps_mission (action-цель содержит 18 GeoPoseStamped),
ни `ros2 service call`, ни `ros2 action send_goal` (все падают с
SIGABRT). C++-часть Nav2 не затронута — поэтому остальные узлы живут.
gps_mission при старте сам зондирует сериализацию: если она сломана, он
НЕ падает, а отклоняет старт с сообщением об apt-лечении (и datum не
пытается отправить — его CLI упал бы так же; navsat берёт datum из
первого фикса, это штатно).

### Datum и «прыжок» TF при его установке

Учтите: установка datum (и смена её опоры) мгновенно сдвигает TF
`map→odom` на разницу между старой и новой привязкой. Если робот стоит
далеко от waypoints[0], в момент вызова datum костмапа на несколько
сканов теряет лидар — то самое предупреждение «Sensor origin ... out of
map bounds» (~0.4 с после строки про datum в логе). Для тестов ВДАЛИ от
маршрута запускайте с `datum_at_first_waypoint:=false` — ноль `map`
встанет по первому GPS-фиксу (у робота), никаких телепортов и километровых
координат:

```bash
ros2 launch gps_navigator route.launch.py datum_at_first_waypoint:=false
```

## Известная проблема: «Sensor origin ... is out of map bounds ... cannot raytrace»

Симптом: глобальная костмапа периодически пишет, что источник скана
(лидар) находится за сотни метров от робота и вне её скользящего окна —
в этом случае она не может «стирать» лучами устаревшие препятствия
(нарастают фантомные стены), Nav2 объезжает то, чего давно нет.

Механика: положение лидара в `map` = TF `map→odom→base_link→laser_frame`
на метке времени скана. `map→odom` держит `ekf_map` и мгновенно
сдвигается на величину скачка позиции GPS. Один мультитрейновый
«прыжок» фикса на N метров телепортирует систему координат: сканы,
пришедшие до скачка, оказываются «датчиком» в N метрах от робота.

Защита в коде: перед navsat_transform стоит узел `gps_fix_gate`
(`/gps/fix → /gps/fix/filtered`): отбрасывает фиксы с плохой ковариацией
(~HDOP, порог `max_h_error_m`, 20 м) и прыжки быстрее `max_jump_mps`
(15 м/с). Ресинхронизация опоры (робота перенесли) — только когда
отброшенные фиксы согласованы МЕЖДУ СОБОЙ (на новом месте они неподвижны
относительно друг друга). Если приёмник непрерывно «бежит» (координаты
меняются на километры в секунду — реальный случай с робота при
проблемной антенне), фиксы блокируются целиком: навигация остаётся на
счислении (vx + кватернион IMU), TF map->odom не телепортируется, а узел
кричит в лог «GPS нестабилен ...». Статистика отбора — раз в 10 с в логе
узла. Питаются фильтром и navsat, и gps_heading.

Диагностика «бегущего» GPS (предупреждение костмапы раз в секунду +
координаты скачут на километры):

```bash
ros2 topic info /gps/fix -v                  # издатель должен быть ОДИН
ros2 topic echo /gps/fix --field position    # lat/lon не должны «бежать»
ros2 topic echo /gps/fix --field status      # статус/quality фикса
ros2 topic echo /gps/fix/filtered            # что дошло до Nav2
```

Если lat/lon «бегут» при стоящем роботе — это железо: антенна не
подключена/плохой контакт, обрыв антенного кабеля, питание приёмника, или
приёмник в помещении. Приёмник без антенны может выдавать «бегущие»
координаты с формально валидным статусом. Проверьте также, что в стеке
вообще работает gps_fix_gate (`ros2 node list | grep gps_fix_gate`) и
что он свежесобран.

Диагностика на месте:

```bash
ros2 topic echo /gps/fix --field position.latitude    # смотреть скачки
ros2 topic echo /gps/fix --field position_covariance  # HDOP-качество
ros2 topic echo /gps/fix/filtered                      # что дошло до Nav2
```

Если предупреждение продолжается при чистом `/gps/fix/filtered` —
проверьте метки времени лидара (`ros2 topic echo /scan --field header.stamp`)
и сверку с `date +%s.%N`: рассинхрон часов тоже даёт этот эффект.

Про большие координаты в этом же сообщении: граница костмапы вида
(41923, 102499) означает, что ноль `map` стоит в waypoint 1 маршрута
(`set_datum_from_first_waypoint: true`), а робот тестируется в ~сотне
километров от него. Математике это не мешает (координаты относительные),
но: (а) preflight предупредит «Робот в N м от первой точки»;
(б) точность float32 в TF на числах ~1e5 — около 1 см, приемлемо;
(в) в RViz робот будет в (41935, 102511) — не пугайтесь. Для тестов
вдали от маршрута можно поставить `set_datum_from_first_waypoint: false`
в `gps_mission/config/mission_params.yaml` — ноль `map` встанет по первому
GPS-фиксу.

## Состав запуска

| Слой | Файл | Что поднимает |
|---|---|---|
| железо | `project_start/launch/start.launch.py` | ELRS, kolesa_control, IMU, GNSS, лидар, robot_state_publisher, cmd_switcher, relay_reliable, robot_odom |
| навигация | `gps_navigator/launch/navigation.launch.py` | ekf_odom, ekf_map, navsat_transform, gps_heading, controller/planner/behavior/bt_navigator/waypoint_follower, lifecycle_manager, gps_mission |
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
