# Как подготовить робота к движению по GPS-маршруту

Цель документа — по шагам довести робота от «включил» до «поехал по маршруту»
и понять на каждом шаге, **что именно проверяется и каким результатом**.

Схема стека (подробнее — `OUTDOOR_NAV.md`):

```
kolesa_control ──/odom/vesc (ТОЛЬКО linear.x)──┐
                                               ├─► ekf_odom ─► TF odom→base_link, /odometry/filtered
imu_stm32_bridge ─/imu/data (кватернион)───────┘            └► ekf_map ──► TF map→odom
nmea_navsat_driver ─/gps/fix─► navsat_transform ─/odometry/gps─┘   + сервисы /fromLL /datum
lidar ─/scan─► Nav2 (костмапы, NavFn, DWB) ─/cmd_vel_nav─► /cmd_vel/auto ─► cmd_switcher ─► /cmd_vel
waypoints.yaml ─► gps_mission ─► Nav2 follow_gps_waypoints
```

> **Правило подготовки:** каждый пункт заканчивается конкретной командой и
> конкретным ожидаемым результатом. Если результат не такой — не идите дальше,
> чините. В конце — чек-лист одной таблицей.

---

## Шаг 0. Железо и кабели

| Что | Как проверить | Норма |
|---|---|---|
| Питание Pi 5 | `vcgencmd get_throttled` | `throttled=0x0` (не `0x50000` — значит просадка питания) |
| UART включены | `ls -l /dev/ttyAMA*` | есть `ttyAMA0..ttyAMA4` |
| Лидар | `ls -l /dev/ttyUSB*` | есть `/dev/ttyUSB0` |
| Батарея | диагностика VESC (см. шаг 4) | ≥ 42 В для 12S под нагрузкой |

Если `/dev/ttyAMA1..4` нет — в `/boot/firmware/config.txt` должны быть
оверлеи `dtoverlay=uart1`, `uart2`, `uart3`, `uart4` и
`enable_uart=1` (см. `imu_stm32_bridge/docs/USB_UART_RASPBERRY_PI.md`),
после правки — перезагрузка.

---

## Шаг 1. Сборка

```bash
cd ~/ros2_ws1809
sudo apt update
sudo apt install -y ros-jazzy-robot-localization ros-jazzy-nav2-bringup \
     ros-jazzy-nav2-waypoint-follower ros-jazzy-nav2-navfn-planner \
     ros-jazzy-nav2-dwb-controller ros-jazzy-nmea-navsat-driver \
     ros-jazzy-geographic-msgs python3-serial python3-yaml
colcon build --symlink-install
source install/setup.bash
```

Проверка: `ros2 pkg list | grep -E "gps_navigator|gps_mission|robot_odom"` —
все три пакета видны.

---

## Шаг 2. IMU: калибровка и курс

Курс берётся **только** из кватерниона IMU, поэтому IMU — самый критичный
датчик.

1. **Гироскоп.** Не трогать робота ~2 с после подачи питания (автокалибровка).
2. **Магнитометр** — обязательно на собранном роботе, рядом с моторами:

   ```bash
   ros2 launch project_start start.launch.py
   ros2 service call /imu/imu_stm32_bridge/mag_calib_start std_srvs/srv/Trigger
   # медленно вращать робота ЦЕЛИКОМ ~30 с: оборот вокруг вертикали,
   # наклоны вперёд/назад/вбок, по возможности вверх ногами
   ros2 service call /imu/imu_stm32_bridge/mag_calib_stop_save std_srvs/srv/Trigger
   ```

3. **Частота и валидность кватерниона:**

   ```bash
   ros2 topic hz /imu/data
   ```
   Норма: 25–50 Гц, без пропусков.

4. **Поправка угла установки (`imu0_yaw_offset`, `imu_yaw_offset_deg`).**
   Поставьте робота «носом» точно на **восток** (компас/карта) и запустите
   интерактивную проверку:

   ```bash
   ros2 run gps_navigator preflight_check --ros-args -p quick:=false
   ```
   Узел попросит: «нос на восток → Enter», затем «повернуть на 90° против
   часовой → Enter».

   | Результат узла | Что делать |
   |---|---|
   | `Курс: поворот на 90° ... дал +90°` и `yaw при «носе на восток»: ≈0°` | всё верно |
   | yaw при востоке ≠ 0 | поправить `imu0_yaw_offset` (рад) в `gps_navigator/config/ekf_localization.yaml` **и** `imu_yaw_offset_deg` (град) в `start.launch.py` — значения должны соответствовать друг другу |
   | поворот дал −90° | перепутан знак оси Z: проверьте ориентацию платы и `imu_rpy` в URDF |

5. **Проверка истинного севера (для GPS).** Проедьте 10 м строго на север.
   В RViz (`ros2 launch gps_navigator route.launch.py` → RViz с TF и путём)
   трек должен идти вдоль **+Y** фрейма `map`. Если трек повёрнут примерно на
   величину магнитного склонения — значит склонение учтено дважды или не учтено:
   см. шаг 3, пункт про `declination_deg`.

---

## Шаг 3. GPS

1. Антенна — под открытым небом, вдали от карбоновых пластин и проводов ВМ.
2. Прогрев 1–3 минуты (холодный старт — до 5 минут).
3. Проверка:

   ```bash
   ros2 topic echo /gps/fix --once
   ```

   | Поле | Норма |
   |---|---|
   | `status.status` | `0` (фикс) или больше; `-1` — фикса нет |
   | `latitude/longitude` | ваши реальные координаты, не `0.0` |
   | `position_covariance[0]` | чем меньше, тем лучше; `sqrt(cov[0])/5 ≈ HDOP ≤ 4` |

4. **Магнитное склонение.** Прошивка STM32 уже применяет склонение при
   фьюжне (команда `SET_DECLINATION`/`SAVE_FLASH`, см. `docs/CALIBRATION.md`).
   Поэтому в `route.launch.py` по умолчанию `declination_deg:=0.0` — иначе
   поправка применится дважды и маршрут «повернёт» на величину склонения
   (≈12° для Москвы). Проверка — пункт 5 шага 2. Если трек уходит в сторону —
   запустите с `declination_deg:=11.9` (своё значение:
   <https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml>).

---

## Шаг 4. Привод и одометрия

**Принцип:** линейное перемещение берётся из `kolesa_control`
(`/odom/vesc`, `twist.linear.x`), повороты — из IMU. Поэтому калибруется
только масштаб пути.

1. **Связь с VESC и направление.** Поставьте робота на подставку:

   ```bash
   ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
   ```
   Обе гусеницы должны крутиться **вперёд**. Если одна назад — инверсия
   (`invert_left/invert_right`, `encoder_invert_*` в `start.launch.py`).

2. **Телеметрия жива:**

   ```bash
   ros2 topic echo /kolesa/diagnostics --once
   ```
   Норма: `connected=True`, `fault=0`, `telemetry_age_s` < 0.2.

3. **Масштаб одометрии (обязательно!).** Отмерьте рулеткой **20 м** прямой,
   проедьте её с пульта и запустите замер:

   ```bash
   ros2 run gps_navigator preflight_check --ros-args -p distance_test:=20.0
   ```
   Узел напечатает: `по /odometry/filtered X м ... Поправка: odometry_scale *= K`.
   Умножьте `odometry_scale` в `start.launch.py` на `K`, пересоберите и
   повторите замер — должно стать 20.0 ± 0.3 м.

   *Контроль «сырой» одометрии без Nav2:* `ros2 topic echo /odom --field pose.pose.position`
   (узел `robot_odom` — та же математика: путь VESC + курс IMU).

4. **Курс не зависит от колёс.** Покрутите робота на месте (гусеницы в
   противоход): `/odom/vesc` даёт `linear.x ≈ 0`, а позиция в `/odom` не
   меняется — меняется только ориентация. Это и есть требование «не
   использовать данные о поворотах из kolesa_control».

---

## Шаг 5. Лидар

```bash
ros2 topic hz /scan
ros2 run gps_navigator preflight_check --ros-args -p quick:=true
```

| Проверка | Норма |
|---|---|
| частота | ≈ 10 Гц |
| доля валидных лучей | > 30 % (на открытой площадке часть лучей уходит в небо — это нормально, они становятся `+inf` и чистят костмап) |
| `invalid_range_is_inf` в `project_start/params/ydlidar_params.yaml` | `true` (иначе костмап «зарастает» призраками) |

Проверка в RViz: вращение робота не должно «размазывать» стены; препятствие
появляется и **исчезает** после того, как его убрали.

---

## Шаг 6. Локализация и TF

```bash
ros2 run tf2_tools view_frames   # или preflight_check
```

| Что | Норма |
|---|---|
| `odom → base_link` | публикует **ekf_odom** |
| `map → odom` | публикует **ekf_map** |
| `base_link → imu_link/gps_link/laser_frame` | `robot_state_publisher` (статические) |
| два издателя одного TF | недопустимо: `odom_publish_tf:=false` у `robot_odom` |

Проверка сервиса, через который Nav2 переводит GPS-точки в метры:

```bash
ros2 service call /fromLL robot_localization/srv/FromLL \
  "{ll_point: {latitude: 56.29913, longitude: 43.92291, altitude: 0.0}}"
```
Норма: `map_point` с конечными числами (не `nan`). Если `nan` — не установлен
datum (его ставит `gps_mission` при старте миссии) или нет GPS-фикса.

---

## Шаг 7. Маршрут

`project_start/config/waypoints.yaml` — точки `lat/lon` (+ необязательный
`radius`).

**Как записать маршрут, проезжая его:**

```bash
ros2 launch gps_navigator route.launch.py
# едем с пульта по маршруту; в нужных точках:
ros2 service call /gps_mission/record_waypoint std_srvs/srv/Trigger
# точки дописываются в ~/gps_route.yaml
```

Правила:
- точки каждые **10–30 м**, на поворотах — чаще;
- не ближе **3 м** к препятствиям и краям (точность GPS 2–5 м + объезд);
- ширина планируемого коридора должна быть ≥ 2 м (иначе Nav2 не найдёт проход);
- не планируйте маршрут по высокой траве/кюветам: лидар 2D не видит ямы и
  провалы — это ответственность оператора.

Проверка файла:

```bash
ros2 run gps_navigator preflight_check --ros-args \
  -p waypoints_file:=~/ros2_ws1809/src/project_start/config/waypoints.yaml \
  -p quick:=true
```
Норма: `Маршрут: N точек, длина ~M м` и `До первой точки маршрута X м`
(меньше 200 м — иначе вы запускаете чужой маршрут).

---

## Шаг 8. Пульт и приоритеты (безопасность)

`cmd_switcher` отдаёт приоритет: **пульт ELRS** → приложение → «домой» →
автопилот. Проверка **обязательна до первого автономного заезда**:

1. `ros2 launch gps_navigator route.launch.py`, старт миссии (см. ниже).
2. Когда робот едет сам — возьмите пульт и дайте газ: робот обязан
   мгновенно перейти под ручное управление.
3. Отпустите стики — робот снова продолжит маршрут (автопилот публикует
   команды постоянно, `cmd_switcher` переключится обратно через ~0.2 с).
4. Проверьте, что при пропадании связи с пультом `kolesa_control`
   останавливает гусеницы через `cmd_timeout` (0.5 с).

---

## Шаг 9. Предполётная проверка одним запуском

```bash
ros2 run gps_navigator preflight_check --ros-args \
  -p waypoints_file:=~/ros2_ws1809/src/project_start/config/waypoints.yaml
```

Узел проверяет: `/odom/vesc`, кватернион `/imu/data`, `/odometry/filtered`,
TF `odom→base_link` и `map→odom`, `/gps/fix` и HDOP, сервис `/fromLL`,
`/scan`, файл маршрута, loadavg и температуру Pi 5, а также диагностику EKF.

**Запускать маршрут можно только при `ИТОГ: готово к движению по маршруту`
(код возврата 0).**

---

## Шаг 10. Запуск маршрута

```bash
# терминал 1
ros2 launch gps_navigator route.launch.py
# терминал 2 — наблюдение
ros2 topic echo /gps_mission/status
ros2 topic echo /cmd_vel
# терминал 3 — старт
ros2 service call /gps_mission/start std_srvs/srv/Trigger
```

Первые 10–20 секунд робот стоит: Nav2 ждёт TF, костмап должен наполниться.

Остановка/пауза:

```bash
ros2 service call /gps_mission/stop std_srvs/srv/Trigger    # отмена миссии
ros2 topic pub -1 /cmd_vel/home geometry_msgs/msg/Twist "{}"  # гарантированный стоп
```

---

## Шаг 11. Контроль в движении

| Наблюдаем | Команда | Тревожный признак |
|---|---|---|
| миссия | `ros2 topic echo /gps_mission/status` | долго висит одна точка |
| команды | `ros2 topic echo /cmd_vel` | `linear.x` прижато к 0 при свободной дороге |
| поправка GPS↔одометрия | `/gps_mission/status`, поле `corr` | растёт больше 3–5 м — плохой GPS |
| температура/CPU | `bash src/gps_navigator/scripts/cpu_report.sh` | > 75 °C или loadavg > 3 |
| костмапы | RViz: `/local_costmap/costmap`, `/global_costmap/costmap` | «заросли» без препятствий |

Первый выезд: `cruise_speed:=0.35`, маршрут 2–3 точки, оператор с пультом в
руках в 5–10 м от робота.

---

## Чек-лист перед выездом

- [ ] `vcgencmd get_throttled` = `0x0`
- [ ] `/dev/ttyAMA0..4` и `/dev/ttyUSB0` на месте
- [ ] магнитометр откалиброван на собранном роботе
- [ ] поворот на 90° против часовой даёт `+90°` в `/odom`; при «носе на восток» yaw ≈ 0
- [ ] проезд 10 м на север идёт вдоль `+Y` в `map` (склонение не задвоено)
- [ ] GPS: фикс, HDOP ≤ 4
- [ ] одометрия: 20 м по рулетке = 20 ± 0.3 м по `/odometry/filtered`
- [ ] лидар ~10 Гц, препятствие исчезает с костмапа после уборки
- [ ] TF: `map→odom` (ekf_map), `odom→base_link` (ekf_odom), дублей нет
- [ ] `/fromLL` возвращает числа
- [ ] пульт перехватывает управление на ходу
- [ ] `preflight_check` — 0 ошибок
- [ ] скорость первого заезда ≤ 0.35 м/с, оператор с пультом рядом

---

## Типовые неисправности

| Симптом | Причина | Лечение |
|---|---|---|
| Робот крутится на месте и не едет | нет TF `map→odom` или `/odometry/gps` | проверить `ekf_map`, `navsat_transform`, GPS-фикс |
| Едет не в ту сторону/развёрнут на ~90–180° | `imu0_yaw_offset` / знак оси | шаг 2, п. 4 |
| Путь в RViz «уезжает» на десятки метров | нет фикса → EKF ушёл на чистой одометрии | ждать фикс, проверить антенну |
| Робот останавливается посреди поля | костмап «зарос» (нет `+inf`-очистки) или цель в инфляции | `invalid_range_is_inf: true`, увеличить `inflation_radius` наоборот уменьшить |
| «Проезда нет» на ровном месте | `robot_radius`/`inflation_radius` больше реальной ширины прохода | уменьшить в `nav2_params.yaml` |
| Робот роет ямы при развороте | включён Spin | уже выключен: в `behavior_plugins` только `backup`, `wait` |
| Дёргается на прямой | GPS «прыгает», малая `gps_correction_tau` не виновата — смотрите `xy_goal_tolerance`, `lookahead` | увеличить допуск, снизить скорость |
| Pi 5 троттлит | нет радиатора/обдува, высокая частота Nav2 | `docs/CPU_RPI5.md` |
