# robot_odom

Одометрия гусеничного робота: **путь — из колёс, курс — из IMU**.

**Роль в стеке GPS-маршрута:** узел — *прозрачный эталон*. В режиме маршрута
(`gps_navigator/launch/route.launch.py`) TF и локализацию считает
`robot_localization` (два EKF + `navsat_transform`) с теми же принципами:
`vx` из `/odom/vesc`, ориентация из кватерниона `/imu/data`. `robot_odom`
при этом публикует `/odom` для сверки и калибровки, но **TF не издаёт**
(`odom_publish_tf: false`), чтобы не конфликтовать с EKF. Nav2 его не
использует. Для работы без Nav2 (ручное катание, стенд) достаточно
`start.launch.py`.

```
/odom/vesc (kolesa_control)  ── twist.linear.x ──┐
                                                 ├─►  /odom  (+ TF odom->base_link опц.)
/imu/data  (imu_stm32_bridge) ── кватернион ─────┘
```

## Принципы (по ТЗ)

1. **Линейное перемещение — только из `kolesa_control`**
   (`/odom/vesc`, поле `twist.twist.linear.x` — скорость центра робота по
   тахометрам VESC). Позиция `x, y` интегрируется этой скоростью.
2. **Повороты — только из IMU, из готового кватерниона**
   (`/imu/data`, `orientation`; ENU-фьюжн считается на STM32).
   Разность бортов / угловая скорость колёс не используется нигде:
   гусеницы в повороте буксуют. Гироскоп тоже не интегрируется —
   курс каждый такт берётся из кватерниона заново, дрейф не копится.

## Топики

| Направление | Топик | Тип |
|---|---|---|
| вход | `/odom/vesc` | `nav_msgs/Odometry` (только `twist.linear.x`) |
| вход | `/imu/data` | `sensor_msgs/Imu` |
| выход | `/odom` | `nav_msgs/Odometry` |
| выход | `/tf` (опц.) | `odom -> base_link`, только при `publish_tf: true` |

## Ключевые параметры

| Параметр | Умолч. | Описание |
|---|---|---|
| `yaw_mode` | `absolute` | `absolute` — курс ENU (0 = восток), нужен для GPS; `relative` — 0 при старте |
| `yaw_offset_deg` | `-48.0` | поправка угла монтажа IMU (см. калибровку ниже) |
| `publish_tf` | `false` | `true` только БЕЗ robot_localization / slam_toolbox |
| `max_dt` | `0.25` | ограничение шага интегрирования, с |
| `vesc_timeout` / `imu_timeout` | `0.5` | стоп-защита при устаревании входов, с |

## Калибровка yaw_offset_deg

1. Запустите железо: `ros2 launch project_start start.launch.py`.
2. Поставьте робота так, чтобы его «нос» смотрел точно на **восток**
   (по компасу / карте).
3. Смотрите yaw: `ros2 topic echo /odom --field pose.pose.orientation` и
   переведите кватернион в угол (или смотрите `gps_navigator`-статус).
4. Подберите `yaw_offset_deg` так, чтобы yaw стал ≈ 0. Поворот робота
   **против** часовой стрелки должен **увеличивать** yaw.
5. Пропишите значение в `start.launch.py` (`imu_yaw_offset_deg`) или в
   `config/odom_params.yaml`.

## CPU

Работа только в колбэках (телеметрия VESC 20 Гц задаёт частоту `/odom`),
watchdog — таймер 2 Гц. Потребление — доли процента CPU на Raspberry Pi 5.
