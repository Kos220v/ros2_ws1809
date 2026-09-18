# Raspberry Pi 5: как не перегрузить CPU

## Что уже сделано в проекте (архитектурно)

| Решение | Экономия |
|---|---|
| Не запускаются `smoother_server`, `velocity_smoother`, `collision_monitor`, `map_server`, `amcl`, `slam_toolbox` | −3…6 процессов, −10–20 % CPU |
| Контроллер 10 Гц, планировщик 1 Гц, BT 100 Гц (`bt_loop_duration: 10`) | вместо 20/20 Гц по умолчанию |
| Локальная костмапа 10×10 м @ 0.1 м (10 000 ячеек), глобальная 24×24 м @ 0.1 м, `publish_voxel_map: false` | костмапы — главный потребитель CPU |
| `obstacle_max_range: 8 м`, `raytrace_max_range: 10 м` вместо 12 м | меньше лучей в обработку |
| Костмапы публикуются 1 и 0.5 Гц (нужны только для RViz) | −трафик DDS |
| `always_send_full_costmap: false` | −трафик DDS |
| DWB вместо MPPI | MPPI на Pi 5 съедает 1–2 ядра |
| Nav2 подписан на `/scan` (BEST_EFFORT) напрямую | `relay_reliable` остаётся только для Wi-Fi/RViz |
| `robot_odom` работает только в колбэках, watchdog 2 Гц | доли процента CPU |
| EKF: 20 Гц (odom) и 10 Гц (map) вместо 30 | −2 процесса по ~1–2 % |
| Поведения: только `backup` + `wait` | меньше плагинов и симуляций |

## Настройка системы (делать один раз)

1. **Governor и частоты.** Для робота важнее стабильность, чем энергосбережение:

   ```bash
   echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor
   ```

   Сделать постоянным — `docs/cpu_governor.service` (см. ниже).

2. **Не разгонять выше штатного**: `arm_freq` в `config.txt` лучше не трогать —
   важнее охлаждение.

3. **Охлаждение обязательно.** Активный кулер или большой радиатор: при 80 °C
   Pi 5 снижает частоту, и Nav2 начинает «заикаться» (пропуски циклов
   контроллера → дёрганое движение).

4. **Убрать лишнее из автозапуска**: браузеры, `bluetooth` (если не нужен),
   `avahi`, десктоп-сессия при запуске через systemd.

5. **DDS-трафик**: держите `ROS_DOMAIN_ID` одинаковым на роботе и ноутбуке и
   не запускайте на борту лишние GUI-подписчики на `/scan` и костмапы.

6. **Логи**: `export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] {message}"`
   и не держите `output='screen'` в systemd-юните для всех узлов — пишите в
   journald.

## Как измерить (обязательно до выезда)

```bash
bash src/gps_navigator/scripts/cpu_report.sh          # 15 с наблюдений
bash src/gps_navigator/scripts/cpu_report.sh 60       # минута
```

Скрипт показывает: общий loadavg, температуру SoC, долю CPU по процессам и
итоги. Ориентиры для полностью поднятого стека (железо + Nav2 + RViz **на
борту не запущен**):

| Показатель | Норма | Плохо |
|---|---|---|
| loadavg (1 мин) | ≤ 1.5 | > 3 |
| сумма CPU всех ROS-процессов | ≤ 120 % одного ядра (из 400 %) | > 250 % |
| температура SoC | ≤ 65 °C | > 75 °C |
| `controller_server` | ≤ 40 % ядра | > 80 % |
| костмапы (`planner_server`/`controller_server` в сумме) | ≤ 60 % | > 120 % |
| `ekf_odom` + `ekf_map` + `navsat_transform` | ≤ 20 % | > 50 % |
| пропуски циклов контроллера в логе | нет | есть |

Проверка пропусков цикла контроллера (признак нехватки CPU):

```bash
ros2 launch gps_navigator route.launch.py 2>&1 | grep -i "missed\|took longer"
```

## Если CPU всё равно мало

По порядку, от дешёвого к дорогому:

1. `controller_frequency: 10.0 → 7.0`, `expected_planner_frequency: 1.0 → 0.5`.
2. `vx_samples: 8 → 6`, `vtheta_samples: 10 → 8`, `sim_time: 1.5 → 1.2`.
3. Локальная костмапа `10×10 → 8×8` м (осторожно: должно хватать на
   `raytrace_max_range`).
4. `local_costmap.publish_frequency: 1.0 → 0.0` (если RViz на борту не нужен).
5. `update_frequency` локальной костмапы `3.0 → 2.0`.
6. Отключить `relay_reliable`, если Wi-Fi-наблюдение не нужно
   (`ros2 launch project_start start.launch.py` без него — правится в launch).
7. `robot_odom` не нужен в бою — `use_robot_odom:=false` (эталон только для
   калибровки).
8. RViz запускать **на ноутбуке**, а не на Pi.

## Автозапуск робота (systemd)

```ini
# /etc/systemd/system/robot-route.service
[Unit]
Description=GPS route stack (ros2_ws1809)
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=pi
Environment=HOME=/home/pi
Environment=ROS_DOMAIN_ID=42
ExecStart=/bin/bash -lc 'source /opt/ros/jazzy/setup.bash && \
  source /home/pi/ros2_ws1809/install/setup.bash && \
  exec ros2 launch gps_navigator route.launch.py'
Restart=on-failure
RestartSec=5
# мягкий приоритет, чтобы не убивать систему при пике
Nice=5

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now robot-route.service
journalctl -u robot-route.service -f
```

Миссию при этом стартует либо оператор командой
`ros2 service call /gps_mission/start std_srvs/srv/Trigger`, либо скрипт
автозапуска (добавьте `ExecStartPost` с `sleep 30` и той же командой — но
лучше держать старт под контролем человека).
