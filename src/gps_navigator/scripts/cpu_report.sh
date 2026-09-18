#!/usr/bin/env bash
# cpu_report.sh — замер загрузки CPU навигационного стека на Raspberry Pi 5.
#
#   bash cpu_report.sh          # 15 секунд наблюдений
#   bash cpu_report.sh 60       # 60 секунд
#
# Показывает: loadavg, температуру SoC, долю CPU по процессам ROS,
# суммарную долю и вердикт по порогам из docs/CPU_RPI5.md.

set -u
DURATION="${1:-15}"
INTERVAL=2

echo "=== Загрузка CPU стека GPS-маршрута (${DURATION} с) ==="

if [ -f /proc/loadavg ]; then
  read -r L1 L5 L15 _ < /proc/loadavg
  NCPU=$(grep -c ^processor /proc/cpuinfo)
  echo "loadavg: ${L1} / ${L5} / ${L15}  (ядер: ${NCPU})"
fi
if [ -f /sys/class/thermal/thermal_zone0/temp ]; then
  TEMP=$(awk '{printf "%.1f", $1/1000}' /sys/class/thermal/thermal_zone0/temp)
  echo "Температура SoC: ${TEMP} °C"
fi
if command -v vcgencmd >/dev/null 2>&1; then
  echo "Троттлинг: $(vcgencmd get_throttled 2>/dev/null)"
fi

echo
echo "Наблюдение ${DURATION} с..."
OUT=$(top -b -d "$INTERVAL" -n "$((DURATION / INTERVAL + 1))" -w 512 2>/dev/null \
      | grep -E "ekf_|navsat|controller_serv|planner_serv|behavior_serv|bt_navig|waypoint_follow|kolesa|bridge_node|ydlidar|robot_odom|gps_mission|cmd_mux|relay_node|elrs|robot_state" \
      | awk '{name=$12; cpu[name]+=$9; n[name]++} END {for (k in cpu) printf "%-28s %6.1f %%\n", k, cpu[k]/n[k]}' \
      | sort -k2 -nr)

if [ -z "${OUT}" ]; then
  echo "Ни один процесс стека не найден — запущен ли route.launch.py?"
  exit 2
fi

printf "%-28s %8s\n" "ПРОЦЕСС" "CPU (средн.)"
echo "${OUT}"

TOTAL=$(echo "${OUT}" | awk '{s+=$2} END {printf "%.1f", s}')
echo
echo "Суммарно по стеку: ${TOTAL} % одного ядра (всего доступно $((NCPU * 100)) %)"

# Вердикт по порогам из docs/CPU_RPI5.md
awk -v total="${TOTAL}" -v ncpu="${NCPU:-4}" -v temp="${TEMP:-0}" 'BEGIN {
  bad = 0
  if (total > 250) { print "ПЛОХО: стек ест больше 2.5 ядра — см. раздел «Если CPU всё равно мало»"; bad = 1 }
  else if (total > 120) { print "ВНИМАНИЕ: больше 1.2 ядра — есть запас, но проверьте частоты Nav2" ; bad = 0 }
  else { print "НОРМА: загрузка в пределах ожидаемой" }
  if (temp+0 > 75) { print "ПЛОХО: SoC горячее 75 °C — возможен троттлинг, нужно охлаждение"; bad = 1 }
  exit bad
}'
