# -*- coding: utf-8 -*-

"""
preflight_check — предполётная проверка робота перед GPS-маршрутом.

Проверяет по ТЗ:
    * /odom/vesc       — линейная скорость из kolesa_control жива;
    * /imu/data        — кватернион валиден (курс берётся из него);
    * TF               — есть odom->base_link (ekf_odom) и map->odom (ekf_map);
    * /gps/fix         — фикс и HDOP;
    * /fromLL          — сервис robot_localization отвечает (его использует
                         Nav2 для перевода точек маршрута в map);
    * /scan            — частота, доля валидных лучей, FOV;
    * waypoints.yaml   — читается, маршрут не нулевой, робот рядом с 1-й точкой;
    * система          — loadavg и температура Raspberry Pi 5.

Запуск (железо и навигация уже запущены, см. docs/PREFLIGHT.md):

    ros2 run gps_navigator preflight_check --ros-args \
      -p waypoints_file:=~/ros2_ws1809/src/project_start/config/waypoints.yaml

Режимы:
    по умолчанию                     — телеметрия + интерактивная проверка курса
    -p quick:=true                   — без интерактивных проверок
    -p distance_test:=20.0           — замер одометрии: проехать N метров
                                       по рулетке, сравнить с /odom

Код возврата: 0 — ошибок нет, 1 — есть FAIL.
"""

import math
import os
import threading
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from robot_localization.srv import FromLL
from sensor_msgs.msg import Imu, LaserScan, NavSatFix
from tf2_msgs.msg import TFMessage

from .diag_checks import ekf_warnings

OK = "OK  "
WARN = "WARN"
FAIL = "FAIL"

UNMEASURED_COV = 1.0e6
METERS_PER_DEG_LAT = 111_320.0


def _geodetic_to_local(lat, lon, lat0, lon0):
    x = math.radians(lon - lon0) * 6_378_137.0 * math.cos(math.radians(lat0))
    y = (lat - lat0) * METERS_PER_DEG_LAT
    return x, y


def _load_waypoints(path):
    """Мини-загрузчик маршрута (без зависимостей от других пакетов)."""
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        raise ValueError(f"файл не найден: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or not data.get("waypoints"):
        raise ValueError(f"{path}: нет списка 'waypoints'")
    pts = []
    for item in data["waypoints"]:
        pts.append((float(item["lat"]), float(item["lon"])))
    return pts


class PreflightCheck(Node):
    def __init__(self):
        super().__init__("preflight_check")

        p = self.declare_parameter
        p("collect_seconds", 6.0)
        p("quick", False)
        p("distance_test", 0.0)
        p("waypoints_file", "")
        p("filtered_odom_topic", "/odometry/filtered")
        p("robot_odom_topic", "/odom")
        p("vesc_odom_topic", "/odom/vesc")
        p("imu_topic", "/imu/data")
        p("gps_topic", "/gps/fix")
        p("scan_topic", "/scan")
        p("from_ll_service", "/fromLL")
        p("diagnostics_topic", "/diagnostics")
        p("min_scan_rate", 5.0)
        p("min_imu_rate", 10.0)
        p("min_odom_rate", 10.0)
        p("max_hdop", 4.0)
        p("cpu_load_warn", 2.0)
        p("temp_warn_c", 70.0)

        g = self.get_parameter
        self.collect_s = float(g("collect_seconds").value)
        self.quick = bool(g("quick").value)
        self.distance_test = float(g("distance_test").value)
        self.waypoints_file = str(g("waypoints_file").value)
        self.min_scan_rate = float(g("min_scan_rate").value)
        self.min_imu_rate = float(g("min_imu_rate").value)
        self.min_odom_rate = float(g("min_odom_rate").value)
        self.max_hdop = float(g("max_hdop").value)
        self.cpu_load_warn = float(g("cpu_load_warn").value)
        self.temp_warn_c = float(g("temp_warn_c").value)
        self.from_ll_service = str(g("from_ll_service").value)

        # -------------------------------------------------------- телеметрия
        self.t0 = time.monotonic()
        self.imu_n = 0
        self.imu_q_ok = False
        self.imu_status_word = None
        self.filt_n = 0
        self.filt_yaw = None
        self.filt_vx = None
        self.filt_start = None
        self.filt_last = None
        self.robot_odom_n = 0
        self.vesc_n = 0
        self.vesc_vx_cov = None
        self.vesc_last_vx = None
        self.fix_n = 0
        self.fix = None
        self.scan_n = 0
        self.scan_valid_frac = 0.0
        self.scan_fov_deg = 0.0
        self.scan_range_max = 0.0
        self.tf_parents = {}          # child -> parent
        self.diag_warnings = []

        self.create_subscription(Imu, str(g("imu_topic").value),
                                 self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, str(g("filtered_odom_topic").value),
                                 self._on_filtered, 10)
        self.create_subscription(Odometry, str(g("robot_odom_topic").value),
                                 self._on_robot_odom, 10)
        self.create_subscription(Odometry, str(g("vesc_odom_topic").value),
                                 self._on_vesc, 10)
        self.create_subscription(NavSatFix, str(g("gps_topic").value),
                                 self._on_fix, 10)
        self.create_subscription(LaserScan, str(g("scan_topic").value),
                                 self._on_scan, qos_profile_sensor_data)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 50)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf,
                                 qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray,
                                 str(g("diagnostics_topic").value),
                                 self._on_diag, 10)

        self.from_ll_cli = self.create_client(FromLL, self.from_ll_service)

    # ------------------------------------------------------------- колбэки
    def _on_imu(self, msg: Imu):
        self.imu_n += 1
        q = msg.orientation
        n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if 0.5 < n2 < 1.5:
            self.imu_q_ok = True

    def _on_filtered(self, msg: Odometry):
        self.filt_n += 1
        q = msg.pose.pose.orientation
        self.filt_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.filt_vx = msg.twist.twist.linear.x
        pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        if self.filt_start is None:
            self.filt_start = pos
        self.filt_last = pos

    def _on_robot_odom(self, msg: Odometry):
        self.robot_odom_n += 1

    def _on_vesc(self, msg: Odometry):
        self.vesc_n += 1
        self.vesc_vx_cov = msg.twist.covariance[0]
        self.vesc_last_vx = msg.twist.twist.linear.x

    def _on_fix(self, msg: NavSatFix):
        self.fix_n += 1
        self.fix = msg

    def _on_scan(self, msg: LaserScan):
        self.scan_n += 1
        sampled = msg.ranges[::3]
        valid = 0
        for r in sampled:
            if math.isfinite(r) and msg.range_min < r < msg.range_max:
                valid += 1
        self.scan_valid_frac = valid / max(1, len(sampled))
        self.scan_fov_deg = math.degrees(msg.angle_max - msg.angle_min)
        self.scan_range_max = msg.range_max

    def _on_tf(self, msg: TFMessage):
        for t in msg.transforms:
            self.tf_parents[t.child_frame_id.lstrip("/")] = \
                t.header.frame_id.lstrip("/")

    def _on_diag(self, msg: DiagnosticArray):
        # level в ROS 2 — byte (IDL octet): в Python это bytes длины 1,
        # а не int — приведение делаем в diag_checks (см. там историю).
        self.diag_warnings.extend(ekf_warnings(msg.status))

    # -------------------------------------------------------------- отчёты
    def telemetry_report(self):
        dt = max(0.1, time.monotonic() - self.t0)
        lines = []

        # 1. Линейная скорость — kolesa_control
        if self.vesc_n == 0:
            lines.append((FAIL, "/odom/vesc молчит — kolesa_control не "
                                "публикует одометрию (VESC/UART?)"))
        elif self.vesc_vx_cov is None or self.vesc_vx_cov >= UNMEASURED_COV:
            lines.append((WARN, "/odom/vesc: vx помечен «не измерено» — нет "
                                "телеметрии обоих бортов (робот на стенде?)"))
        else:
            lines.append((OK, f"/odom/vesc: {self.vesc_n / dt:.0f} Гц, "
                              f"vx={self.vesc_last_vx:+.2f} м/с"))

        # 2. Курс — кватернион IMU
        imu_rate = self.imu_n / dt
        if self.imu_n == 0:
            lines.append((FAIL, "IMU: /imu/data молчит — проверьте "
                                "imu_stm32_bridge и /dev/ttyAMA1"))
        elif not self.imu_q_ok:
            lines.append((FAIL, "IMU: кватернион не единичный — курс "
                                "недостоверен"))
        elif imu_rate < self.min_imu_rate:
            lines.append((WARN, f"IMU: низкая частота {imu_rate:.1f} Гц"))
        else:
            lines.append((OK, f"IMU: {imu_rate:.0f} Гц, кватернион валиден"))

        # 3. Локализация: EKF + TF
        if self.filt_n == 0:
            lines.append((FAIL, "/odometry/filtered молчит — ekf_odom "
                                "(robot_localization) не запущен?"))
        else:
            rate = self.filt_n / dt
            level = OK if rate >= self.min_odom_rate else WARN
            lines.append((level, f"/odometry/filtered: {rate:.0f} Гц, "
                                 f"yaw={math.degrees(self.filt_yaw or 0.0):+.0f}°, "
                                 f"vx={self.filt_vx or 0.0:+.2f} м/с"))
        if self.robot_odom_n > 0:
            lines.append((OK, f"robot_odom (/odom): {self.robot_odom_n / dt:.0f} Гц "
                              "— эталон для сверки (в Nav2 не используется)"))

        need_tf = [("odom", "base_link", "ekf_odom"),
                   ("map", "odom", "ekf_map")]
        for parent, child, who in need_tf:
            got = self.tf_parents.get(child)
            if got == parent:
                lines.append((OK, f"TF {parent} -> {child} есть ({who})"))
            elif got is None:
                lines.append((FAIL, f"TF {parent} -> {child} НЕТ — {who} не "
                                    "публикует (проверьте navigation.launch.py)"))
            else:
                lines.append((FAIL, f"TF {child} публикует '{got}' вместо "
                                    f"'{parent}' — конфликт издателей TF"))

        # 4. GPS + сервис /fromLL
        if self.fix_n == 0:
            lines.append((FAIL, "/gps/fix молчит — для уличного маршрута "
                                "обязателен (/dev/ttyAMA2, антенна, 1-3 мин "
                                "прогрев под открытым небом)"))
        elif self.fix.status.status < 0:
            lines.append((WARN, f"GPS: {self.fix_n / dt:.1f} Гц, но НЕТ фикса "
                                "(status<0) — небо/антенна"))
        else:
            try:
                hdop = math.sqrt(max(0.0, self.fix.position_covariance[0])) / 5.0
            except IndexError:
                hdop = 0.0
            if hdop > self.max_hdop:
                lines.append((WARN, f"GPS: фикс есть, HDOP≈{hdop:.1f} > "
                                    f"{self.max_hdop} — плохое небо"))
            else:
                lines.append((OK, f"GPS: фикс, HDOP≈{hdop:.1f}, "
                                  f"{self.fix.latitude:.6f}, "
                                  f"{self.fix.longitude:.6f}"))

        lines.extend(self._from_ll_report())

        # 5. Лидар
        if self.scan_n == 0:
            lines.append((FAIL, "/scan молчит — лидар не запущен "
                                "(lidar_delay, /dev/ttyUSB0?)"))
        else:
            rate = self.scan_n / dt
            if rate < self.min_scan_rate:
                lines.append((WARN, f"Лидар: низкая частота {rate:.1f} Гц"))
            elif self.scan_valid_frac < 0.3:
                lines.append((WARN, f"Лидар: только {self.scan_valid_frac:.0%} "
                                    "лучей с валидной дальностью"))
            else:
                lines.append((OK, f"Лидар: {rate:.0f} Гц, FOV "
                                  f"{self.scan_fov_deg:.0f}°, "
                                  f"{self.scan_valid_frac:.0%} валидных лучей, "
                                  f"до {self.scan_range_max:.0f} м"))

        # 6. Диагностика EKF
        if self.diag_warnings:
            seen = set()
            for w in self.diag_warnings:
                if w not in seen:
                    seen.add(w)
                    lines.append((WARN, f"Диагностика: {w}"))
        return lines

    def _from_ll_report(self):
        """Проверка сервиса /fromLL (им пользуется Nav2 для точек маршрута)."""
        if not self.from_ll_cli.wait_for_service(timeout_sec=2.0):
            return [(FAIL, f"Сервис {self.from_ll_service} недоступен — "
                           "navsat_transform не запущен, Nav2 не сможет "
                           "перевести GPS-точки в map")]
        lat, lon = 56.0, 44.0
        if self.fix is not None and self.fix.status.status >= 0:
            lat, lon = self.fix.latitude, self.fix.longitude
        elif self.waypoints_file:
            try:
                lat, lon = _load_waypoints(self.waypoints_file)[0]
            except (ValueError, KeyError, IndexError):
                pass
        req = FromLL.Request()
        req.ll_point.latitude = lat
        req.ll_point.longitude = lon
        req.ll_point.altitude = 0.0
        future = self.from_ll_cli.call_async(req)
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return [(WARN, f"{self.from_ll_service} не ответил за 3 с "
                           "(нет datum/фикса?)")]
        pt = future.result().map_point
        if not (math.isfinite(pt.x) and math.isfinite(pt.y)):
            return [(WARN, f"{self.from_ll_service} вернул нечисловые "
                           "координаты — datum не установлен?")]
        return [(OK, f"{self.from_ll_service} работает: "
                     f"({lat:.6f},{lon:.6f}) -> map ({pt.x:.1f}, {pt.y:.1f}) м")]

    def waypoints_report(self):
        if not self.waypoints_file:
            return [(WARN, "waypoints_file не задан — маршрут не проверен")]
        try:
            pts = _load_waypoints(self.waypoints_file)
        except ValueError as e:
            return [(FAIL, f"Файл маршрута: {e}")]

        lat0, lon0 = pts[0]
        length = 0.0
        for (la, lo), (lb, lb2) in zip(pts, pts[1:]):
            ax, ay = _geodetic_to_local(la, lo, lat0, lon0)
            bx, by = _geodetic_to_local(lb, lb2, lat0, lon0)
            length += math.hypot(bx - ax, by - ay)
        lines = [(OK, f"Маршрут: {len(pts)} точек, длина ~{length:.0f} м")]

        if self.fix is not None and self.fix.status.status >= 0:
            dx, dy = _geodetic_to_local(self.fix.latitude, self.fix.longitude,
                                        lat0, lon0)
            d0 = math.hypot(dx, dy)
            if d0 > 200.0:
                lines.append((WARN, f"Робот в {d0:.0f} м от первой точки — "
                                    "тот ли waypoints.yaml?"))
            else:
                lines.append((OK, f"До первой точки маршрута {d0:.1f} м"))
        return lines

    def system_report(self):
        lines = []
        try:
            with open("/proc/loadavg", encoding="utf-8") as f:
                load1 = float(f.read().split()[0])
            with open("/proc/cpuinfo", encoding="utf-8") as f:
                ncpu = max(1, f.read().count("processor\t:"))
            if load1 > self.cpu_load_warn * ncpu:
                lines.append((WARN, f"Загрузка CPU высокая: loadavg={load1:.2f} "
                                    f"при {ncpu} ядрах — см. docs/CPU_RPI5.md"))
            else:
                lines.append((OK, f"Загрузка CPU: loadavg={load1:.2f} "
                                  f"({ncpu} ядер, {load1 / ncpu * 100:.0f}% ядра)"))
        except OSError:
            lines.append((WARN, "Не удалось прочитать /proc/loadavg"))

        try:
            with open("/sys/class/thermal/thermal_zone0/temp",
                      encoding="utf-8") as f:
                temp = int(f.read().strip()) / 1000.0
            if temp > self.temp_warn_c:
                lines.append((WARN, f"Температура SoC {temp:.1f}°C — возможен "
                                    "троттлинг, нужен радиатор/обдув"))
            else:
                lines.append((OK, f"Температура SoC {temp:.1f}°C"))
        except OSError:
            lines.append((WARN, "Не удалось прочитать температуру SoC"))
        return lines

    def odom_distance_report(self):
        if self.filt_start is None or self.filt_last is None:
            return [(FAIL, "Замер одометрии: нет данных /odometry/filtered")]
        d = math.hypot(self.filt_last[0] - self.filt_start[0],
                       self.filt_last[1] - self.filt_start[1])
        scale = self.distance_test / d if d > 0.01 else float("inf")
        return [(OK, f"Замер одометрии: по /odometry/filtered {d:.2f} м при "
                     f"заданных {self.distance_test:.1f} м. "
                     f"Поправка: odometry_scale *= {scale:.3f} "
                     f"(параметр в kolesa_control)")]


def _print_report(lines):
    n_fail = 0
    for level, text in lines:
        print(f"[{level}] {text}")
        if level == FAIL:
            n_fail += 1
    return n_fail


def _spin_until_enter(node):
    """Крутит узел, пока оператор не нажмёт Enter."""
    done = threading.Event()

    def _wait():
        try:
            input()
        except EOFError:
            pass
        done.set()

    th = threading.Thread(target=_wait, daemon=True)
    th.start()
    while not done.is_set() and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    node = PreflightCheck()
    fails = 0
    try:
        if node.distance_test > 0.0:
            print("\n=== ЗАМЕР ОДОМЕТРИИ ===")
            print(f"Проедьте по рулетке ровно {node.distance_test:.0f} м вперёд "
                  "(с пульта), затем нажмите Enter...")
            _spin_until_enter(node)
            fails += _print_report(node.telemetry_report())
            fails += _print_report(node.odom_distance_report())
        else:
            print(f"\n=== ПРЕДПОЛЁТНАЯ ПРОВЕРКА: сбор {node.collect_s:.0f} с ===")
            end = time.monotonic() + node.collect_s
            while time.monotonic() < end and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)

            if not node.quick and node.filt_n > 0:
                print("Поставьте робота «носом» на ВОСТОК (по компасу). Enter...")
                _spin_until_enter(node)
                yaw_e = node.filt_yaw
                print("Поверните робота на 90° ПРОТИВ часовой стрелки "
                      "(нос на север). Enter...")
                _spin_until_enter(node)
                yaw_n = node.filt_yaw
                if yaw_e is not None and yaw_n is not None:
                    d = math.degrees(math.atan2(math.sin(yaw_n - yaw_e),
                                                math.cos(yaw_n - yaw_e)))
                    if 60.0 < d < 120.0:
                        fails += _print_report([(OK, f"Курс: поворот на 90° "
                                                     f"против часовой дал "
                                                     f"{d:+.0f}° — знак верный "
                                                     "(REP-103)")])
                    else:
                        fails += _print_report([(FAIL, f"Курс: ожидалось ≈ +90°, "
                                                       f"получилось {d:+.0f}° — "
                                                       "проверьте imu0_yaw_offset "
                                                       "и знак")])
                print(f"  yaw при «носе на восток»: "
                      f"{math.degrees(yaw_e or 0.0):+.1f}° (должно быть ≈0).")
                print("  Если не ≈0 — поправьте imu0_yaw_offset в "
                      "ekf_localization.yaml (и imu_yaw_offset_deg в "
                      "start.launch.py).")
                print("  Для проверки navsat_transform: проедьте 10 м строго "
                      "на север — путь в RViz должен идти вдоль +Y.")

            fails += _print_report(node.telemetry_report())
            fails += _print_report(node.waypoints_report())
            fails += _print_report(node.system_report())

        print()
        if fails:
            print(f"ИТОГ: {fails} ошибок(и) — ЗАПУСК МАРШРУТА НЕ РЕКОМЕНДУЕТСЯ")
        else:
            print("ИТОГ: готово к движению по маршруту")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(1 if fails else 0)


if __name__ == "__main__":
    main()
