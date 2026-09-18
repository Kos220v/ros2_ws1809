# -*- coding: utf-8 -*-
"""gyro_drift_check — измеритель дрейфа курса в покое.

Зачем: курс робота по ТЗ берётся только из кватерниона STM32 (/imu/data),
поэтому «плывущий угол» лечится калибровкой платы, а не ROS-стека. Узел
записывает /imu/data заданное время (робот должен стоять неподвижно!),
считает скорость уплывания курса и подсказывает нужную калибровку
(гироскоп или магнитометр). Чистая логика — в drift_check.py (покрыта
тестами).

Запуск (робот неподвижен, двигатель выключен):

    ros2 run imu_stm32_bridge gyro_drift_check --ros-args \
        -p duration_s:=60.0

Код возврата: 0 — дрейф в норме, 1 — требует внимания/измерение
недействительно. Типовые вердикты и что делать — docs/CALIBRATION.md.

Параметры:
    imu_topic          (по умолч. /imu/data)
    duration_s         длительность замера, с (60)
    warn_dpm           порог «дрейф заметен», °/мин (1.0)
    fail_dpm           порог «ехать нельзя», °/мин (10.0)
    motion_thresh_dps  отклонение угловой скорости от медианы канала,
                       считающееся движением, °/с (3.0)
    calib_service      сервис gyro_calib моста, проверяется доступность
                       (по умолч. /imu/imu_stm32_bridge/gyro_calib)
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from .drift_check import analyze

_VERDICT_TITLE = {
    "OK": "ИТОГ: дрейф в норме",
    "MOVED": "ИТОГ: измерение недействительно (робот двигался)",
    "GYRO_BIAS": "ИТОГ: смещение нуля гироскопа — нужна gyro_calib",
    "MAG": "ИТОГ: магнитный якорь слаб — нужна mag_calib / поиск помех",
}


class GyroDriftCheck(Node):

    def __init__(self):
        super().__init__("gyro_drift_check")
        p = self.declare_parameter
        p("imu_topic", "/imu/data")
        p("duration_s", 60.0)
        p("warn_dpm", 1.0)
        p("fail_dpm", 10.0)
        p("motion_thresh_dps", 3.0)
        p("calib_service", "/imu/imu_stm32_bridge/gyro_calib")

        g = self.get_parameter
        self.duration = float(g("duration_s").value)
        self.warn_dpm = float(g("warn_dpm").value)
        self.fail_dpm = float(g("fail_dpm").value)
        self.motion_thresh = float(g("motion_thresh_dps").value)
        # кандидаты имени сервиса калибровки: заданный параметром +
        # типовые варианты (мост без namespace, корень и т.п.)
        primary = self.calib_service = str(g("calib_service").value)
        candidates = [primary]
        for alt in ("/imu/imu_stm32_bridge/gyro_calib",
                    "/imu_stm32_bridge/gyro_calib",
                    "/gyro_calib"):
            if alt not in candidates:
                candidates.append(alt)
        self.calib_clis = [
            (name, self.create_client(Trigger, name)) for name in candidates
        ]

        self.samples = []
        self.t_first = None
        self.finished = False

        self.create_subscription(Imu, str(g("imu_topic").value),
                                 self._on_imu, qos_profile_sensor_data)
        self.get_logger().info(
            f"ЗАМЕР ДРЕЙФА: {self.duration:.0f} с. НЕ ТРОГАЙТЕ робота и не "
            "перекладывайте плату. Первые кадры: "
            + (str(g("imu_topic").value)))

    def _on_imu(self, msg: Imu):
        if self.finished:
            return
        q = msg.orientation
        n2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if not 0.5 < n2 < 1.5:
            return  # кватернион не валиден — кадр пропускаем
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = time.monotonic()
        self.samples.append((t, yaw,
                             msg.angular_velocity.x,
                             msg.angular_velocity.y,
                             msg.angular_velocity.z))
        if self.t_first is None:
            self.t_first = t
        if t - self.t_first >= self.duration:
            self.finished = True

    def report(self) -> int:
        """Напечатать отчёт. Возвращает код выхода (0 — норма)."""
        try:
            res = analyze(self.samples,
                          warn_dpm=self.warn_dpm,
                          fail_dpm=self.fail_dpm,
                          motion_thresh_dps=self.motion_thresh)
        except ValueError as e:
            self.get_logger().error(
                f"Замер не состоялся: {e}. Жив ли /imu/data "
                "(imu_stm32_bridge запущен)?")
            return 1

        log = self.get_logger().info
        log(f"длительность {res.duration_s:.0f} с, "
            f"курс {res.yaw_start_deg:+.1f}° -> {res.yaw_end_deg:+.1f}°")
        log(f"скорость уплывания курса: {res.drift_dpm:+.2f} °/мин "
            f"(пороги: warn {self.warn_dpm}, fail {self.fail_dpm})")
        log(f"средние wz={res.wz_mean_dps:+.3f} °/с "
            f"(сигма {res.wz_sigma_dps:.3f}), "
            f"wx={res.wx_mean_dps:+.3f}, wy={res.wy_mean_dps:+.3f}, "
            f"пик |w|={res.max_w_dps:.1f} °/с")
        log(f"смещения нуля (медианы): wx={res.wx_med_dps:+.3f}, "
            f"wy={res.wy_med_dps:+.3f}, wz={res.wz_med_dps:+.3f} °/с; "
            f"макс. отклонение (рывки) {res.max_dev_dps:.2f} °/с")
        ready = [(n, c) for n, c in self.calib_clis if c.service_is_ready()]
        if ready:
            for n, _ in ready:
                log(f"сервис gyro_calib: НАЙДЕН {n}")
            if len(ready) == 1 and ready[0][0] != self.calib_service:
                log(f"вызывайте: ros2 service call {ready[0][0]} "
                    "std_srvs/srv/Trigger")
        else:
            log(f"ВНИМАНИЕ: сервис gyro_calib НЕ найден ни по одному имени "
                f"({', '.join(n for n, _ in self.calib_clis)}). Диагностика: "
                "ros2 node list | grep imu; ros2 service list | grep calib; "
                "ros2 daemon stop && ros2 daemon start; пересборка "
                "imu_stm32_bridge + перезапуск start.launch.py. Если узел "
                "моста виден, а сервисов нет — запущена старая сборка моста "
                "(см. раздел 'waiting for service' в docs/CALIBRATION.md)")
        log(_VERDICT_TITLE.get(res.verdict, res.verdict))
        log("Что делать: " + res.recommendation)
        return 0 if res.verdict == "OK" else 1


def main(args=None):
    rclpy.init(args=args)
    node = GyroDriftCheck()
    code = 1
    try:
        deadline = time.monotonic() + node.duration + 15.0
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
            if not node.samples and time.monotonic() > deadline - node.duration:
                node.get_logger().error(
                    "/imu/data молчит — запущен ли imu_stm32_bridge?")
                break
            if time.monotonic() > deadline:
                break
        code = node.report()
    except KeyboardInterrupt:
        node.get_logger().warn("прервано — замер не завершён")
        code = node.report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
