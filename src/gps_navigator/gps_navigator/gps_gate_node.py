# -*- coding: utf-8 -*-
"""gps_fix_gate — фильтр GPS-фиксов перед navsat_transform.

/gps/fix -> проверка (статус, ошибка по ковариации ~HDOP, прыжки)
-> /gps/fix/filtered. navsat_transform и gps_heading питаются от
фильтрованного топика (remap в navigation.launch.py), чтобы один
мультитрейновый прыжок не телепортировал TF map->odom и костмапы
(симптом: «Sensor origin ... is out of map bounds ... cannot
raytrace»). Логика — gps_fix_gate.py (покрыта тестами).

Негативные фиксы (status < 0) не пропускаются. Оценка ошибки по
ковариации: sqrt(cov[0]) м; если драйвер не заполняет ковариацию
(position_covariance_type == NO_VALID = 0), фильтр по HDOP
недоступен — работает только отсечка прыжков (об этом узел пишет
один раз). Статистика отбора — в лог раз в stats_period_s.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

from .gps_fix_gate import FixGate


class GpsFixGate(Node):

    def __init__(self):
        super().__init__("gps_fix_gate")
        p = self.declare_parameter
        p("gps_topic", "/gps/fix")
        p("output_topic", "/gps/fix/filtered")
        p("max_h_error_m", 20.0)
        p("max_jump_mps", 15.0)
        p("stats_period_s", 10.0)

        g = self.get_parameter
        self.max_h_error_m = float(g("max_h_error_m").value)
        self.max_jump_mps = float(g("max_jump_mps").value)
        self.stats_period = float(g("stats_period_s").value)

        self.gate = FixGate(max_h_error_m=self.max_h_error_m,
                            max_jump_mps=self.max_jump_mps)
        self.pub = self.create_publisher(
            NavSatFix, str(g("output_topic").value), 10)
        self.create_subscription(
            NavSatFix, str(g("gps_topic").value), self._on_fix,
            qos_profile_sensor_data)

        self._cov_warned = False
        self._last_stats_t = time.monotonic()
        self.get_logger().info(
            f"gps_fix_gate: {g('gps_topic').value} -> "
            f"{g('output_topic').value} "
            f"(ошибка <= {self.max_h_error_m} м, прыжок <= "
            f"{self.max_jump_mps} м/с)")

    def _on_fix(self, msg: NavSatFix):
        if msg.status.status < 0:
            return
        stamp = msg.header.stamp
        t = (stamp.sec + stamp.nanosec * 1e-9) if (stamp.sec or stamp.nanosec) \
            else time.monotonic()

        h_error = None
        if msg.position_covariance_type != 0:      # NO_VALID
            h_error = math.sqrt(abs(msg.position_covariance[0]))
        elif not self._cov_warned:
            self._cov_warned = True
            self.get_logger().info(
                "драйвер не заполняет ковариацию позиции — гейт по HDOP "
                "недоступен, работает только отсечка прыжков")

        if self.gate.accept(t, msg.latitude, msg.longitude, h_error):
            self.pub.publish(msg)

        now = time.monotonic()
        if now - self._last_stats_t >= self.stats_period:
            self._last_stats_t = now
            st = self.gate.stats
            if st.rejected_total:
                self.get_logger().info(
                    f"GPS-гейт: принято {st.accepted}, отброшено "
                    f"{st.rejected_total} (ковариация {st.rejected_cov}, "
                    f"прыжки {st.rejected_jump}, битые {st.rejected_bad}), "
                    f"ресинхронизаций {st.resyncs}; "
                    f"последняя причина: {st.last_reject_reason}")
            if self.gate._jump_streak >= 10 and st.resyncs == 0:
                self.get_logger().warning(
                    f"GPS нестабилен: {self.gate._jump_streak} отброшенных "
                    "прыжков подряд без согласованности - координаты "
                    "приёмника «бегут». Навигация по счислению (vx+IMU), "
                    "map не телепортируется. Проверьте: одна ли публикация "
                    "в /gps/fix (ros2 topic info /gps/fix -v), не «бежит» "
                    "ли lat/lon (ros2 topic echo /gps/fix --field "
                    "position), антенну и её кабель")


def main(args=None):
    rclpy.init(args=args)
    node = GpsFixGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
