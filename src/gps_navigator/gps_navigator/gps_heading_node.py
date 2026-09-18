# -*- coding: utf-8 -*-
"""gps_heading — курс робота по фактическому перемещению GPS.

Подписка /gps/fix -> скользящее окно (~3 с) -> пока робот едет и сместился
минимум на min_dist_m, направление перемещения = курс (ENU). Публикуется
geometry_msgs/PoseWithCovarianceStamped (frame_id=map, заполнен ТОЛЬКО yaw,
позиция — нули с гигантскими ковариациями) в /gps/heading.

Назначение: ekf_map берёт это как pose0 (только yaw) и не даёт дрейфу
кватерниона IMU накапливаться в глобальной системе координат. На стоянке
оценки нет — дрейф на стоянке на позицию не влияет. Логика — в
gps_heading_logic.py (покрыта тестами), узел только перекладывает сообщения.

CPU: единицы операций на каждый GPS-фикс (1-10 Гц) — пренебрежимо мало.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import NavSatFix

from .gps_heading_logic import (
    HeadingEstimator,
    yaw_covariance,
    yaw_to_quaternion,
)


class GpsHeading(Node):

    def __init__(self):
        super().__init__("gps_heading")
        p = self.declare_parameter
        p("gps_topic", "/gps/fix")
        p("output_topic", "/gps/heading")
        p("frame_id", "map")
        p("window_s", 3.0)
        p("min_dist_m", 2.0)
        p("max_speed_mps", 30.0)
        p("yaw_var_min", 0.05)
        p("yaw_var_max", 1.0)
        p("yaw_var_ref_speed", 1.0)

        g = self.get_parameter
        self.frame_id = str(g("frame_id").value)
        self.var_min = float(g("yaw_var_min").value)
        self.var_max = float(g("yaw_var_max").value)
        self.var_ref_speed = float(g("yaw_var_ref_speed").value)

        self.est = HeadingEstimator(
            window_s=float(g("window_s").value),
            min_dist_m=float(g("min_dist_m").value),
            max_speed_mps=float(g("max_speed_mps").value),
        )

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, str(g("output_topic").value), 10)
        self.create_subscription(
            NavSatFix, str(g("gps_topic").value), self._on_fix,
            qos_profile_sensor_data)

        self.published = 0
        self.first_yaw = None
        self.get_logger().info(
            f"gps_heading: {g('gps_topic').value} -> {g('output_topic').value} "
            f"(окно {g('window_s').value} с, минимум "
            f"{g('min_dist_m').value} м перемещения)")

    def _on_fix(self, msg: NavSatFix):
        if msg.status.status < 0:
            return
        # время берём из штампа фикса (единые часы драйвера); при нулевом
        # штампе — монотонные часы приёма
        stamp = msg.header.stamp
        t = (stamp.sec + stamp.nanosec * 1e-9) if (stamp.sec or stamp.nanosec) \
            else time.monotonic()
        est = self.est.add(t, msg.latitude, msg.longitude)
        if est is None:
            return

        m = PoseWithCovarianceStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        # Позиция не используется (в pose0_config включён только yaw), но
        # обнуляем её и ставим огромные ковариации — на всякий случай.
        m.pose.pose.position.x = 0.0
        m.pose.pose.position.y = 0.0
        m.pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(est.yaw_rad)
        m.pose.pose.orientation.x = qx
        m.pose.pose.orientation.y = qy
        m.pose.pose.orientation.z = qz
        m.pose.pose.orientation.w = qw
        var = yaw_covariance(est.speed_mps, self.var_min, self.var_max,
                             self.var_ref_speed)
        cov = [1e9] * 36
        cov[0] = cov[7] = cov[14] = cov[21] = cov[28] = 1e9
        cov[35] = var
        m.pose.covariance = list(cov)
        self.pub.publish(m)

        self.published += 1
        if self.first_yaw is None:
            self.first_yaw = est.yaw_rad
            self.get_logger().info(
                f"первая оценка курса: {math.degrees(est.yaw_rad):.1f}° "
                f"(ENU), {est.speed_mps:.2f} м/с, окно {est.span_s:.1f} с")


def main(args=None):
    rclpy.init(args=args)
    node = GpsHeading()
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
