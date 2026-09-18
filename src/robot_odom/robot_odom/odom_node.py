# -*- coding: utf-8 -*-

"""
robot_odom — объединение одометрии привода и курса IMU в /odom.

ИСТОЧНИКИ (строго по ТЗ):
    /odom/vesc (nav_msgs/Odometry)  — ТОЛЬКО twist.twist.linear.x
                                      (линейная скорость центра робота
                                      по тахометрам VESC, kolesa_control).
                                      Разность бортов / угловая скорость
                                      колёс НЕ используются НИГДЕ.
    /imu/data  (sensor_msgs/Imu)    — курс берётся из ГОТОВОГО КВАТЕРНИОНА
                                      ориентации (ENU-фьюжн на STM32).
                                      Гироскоп не интегрируется.

ПУБЛИКАЦИЯ:
    /odom (nav_msgs/Odometry) — pose (x, y интегрируются из скорости VESC,
        ориентация — кватернион из IMU) + twist (vx из VESC, wz из гироскопа
        IMU — только для информации потребителей).
    TF odom -> base_link — только если publish_tf:=true (БЕЗ
        robot_localization / slam_toolbox, иначе будет два издателя TF).

РЕЖИМЫ yaw_mode:
    absolute — курс в ENU: 0 = восток, +pi/2 = север. Нужен для GPS-маршрута
               (gps_navigator сравнивает его с азимутом на точку).
    relative — yaw = 0 в момент старта (для стенда/помещения).

CPU: вся работа выполняется в колбэках (20 Гц телеметрия VESC, 25-50 Гц IMU),
отдельного тяжёлого цикла нет; watchdog — редкий таймер 2 Гц.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TransformStamped, TwistWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

from .odom_math import (
    UNMEASURED_COV,
    integrate_pose,
    pose_covariance,
    quaternion_from_yaw,
    quaternion_to_yaw,
    twist_covariance,
    wrap_angle,
)


class RobotOdom(Node):
    def __init__(self, param_overrides=None):
        """
        param_overrides используется только тестами без установленного rclpy
        (см. test/ros_stubs): позволяет подставить параметры до того, как
        __init__ их прочитает. В бою (rclpy) параметры приходят из
        config/odom_params.yaml и launch-файла.
        """
        if param_overrides:
            # только тесты на заглушках rclpy; боевой путь не меняется
            super().__init__("robot_odom", param_overrides=param_overrides)
        else:
            super().__init__("robot_odom")

        # ---------------------------------------------------------- параметры
        p = self.declare_parameter
        p("vesc_odom_topic", "/odom/vesc")
        p("imu_topic", "/imu/data")
        p("odom_topic", "/odom")
        p("odom_frame", "odom")
        p("base_frame", "base_link")
        p("publish_tf", False)
        p("yaw_mode", "absolute")          # absolute | relative
        p("yaw_offset_deg", 0.0)           # поправка монтажа IMU, град
        p("max_dt", 0.25)                  # защита от скачков времени, с
        p("vesc_timeout", 0.5)             # /odom/vesc устарел, с
        p("imu_timeout", 0.5)              # /imu/data устарел, с
        p("publish_rate_watchdog", 2.0)    # Гц проверки устаревания

        g = self.get_parameter
        self.odom_frame = str(g("odom_frame").value)
        self.base_frame = str(g("base_frame").value)
        self.publish_tf = bool(g("publish_tf").value)
        self.yaw_mode = str(g("yaw_mode").value).lower()
        if self.yaw_mode not in ("absolute", "relative"):
            raise ValueError(
                f"yaw_mode должен быть 'absolute' или 'relative', "
                f"получено '{self.yaw_mode}'"
            )
        self.yaw_offset = math.radians(float(g("yaw_offset_deg").value))
        self.max_dt = float(g("max_dt").value)
        self.vesc_timeout = float(g("vesc_timeout").value)
        self.imu_timeout = float(g("imu_timeout").value)

        # --------------------------------------------------------- состояние
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0                 # линейная скорость из /odom/vesc
        self.wz_imu = 0.0             # угловая скорость из гироскопа IMU (инфо)
        self.vesc_valid = False       # последний /odom/vesc пригоден (ковар. vx)
        self.imu_valid = False
        self._yaw0 = None             # yaw при старте (режим relative)
        self._last_odom_t = None      # монотонное время прошлого шага, с
        self._last_vesc_msg_t = None  # время последнего /odom/vesc
        self._last_imu_msg_t = None
        self._vesc_warned = False
        self._imu_warned = False

        self._pose_cov = pose_covariance()
        self._twist_cov_ok = twist_covariance(True)
        self._twist_cov_bad = twist_covariance(False)

        # ----------------------------------------------------------- топики
        self.pub_odom = self.create_publisher(
            Odometry, str(g("odom_topic").value), 10)
        self.pub_tf = None
        if self.publish_tf:
            self.pub_tf = self.create_publisher(TFMessage, "/tf", 10)

        # /odom/vesc публикуется kolesa_control с надёжным QoS (depth 20)
        self.create_subscription(
            Odometry, str(g("vesc_odom_topic").value), self._on_vesc, 10)
        # IMU — сенсорный BEST_EFFORT-поток
        self.create_subscription(
            Imu, str(g("imu_topic").value), self._on_imu, qos_profile_sensor_data)

        watchdog_hz = float(g("publish_rate_watchdog").value)
        self.create_timer(1.0 / watchdog_hz, self._watchdog)

        self.get_logger().info(
            "robot_odom: путь — /odom/vesc (только linear.x), курс — кватернион "
            f"/imu/data, yaw_mode={self.yaw_mode}, offset="
            f"{math.degrees(self.yaw_offset):+.1f}°, publish_tf={self.publish_tf}"
        )

    # ------------------------------------------------------------- колбэки
    def _on_imu(self, msg: Imu):
        """Курс — из ГОТОВОГО кватерниона. Интеграции гироскопа нет."""
        yaw_raw = quaternion_to_yaw(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w,
        )
        self.wz_imu = float(msg.angular_velocity.z)

        if self.yaw_mode == "relative" and self._yaw0 is None:
            self._yaw0 = yaw_raw

        if self.yaw_mode == "relative" and self._yaw0 is not None:
            self.yaw = wrap_angle(yaw_raw - self._yaw0 + self.yaw_offset)
        else:
            self.yaw = wrap_angle(yaw_raw + self.yaw_offset)

        self.imu_valid = True
        self._last_imu_msg_t = self._now_s()
        if self._imu_warned:
            self._imu_warned = False
            self.get_logger().info("IMU снова в сети — курс обновляется")

    def _on_vesc(self, msg: Odometry):
        """
        Из /odom/vesc берётся ТОЛЬКО twist.twist.linear.x.

        twist.angular.z там намеренно не заполнен (ковариация 1e6) — и даже
        если бы был, он бы игнорировался: курс даёт IMU.
        """
        now = self._now_s()

        # Пригодность: kolesa_control ставит vx-ковариацию 1e6, если бортов нет.
        cov0 = float(msg.twist.covariance[0])
        self.vesc_valid = cov0 < UNMEASURED_COV
        self.vx = float(msg.twist.twist.linear.x) if self.vesc_valid else 0.0

        # Интегрирование позиции: путь (VESC) раскладывается по курсу (IMU).
        if self._last_odom_t is not None:
            dt = now - self._last_odom_t
            if dt < 0.0:
                dt = 0.0
            elif dt > self.max_dt:
                # Пропуск сообщений/скачок часов — ограничиваем шаг,
                # чтобы не «телепортировать» робота.
                dt = self.max_dt
            v = self.vx if self.vesc_valid and self.imu_valid else 0.0
            self.x, self.y = integrate_pose(self.x, self.y, self.yaw, v, dt)
        self._last_odom_t = now

        self._last_vesc_msg_t = now
        if self._vesc_warned:
            self._vesc_warned = False
            self.get_logger().info("/odom/vesc снова в сети — путь считается")

        self._publish(now)

    def _watchdog(self):
        """Редкий таймер: следит за устареванием источников, стоп при пропаже."""
        now = self._now_s()
        vesc_stale = (
            self._last_vesc_msg_t is None
            or now - self._last_vesc_msg_t > self.vesc_timeout
        )
        imu_stale = (
            self._last_imu_msg_t is None
            or now - self._last_imu_msg_t > self.imu_timeout
        )

        if vesc_stale and not self._vesc_warned:
            self._vesc_warned = True
            self.vx = 0.0
            self.get_logger().warning(
                f"/odom/vesc не обновляется > {self.vesc_timeout:.1f} с — "
                "путь не интегрируется (проверьте kolesa_control / VESC)")
        if imu_stale and not self._imu_warned:
            self._imu_warned = True
            self.imu_valid = False
            self.get_logger().warning(
                f"/imu/data не обновляется > {self.imu_timeout:.1f} с — "
                "курс недостоверен (проверьте imu_stm32_bridge)")

    # ---------------------------------------------------------- публикация
    def _publish(self, now_s):
        msg = Odometry()
        stamp = self.get_clock().now().to_msg()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        qx, qy, qz, qw = quaternion_from_yaw(self.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance = self._pose_cov

        twist = TwistWithCovariance()
        twist.twist.linear.x = self.vx if self.vesc_valid else 0.0
        twist.twist.angular.z = self.wz_imu if self.imu_valid else 0.0
        twist.covariance = (
            self._twist_cov_ok if (self.vesc_valid and self.imu_valid)
            else self._twist_cov_bad
        )
        msg.twist = twist

        self.pub_odom.publish(msg)

        if self.pub_tf is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            tfm = TFMessage()
            tfm.transforms.append(t)
            self.pub_tf.publish(tfm)

    # ------------------------------------------------------------- утилиты
    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = RobotOdom()
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
