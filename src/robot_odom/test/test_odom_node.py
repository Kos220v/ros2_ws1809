# -*- coding: utf-8 -*-

"""
Сквозной тест УЗЛА robot_odom на заглушках rclpy (см. ros_stubs/msg_stubs.py).

Исполняется настоящий robot_odom/odom_node.py: его колбэки _on_imu/_on_vesc,
watchdog и публикация /odom. Проверяются требования ТЗ:
  * путь интегрируется из twist.linear.x сообщения /odom/vesc;
  * курс берётся из кватерниона /imu/data и НЕ интегрируется;
  * twist.angular.z из /odom/vesc игнорируется полностью.

Если в системе установлен настоящий rclpy (на роботе), заглушки не
используются — тесты идут против реального rclpy.
"""

import importlib
import math
import os
import sys

import pytest

TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_ROOT = os.path.normpath(os.path.join(TEST_DIR, "..", ".."))

try:                                    # на роботе есть настоящий rclpy
    import rclpy  # noqa: F401
    assert rclpy is not None
    USING_STUBS = False
except ImportError:                     # в CI/на машине разработки — заглушки
    sys.path.insert(0, os.path.join(TEST_DIR, "ros_stubs"))
    USING_STUBS = True

sys.path.insert(0, PKG_ROOT)

msg_stubs = None
if USING_STUBS:
    msg_stubs = importlib.import_module("msg_stubs")

odom_node_mod = importlib.import_module("robot_odom.odom_node")
RobotOdom = odom_node_mod.RobotOdom


# ------------------------------------------------------------------- хелперы
def make_node(**overrides):
    """
    Создаёт узел. На заглушках параметры подставляются в declare_parameter
    до того, как __init__ их прочитает; с настоящим rclpy overrides не
    используются (тесты не зависят от них).
    """
    if USING_STUBS:
        return RobotOdom(param_overrides=overrides)
    return RobotOdom()


def imu_msg(yaw_rad, wz=0.0):
    Imu = msg_stubs.Imu if USING_STUBS else _real_imu()
    m = Imu()
    m.orientation.z = math.sin(yaw_rad / 2.0)
    m.orientation.w = math.cos(yaw_rad / 2.0)
    m.angular_velocity.z = wz
    return m


def vesc_msg(vx, wz=0.0, valid=True):
    Odometry = msg_stubs.Odometry if USING_STUBS else _real_odom()
    m = Odometry()
    m.twist.twist.linear.x = vx
    m.twist.twist.angular.z = wz
    cov = [0.0] * 36
    cov[0] = 0.01 if valid else 1.0e6
    m.twist.covariance = cov
    return m


def _real_imu():
    from sensor_msgs.msg import Imu
    return Imu


def _real_odom():
    from nav_msgs.msg import Odometry
    return Odometry


def set_time(node, t_sec):
    node._clock.nanoseconds = int(t_sec * 1e9)


def yaw_of(msg):
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def last_odom(node):
    return node.pubs()["/odom"].published[-1]


# --------------------------------------------------------------------- тесты
def test_straight_line_uses_vesc_linear_speed():
    node = make_node()
    set_time(node, 0.0)
    node.feed("/imu/data", imu_msg(0.0))           # курс на восток
    # 21 кадр по 0.05 с: первый задаёт точку отсчёта, далее 20 шагов = 1 с
    for i in range(21):
        set_time(node, i * 0.05)
        node.feed("/odom/vesc", vesc_msg(1.0))
    msg = last_odom(node)
    assert msg.pose.pose.position.x == pytest.approx(1.0, abs=1e-6)
    assert msg.pose.pose.position.y == pytest.approx(0.0, abs=1e-9)
    assert msg.twist.twist.linear.x == pytest.approx(1.0)


def test_yaw_follows_imu_quaternion():
    node = make_node()
    set_time(node, 0.0)
    node.feed("/imu/data", imu_msg(math.pi / 2.0))  # курс на север
    for i in range(21):
        set_time(node, i * 0.05)
        node.feed("/odom/vesc", vesc_msg(1.0))
    msg = last_odom(node)
    assert msg.pose.pose.position.x == pytest.approx(0.0, abs=1e-9)
    assert msg.pose.pose.position.y == pytest.approx(1.0, abs=1e-6)
    assert yaw_of(msg) == pytest.approx(math.pi / 2.0)


def test_wheel_angular_velocity_is_ignored():
    """
    ТЗ: данные о поворотах из kolesa_control не используются СОВСЕМ.
    Даже при огромном twist.angular.z в /odom/vesc робот едет прямо,
    а в /odom попадает угловая скорость гироскопа IMU.
    """
    node = make_node()
    set_time(node, 0.0)
    node.feed("/imu/data", imu_msg(0.0, wz=0.123))
    for i in range(21):
        set_time(node, i * 0.05)
        node.feed("/odom/vesc", vesc_msg(1.0, wz=9.9))
    msg = last_odom(node)
    assert msg.pose.pose.position.x == pytest.approx(1.0, abs=1e-6)
    assert msg.pose.pose.position.y == pytest.approx(0.0, abs=1e-9)
    assert msg.twist.twist.angular.z == pytest.approx(0.123)


def test_rotation_in_place_does_not_move_robot():
    node = make_node()
    set_time(node, 0.0)
    node.feed("/odom/vesc", vesc_msg(0.0))
    for i in range(10):
        set_time(node, (i + 1) * 0.05)
        node.feed("/imu/data", imu_msg(math.radians(9.0 * (i + 1))))
        node.feed("/odom/vesc", vesc_msg(0.0))
    msg = last_odom(node)
    assert (msg.pose.pose.position.x, msg.pose.pose.position.y) == (0.0, 0.0)
    assert yaw_of(msg) == pytest.approx(math.radians(90.0), abs=1e-9)


def test_invalid_vesc_covariance_stops_integration():
    node = make_node()
    set_time(node, 0.0)
    node.feed("/imu/data", imu_msg(0.0))
    for i in range(10):
        set_time(node, (i + 1) * 0.05)
        node.feed("/odom/vesc", vesc_msg(5.0, valid=False))  # бортов нет
    msg = last_odom(node)
    assert msg.pose.pose.position.x == 0.0
    assert msg.twist.twist.linear.x == 0.0
    assert msg.twist.covariance[0] == 1.0e6


def test_time_jump_is_clamped():
    node = make_node()
    set_time(node, 0.0)
    node.feed("/imu/data", imu_msg(0.0))
    node.feed("/odom/vesc", vesc_msg(1.0))
    set_time(node, 100.0)                       # узел молчал 100 с
    node.feed("/odom/vesc", vesc_msg(1.0))
    # max_dt = 0.25 с -> максимум 0.25 м, а не 100 м
    assert last_odom(node).pose.pose.position.x == pytest.approx(0.25, abs=1e-9)


def test_watchdog_zeroes_speed_when_vesc_goes_stale():
    node = make_node()
    set_time(node, 0.0)
    node.feed("/imu/data", imu_msg(0.0))
    node.feed("/odom/vesc", vesc_msg(1.0))
    # имитируем 10 секунд тишины: последний кадр был «давно»
    node._last_vesc_msg_t = node._now_s() - 10.0
    for cb in node.timers:
        cb()                                    # watchdog
    assert node.vx == 0.0, "watchdog обязан обнулить скорость при пропаже VESC"
    set_time(node, 0.05)
    node.feed("/odom/vesc", vesc_msg(1.0))
    # интегрируется только ограниченный шаг, «старая» скорость не тянется
    assert last_odom(node).pose.pose.position.x == pytest.approx(0.05, abs=1e-9)


def test_publish_tf_disabled_by_default():
    node = make_node()
    assert "/tf" not in node.pubs()


def test_relative_yaw_mode_zeroes_at_start():
    node = make_node(yaw_mode="relative", yaw_offset_deg=0.0)
    if not USING_STUBS:
        pytest.skip("параметры узла подставляются только на заглушках")
    set_time(node, 0.0)
    node.feed("/imu/data", imu_msg(math.radians(30.0)))
    assert node.yaw == pytest.approx(0.0, abs=1e-9)
    node.feed("/imu/data", imu_msg(math.radians(120.0)))
    assert node.yaw == pytest.approx(math.radians(90.0), abs=1e-9)
