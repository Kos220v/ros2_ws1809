# -*- coding: utf-8 -*-
"""Смоук-тест узла GpsHeading на заглушках rclpy (полное инстанцирование).

Страховка после инцидента с gps_mission (потерянный атрибут падал только
на роботе): узел собирается целиком, колбэк /gps/fix прогоняется по
реальному маршруту движения, проверяется опубликованное сообщение.
Заглушки ставятся в sys.modules только на время импорта узла и
восстанавливаются — порядок сбора тестов не важен.
"""
import math
import os
import sys
import types

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))


class _Param:
    def __init__(self, value):
        self.value = value


class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("I", str(m)))

    def warn(self, m):
        self.lines.append(("W", str(m)))

    warning = warn

    def error(self, m):
        self.lines.append(("E", str(m)))


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _StubNode:
    overrides = {}

    def __init__(self, name, **kw):
        self.name = name
        self._defaults = {}
        self.subs = []
        self.pubs = {}
        self._logger = _Logger()

    def declare_parameter(self, name, value):
        self._defaults[name] = value

    def get_parameter(self, name):
        return _Param(_StubNode.overrides.get(name, self._defaults[name]))

    def create_subscription(self, typ, topic, cb, qos):
        self.subs.append((typ, topic, cb))
        return (typ, topic)

    def create_publisher(self, typ, topic, qos):
        p = _Pub()
        self.pubs[topic] = p
        return p

    def get_logger(self):
        return self._logger

    def get_clock(self):
        class _St:
            def to_msg(self):
                return types.SimpleNamespace(sec=0, nanosec=0)

        class _Cl:
            def now(self):
                return _St()

        return _Cl()


def _install_stubs():
    saved = {}

    def mod(name):
        saved[name] = sys.modules.get(name)
        m = types.ModuleType(name)
        sys.modules[name] = m
        return m

    rclpy = mod("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.ok = lambda: True
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda node: None

    node_mod = mod("rclpy.node")
    node_mod.Node = _StubNode

    qos_mod = mod("rclpy.qos")
    qos_mod.qos_profile_sensor_data = object()

    geo = mod("geometry_msgs.msg")

    class PoseWithCovarianceStamped:
        def __init__(self):
            self.header = types.SimpleNamespace(
                frame_id="", stamp=types.SimpleNamespace(sec=0, nanosec=0))
            self.pose = types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=types.SimpleNamespace(
                        x=0.0, y=0.0, z=0.0, w=1.0)),
                covariance=[])

    geo.PoseWithCovarianceStamped = PoseWithCovarianceStamped

    sens = mod("sensor_msgs.msg")

    class NavSatFix:
        def __init__(self):
            self.status = types.SimpleNamespace(status=0)
            self.latitude = 0.0
            self.longitude = 0.0

    sens.NavSatFix = NavSatFix
    return saved


_SAVED = _install_stubs()

from gps_navigator.gps_heading_node import GpsHeading  # noqa: E402

NavSatFix = sys.modules["sensor_msgs.msg"].NavSatFix

for _name, _orig in _SAVED.items():
    if _orig is None:
        sys.modules.pop(_name, None)
    else:
        sys.modules[_name] = _orig
del _SAVED

LAT0 = 59.4
LON0 = 24.7


@pytest.fixture(autouse=True)
def _reset():
    _StubNode.overrides = {}
    yield
    _StubNode.overrides = {}


class _Fix:
    def __init__(self, lat, lon, status=0, sec=0):
        self.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=sec, nanosec=0))
        self.status = types.SimpleNamespace(status=status)
        self.latitude = lat
        self.longitude = lon


class TestNodeConstructs:
    def test_defaults(self):
        node = GpsHeading()
        assert node.frame_id == "map"
        assert node.published == 0
        assert node.first_yaw is None
        topics = [s[1] for s in node.subs]
        assert "/gps/fix" in topics
        assert "/gps/heading" in node.pubs

    def test_param_overrides(self):
        _StubNode.overrides["gps_topic"] = "/uav/fix"
        _StubNode.overrides["frame_id"] = "world"
        node = GpsHeading()
        assert node.frame_id == "world"
        assert "/uav/fix" in [s[1] for s in node.subs]


class TestFixProcessing:
    def _drive_north(self, node, n=6, speed=1.0, t0=0):
        """n фиксов по 1 с (штампы t0..t0+n-1), движение строго на север."""
        out = None
        for i in range(n):
            out = node._on_fix(
                _Fix(LAT0 + i * speed / 111319.49, LON0, sec=t0 + i))
        return out

    def test_publishes_heading_while_moving(self):
        node = GpsHeading()
        self._drive_north(node)
        assert node.published >= 1
        (m,) = node.pubs["/gps/heading"].msgs[-1:]
        assert m.header.frame_id == "map"
        yaw = math.atan2(2.0 * (m.pose.pose.orientation.w * m.pose.pose.orientation.z), 1.0 - 2.0 * m.pose.pose.orientation.z ** 2)
        assert abs(yaw - math.pi / 2) < 1e-6
        # ковариации: yaw маленькая, позиция/углы — огромные
        assert m.pose.covariance[35] <= 1.0
        assert m.pose.covariance[0] > 1e8
        assert m.pose.covariance[28] > 1e8

    def test_negative_fix_status_ignored(self):
        node = GpsHeading()
        for i in range(8):
            node._on_fix(_Fix(LAT0 + i / 111319.49, LON0, status=-1, sec=i))
        assert node.published == 0
        assert node.pubs["/gps/heading"].msgs == []

    def test_stationary_no_publication(self):
        node = GpsHeading()
        for i in range(10):
            node._on_fix(_Fix(LAT0, LON0, sec=i))
        assert node.published == 0

    def test_gps_jump_not_published(self):
        node = GpsHeading()
        node._on_fix(_Fix(LAT0, LON0, sec=1))
        # телепорт на 500 м за 1 с — защита должна отбросить
        node._on_fix(_Fix(LAT0 + 500.0 / 111319.49, LON0, sec=2))
        node._on_fix(_Fix(LAT0 + 503.0 / 111319.49, LON0, sec=3))
        node._on_fix(_Fix(LAT0 + 506.0 / 111319.49, LON0, sec=4))
        assert node.published == 0
        assert node.pubs["/gps/heading"].msgs == []

    def test_first_yaw_logged_once(self):
        node = GpsHeading()
        self._drive_north(node, t0=0)
        self._drive_north(node, t0=100)
        first_logs = [m for lvl, m in node._logger.lines
                      if lvl == "I" and "первая оценка курса" in m]
        assert len(first_logs) == 1
