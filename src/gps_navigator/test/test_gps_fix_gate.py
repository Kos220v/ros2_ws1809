# -*- coding: utf-8 -*-
"""Тесты гейта GPS-фиксов: логика (gps_fix_gate.py) + смоук-тест узла."""
import math
import os
import sys
import types

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from gps_navigator.gps_fix_gate import FixGate


class TestFixGateLogic:
    def test_first_fix_accepted(self):
        g = FixGate()
        assert g.accept(0.0, 56.3, 43.9, 3.0) is True
        assert g.stats.accepted == 1

    def test_nan_rejected(self):
        g = FixGate()
        assert g.accept(0.0, float("nan"), 43.9, 3.0) is False
        assert g.accept(0.1, 56.3, float("inf"), 3.0) is False
        assert g.stats.rejected_bad == 2

    def test_high_covariance_rejected(self):
        g = FixGate(max_h_error_m=20.0)
        g.accept(0.0, 56.3, 43.9, 3.0)
        assert g.accept(0.2, 56.3, 43.9, 25.0) is False
        assert g.stats.rejected_cov == 1

    def test_unknown_covariance_skips_cov_check(self):
        g = FixGate()
        assert g.accept(0.0, 56.3, 43.9, None) is True
        # следующий фикс принят без ковариации, прыжка нет
        assert g.accept(1.0, 56.30001, 43.9, None) is True

    def test_nonfinite_cov_treated_as_unknown(self):
        g = FixGate()
        assert g.accept(0.0, 56.3, 43.9, float("nan")) is True

    def test_jump_rejected_then_resync(self):
        g = FixGate(max_jump_mps=15.0, jump_resync_after=3)
        g.accept(0.0, 56.3, 43.9, 3.0)
        # 500 м за 1 с = 500 м/с: одиночный прыжок отброшен
        assert g.accept(1.0, 56.3 + 500.0 / 111319.49, 43.9, 3.0) is False
        assert g.stats.rejected_jump == 1
        # следующая точка от той же опоры — тоже «прыжок» (гейт прав:
        # телепорт не становится нормой)
        assert g.accept(2.0, 56.3 + 501.0 / 111319.49, 43.9, 3.0) is False
        # после jump_resync_after отбросов подряд фикс принимается как
        # новая опора (робота реально перенесли)
        assert g.accept(3.0, 56.3 + 502.0 / 111319.49, 43.9, 3.0) is True
        # и дальше обычное движение свободно
        assert g.accept(4.0, 56.3 + 503.0 / 111319.49, 43.9, 3.0) is True
        assert g.stats.rejected_jump == 2

    def test_racing_receiver_never_resyncs(self):
        # реальный случай с робота: приёмник «бежит» ~3 км/с — каждый
        # отброшенный фикс далеко от предыдущего отброшенного -> ресинхрон
        # запрещён, навигация остаётся на счислении
        g = FixGate(max_jump_mps=15.0, jump_resync_after=5)
        g.accept(0.0, 56.3, 43.9, 3.0)
        for i in range(1, 21):
            assert g.accept(float(i), 56.3 + i * 100.0 / 111319.49,
                            43.9, 3.0) is False
        assert g.stats.accepted == 1
        assert g.stats.resyncs == 0
        assert g.stats.rejected_jump == 20

    def test_racing_then_stable_resyncs(self):
        # мусор кончился, приёмник стоит на месте: взаимно согласованные
        # отброшенные фиксы дают ресинхрон
        g = FixGate(max_jump_mps=15.0, jump_resync_after=5)
        g.accept(0.0, 56.3, 43.9, 3.0)
        for i in range(1, 6):
            assert g.accept(float(i), 56.3 + i * 100.0 / 111319.49,
                            43.9, 3.0) is False
        # приёмник «остановился» на новом месте: счётчик прыжков уже
        # набран, первый же согласованный (неподвижный относительно
        # предыдущего отброшенного) фикс даёт ресинхрон
        assert g.accept(6.0, 56.3 + 498.0 / 111319.49, 43.9, 3.0) is True
        assert g.stats.resyncs == 1
        # и дальше обычное движение свободно
        assert g.accept(7.0, 56.3 + 498.5 / 111319.49, 43.9, 3.0) is True
        assert g.accept(8.0, 56.3 + 499.0 / 111319.49, 43.9, 3.0) is True

    def test_single_jump_then_normal_motion_no_resync(self):
        g = FixGate(max_jump_mps=15.0, jump_resync_after=5)
        g.accept(0.0, 56.3, 43.9, 3.0)
        assert g.accept(1.0, 56.3 + 500.0 / 111319.49, 43.9, 3.0) is False
        # после одиночного прыжка робот «вернулся» и едет нормально —
        # всё принимается без ресинхронизации
        assert g.accept(2.0, 56.3 + 2.0 / 111319.49, 43.9, 3.0) is True
        assert g.accept(3.0, 56.3 + 3.0 / 111319.49, 43.9, 3.0) is True
        assert g.stats.accepted == 3

    def test_time_rollback_resets_reference(self):
        g = FixGate()
        g.accept(100000.0, 56.3, 43.9, 3.0)
        # время скакнуло назад на другую эпоху — не считается прыжком
        assert g.accept(0.0, 56.3 + 400.0 / 111319.49, 43.9, 3.0) is True

    def test_equal_timestamps_not_jumps(self):
        g = FixGate()
        g.accept(5.0, 56.3, 43.9, 3.0)
        # dt == 0: деления нет, фикс принимается (координаты те же)
        assert g.accept(5.0, 56.3, 43.9, 3.0) is True

    def test_bad_thresholds(self):
        for kw in ({"max_h_error_m": 0.0}, {"max_jump_mps": -1.0}):
            try:
                FixGate(**kw)
                raise AssertionError("ожидался ValueError")
            except ValueError:
                pass

    def test_stats_summary(self):
        g = FixGate()
        g.accept(0.0, 56.3, 43.9, 3.0)
        g.accept(0.1, 56.3, 43.9, 99.0)
        g.accept(0.2, float("nan"), 43.9, 3.0)
        assert g.stats.rejected_total == 2
        assert "не число" in g.stats.last_reject_reason


# ---------------------------------------------------------------- смоук узла
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


def _install_stubs():
    import types
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

    sens = mod("sensor_msgs.msg")

    class NavSatFix:
        def __init__(self):
            self.header = types.SimpleNamespace(
                stamp=types.SimpleNamespace(sec=0, nanosec=0))
            self.status = types.SimpleNamespace(status=0)
            self.latitude = 0.0
            self.longitude = 0.0
            self.position_covariance = [0.0] * 9
            self.position_covariance_type = 0

    sens.NavSatFix = NavSatFix
    return saved


_SAVED = _install_stubs()

from gps_navigator.gps_gate_node import GpsFixGate  # noqa: E402

NavSatFix = sys.modules["sensor_msgs.msg"].NavSatFix

for _name, _orig in _SAVED.items():
    if _orig is None:
        sys.modules.pop(_name, None)
    else:
        sys.modules[_name] = _orig
del _SAVED

LAT0 = 56.3
LON0 = 43.9



import pytest  # noqa: E402


@pytest.fixture(autouse=True)
def _reset():
    _StubNode.overrides = {}
    yield
    _StubNode.overrides = {}


class _Fix:
    def __init__(self, lat, lon, sec, status=0, cov=9.0, cov_type=2):
        self.header = types.SimpleNamespace(
            stamp=types.SimpleNamespace(sec=sec, nanosec=0))
        self.status = types.SimpleNamespace(status=status)
        self.latitude = lat
        self.longitude = lon
        self.position_covariance = [cov * cov] + [0.0] * 8
        self.position_covariance_type = cov_type


class TestGateNode:
    def _node(self):
        return GpsFixGate()

    def test_constructs_and_topics(self):
        node = self._node()
        assert "/gps/fix" in [s[1] for s in node.subs]
        assert "/gps/fix/filtered" in node.pubs

    def test_good_fixes_pass_through(self):
        node = self._node()
        for i in range(4):
            node._on_fix(_Fix(LAT0 + i * 0.5 / 111319.49, LON0, sec=i))
        assert len(node.pubs["/gps/fix/filtered"].msgs) == 4

    def test_jump_not_forwarded(self):
        node = self._node()
        node._on_fix(_Fix(LAT0, LON0, sec=1))
        node._on_fix(_Fix(LAT0 + 500.0 / 111319.49, LON0, sec=2))
        assert len(node.pubs["/gps/fix/filtered"].msgs) == 1

    def test_negative_status_not_forwarded(self):
        node = self._node()
        for i in range(3):
            node._on_fix(_Fix(LAT0, LON0, sec=i, status=-1))
        assert node.pubs["/gps/fix/filtered"].msgs == []

    def test_high_cov_not_forwarded(self):
        node = self._node()
        node._on_fix(_Fix(LAT0, LON0, sec=1))
        node._on_fix(_Fix(LAT0, LON0, sec=2, cov=99.0))
        assert len(node.pubs["/gps/fix/filtered"].msgs) == 1

    def test_no_covariance_only_jump_gate(self):
        node = self._node()
        node._on_fix(_Fix(LAT0, LON0, sec=1, cov_type=0))
        node._on_fix(_Fix(LAT0 + 1.0 / 111319.49, LON0, sec=2, cov_type=0))
        assert len(node.pubs["/gps/fix/filtered"].msgs) == 2
