# -*- coding: utf-8 -*-
"""Смоук-тест узла GpsMission на заглушках rclpy.

Страховка после реального инцидента: узел собирался, py_compile и тесты
чистой логики проходили, а на роботе __init__ падал с AttributeError —
кусок присваиваний атрибутов был потерян. Здесь узел ПОЛНОСТЬЮ
инстанцируется (оба значения start_on_auto) и дёргаются колбэки, так что
любой потерянный атрибут/метод ловится здесь, а не на железе.

Заглушки ставятся в sys.modules принудительно (с сохранением прежних
значений и восстановлением после модуля), чтобы не зависеть от того,
какие rclpy-заглушки успели попасть в кэш от других пакетов.
"""
import os
import sys
import types

import pytest

# ----------------------------------------------------------------- заглушки
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


class _Clock:
    class _Stamp:
        pass

    def now(self):
        return _Clock._Stamp()


class _Pub:
    def __init__(self):
        self.msgs = []

    def publish(self, m):
        self.msgs.append(m)


class _Client:
    def wait_for_service(self, timeout_sec=None):
        return False


class _ActionClient:
    server_up = False

    def __init__(self, node, action_type, name, **kw):
        self.action_type = action_type
        self.name = name

    def wait_for_server(self, timeout_sec=None):
        return _ActionClient.server_up


class _StubNode:
    """Минимальный rclpy.node.Node: параметры, подписки, сервисы, логгер."""

    overrides = {}

    def __init__(self, name, **kw):
        self.name = name
        self._defaults = {}
        self.subs = []
        self.pubs = {}
        self.srvs = []
        self.timers = []
        self._logger = _Logger()

    # -- параметры
    def declare_parameter(self, name, value):
        self._defaults[name] = value

    def get_parameter(self, name):
        return _Param(_StubNode.overrides.get(name, self._defaults[name]))

    # -- сущности ROS
    def create_subscription(self, typ, topic, cb, qos):
        self.subs.append((typ, topic, cb))
        return (typ, topic)

    def create_publisher(self, typ, topic, qos):
        p = _Pub()
        self.pubs[topic] = p
        return p

    def create_client(self, srv, topic):
        return _Client()

    def create_service(self, srv, topic, cb):
        self.srvs.append((srv, topic, cb))

    def create_timer(self, period, cb):
        self.timers.append((period, cb))

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return _Clock()


def _install_stubs():
    """Создать модули-заглушки и вернуть сохранённые оригиналы."""
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

    action_mod = mod("rclpy.action")
    action_mod.ActionClient = _ActionClient

    # --- сообщения
    geo = mod("geographic_msgs.msg")

    class GeoPoseStamped:
        def __init__(self):
            self.header = types.SimpleNamespace(frame_id="", stamp=None)
            self.pose = types.SimpleNamespace(
                position=types.SimpleNamespace(
                    latitude=0.0, longitude=0.0, altitude=0.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))

    geo.GeoPoseStamped = GeoPoseStamped

    nav2a = mod("nav2_msgs.action")

    class FollowGPSWaypoints:
        class Goal:
            def __init__(self):
                self.number_of_loops = 0
                self.goal_index = 0
                self.gps_poses = []

        class Result:
            def __init__(self):
                self.missed_waypoints = []

    nav2a.FollowGPSWaypoints = FollowGPSWaypoints

    rl_srv = mod("robot_localization.srv")

    class SetDatum:
        class Request:
            def __init__(self):
                self.geo_pose = GeoPoseStamped().pose

    rl_srv.SetDatum = SetDatum

    sens = mod("sensor_msgs.msg")

    class NavSatFix:
        def __init__(self):
            self.status = types.SimpleNamespace(status=0)
            self.latitude = 0.0
            self.longitude = 0.0

    sens.NavSatFix = NavSatFix

    std = mod("std_msgs.msg")

    class Int8:
        def __init__(self):
            self.data = 0

    class String:
        def __init__(self):
            self.data = ""

    std.Int8 = Int8
    std.String = String

    sstd = mod("std_srvs.srv")

    class Trigger:
        class Request:
            pass

        class Response:
            def __init__(self):
                self.success = False
                self.message = ""

    sstd.Trigger = Trigger
    return saved


_SAVED = _install_stubs()

import gps_mission.gps_mission_node as gmn
from gps_mission.gps_mission_node import GpsMission  # noqa: E402

# имена из заглушек для использования в тестах
Trigger = sys.modules["std_srvs.srv"].Trigger
NavSatFix = sys.modules["sensor_msgs.msg"].NavSatFix

# Зонд сериализации в песочнице невозможен (нет rclpy) — считаем
# окружение исправным по умолчанию; отдельный тест проверяет отказ.
gmn._ENV_PROBE_DONE = True
gmn.ENV_OK = True

# Сразу восстанавливаем sys.modules: узел уже импортирован и держит ссылки
# на наши заглушки в своём пространстве имён, а другим пакетам (например
# robot_odom со своими rclpy-заглушками) ничего мешать не должно —
# независимо от порядка сбора тестов.
for _name, _orig in _SAVED.items():
    if _orig is None:
        sys.modules.pop(_name, None)
    else:
        sys.modules[_name] = _orig
del _SAVED


@pytest.fixture(autouse=True)
def _restore_env():
    _StubNode.overrides = {}
    _ActionClient.server_up = False
    gmn.ENV_OK = True
    yield
    _StubNode.overrides = {}
    _ActionClient.server_up = False
    gmn.ENV_OK = True


class TestNodeConstructs:
    def test_default_start_on_auto_false(self):
        node = GpsMission()
        assert node.start_on_auto is False
        assert node._auto_prev is None
        assert node._resume_index is None
        assert node.state == "IDLE"

    def test_start_on_auto_true_from_params(self):
        _StubNode.overrides["start_on_auto"] = True
        node = GpsMission()
        assert node.start_on_auto is True
        # подписка на /control_mode создана
        topics = [s[1] for s in node.subs]
        assert "/control_mode" in topics
        assert "/gps/fix" in topics

    def test_services_declared(self):
        node = GpsMission()
        srv_topics = [s[1] for s in node.srvs]
        assert "~/start" in srv_topics and "~/stop" in srv_topics

    def test_status_timer_created(self):
        node = GpsMission()
        assert any(abs(p - 0.5) < 1e-9 for p, _ in node.timers)


class TestModeCallback:
    def _node_auto(self):
        _StubNode.overrides["start_on_auto"] = True
        return GpsMission()

    def test_disabled_ignores_mode(self):
        node = GpsMission()  # start_on_auto=False
        node._on_mode(types.SimpleNamespace(data=0))
        assert node._auto_prev is None
        assert node.state == "IDLE"

    def test_first_message_only_records(self):
        node = self._node_auto()
        node._on_mode(types.SimpleNamespace(data=1))  # MANUAL
        assert node._auto_prev is False
        assert node.state == "IDLE"

    def test_toggle_to_auto_with_bad_file_fails_gracefully(self):
        node = self._node_auto()
        node._on_mode(types.SimpleNamespace(data=1))
        node._on_mode(types.SimpleNamespace(data=0))  # -> AUTO
        # waypoints_file пуст -> load_waypoints вернёт ошибку, без падения
        assert node.state == "IDLE"
        assert any(lvl == "E" for lvl, _ in node._logger.lines)

    def test_cancel_with_no_active_mission_is_noop(self):
        node = self._node_auto()
        node._on_mode(types.SimpleNamespace(data=0))
        node._on_mode(types.SimpleNamespace(data=1))  # уход из AUTO
        assert node.state == "IDLE"
        assert node.goal_handle is None

    def test_cancel_active_mission_saves_resume_index(self):
        node = self._node_auto()
        node._on_mode(types.SimpleNamespace(data=0))  # первое: фиксация AUTO
        node.current_waypoint = 4

        class _Goal:
            cancelled = False

            def done(self):
                return False

            def cancel_goal_async(self):
                _Goal.cancelled = True

        node.goal_handle = _Goal()
        node.state = "NAVIGATING"
        node._on_mode(types.SimpleNamespace(data=1))  # уход из AUTO
        assert node.state == "CANCELLED"
        assert node._resume_index == 4
        assert _Goal.cancelled is True


class TestMissionActions:
    def test_start_refused_when_env_broken(self):
        # сломанный Python->C конвертер GeoPose: осмысленный отказ вместо
        # SIGABRT процесса
        gmn.ENV_OK = False
        node = GpsMission()
        ok, msg = node._start_mission()
        assert ok is False
        assert "окружение ROS сломано" in msg
        assert "apt" in msg
        assert node.state == "IDLE"
        gmn.ENV_OK = True

    def test_start_with_unreachable_action_server(self, tmp_path):
        route = tmp_path / "w.yaml"
        route.write_text(
            "waypoints:\n"
            "  - lat: 56.299\n"
            "    lon: 43.922\n"
            "    radius: 2.0\n",
            encoding="utf-8")
        _StubNode.overrides["waypoints_file"] = str(route)
        node = GpsMission()
        ok, msg = node._start_mission()
        assert ok is False
        assert "follow_gps_waypoints" in msg
        assert node.state == "IDLE"

    def test_stop_without_mission(self):
        node = GpsMission()
        req, resp = types.SimpleNamespace(), Trigger.Response()
        out = node._srv_stop(req, resp)
        assert out.success is True
        assert "нет" in out.message

    def test_publish_status_message(self):
        node = GpsMission()
        node.total_waypoints = 3
        node._publish_status()
        (msg,) = node.pubs["/gps_mission/status"].msgs
        # current_waypoint ещё -1 -> "0/3"
        assert "IDLE" in msg.data and "0/3" in msg.data

    def test_fix_negative_status_ignored(self):
        node = GpsMission()
        fix = NavSatFix()
        fix.status.status = -1
        node._on_fix(fix)
        assert node.last_fix is None
        fix.status.status = 0
        fix.latitude, fix.longitude = 56.3, 43.9
        node._on_fix(fix)
        assert node.last_fix == (56.3, 43.9)
