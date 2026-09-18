# -*- coding: utf-8 -*-

"""
Структурная проверка launch-файлов маршрута.

Что проверяется (без установленной ROS 2 — `launch`/`launch_ros` подменяются
заглушками, поэтому тест ловит опечатки и ошибки связности, а не синтаксис):
  * start.launch.py объявляет все аргументы, которые ему передаёт
    route.launch.py (иначе launch упадёт с «argument not declared»);
  * route.launch.py подключает start.launch.py с odom_publish_tf:=false —
    иначе будет два издателя TF odom->base_link (robot_odom + ekf_odom);
  * navigation.launch.py поднимает ровно те узлы, что нужны для GPS-маршрута,
    и НЕ поднимает smoother_server/velocity_smoother/collision_monitor;
  * команды Nav2 уходят в /cmd_vel/auto (через cmd_switcher, где пульт
    ELRS имеет приоритет), а не напрямую в /cmd_vel.

Тест выполняется только если в системе нет настоящего launch_ros: на роботе
смысла в заглушках нет (там проверка — реальный `ros2 launch --print`).
"""

import importlib
import importlib.util
import os
import sys
import types

import pytest

try:
    import launch_ros
    HAS_LAUNCH_ROS = launch_ros is not None
except ImportError:
    HAS_LAUNCH_ROS = False

HERE = os.path.dirname(os.path.abspath(__file__))
LAUNCH_DIR = os.path.normpath(os.path.join(HERE, "..", "launch"))
START_LAUNCH = os.path.normpath(os.path.join(
    HERE, "..", "..", "project_start", "launch", "start.launch.py"))

pytestmark = pytest.mark.skipif(
    HAS_LAUNCH_ROS,
    reason="установлен настоящий launch_ros — проверяйте `ros2 launch --print`")


# ------------------------------------------------------------------ заглушки
RECORD = {"nodes": [], "launch_arguments": {}, "params": {}}


def _install_stub_modules(monkeypatch):
    launch_mod = types.ModuleType("launch")
    actions_mod = types.ModuleType("launch.actions")
    subs_mod = types.ModuleType("launch.substitutions")
    lds_mod = types.ModuleType("launch.launch_description_sources")
    launch_ros_mod = types.ModuleType("launch_ros")
    lr_actions = types.ModuleType("launch_ros.actions")
    lr_pd = types.ModuleType("launch_ros.parameter_descriptions")

    class LaunchDescription:
        def __init__(self, entities=None):
            self.entities = list(entities or [])

    class DeclareLaunchArgument:
        def __init__(self, name, default_value=None, description=""):
            self.name = name
            self.default_value = default_value

    class _Action:
        def __init__(self, *a, **k):
            self.args = a
            self.kwargs = k

    class OpaqueFunction(_Action):
        def __init__(self, function=None, args=None, kwargs=None):
            self.function = function

    class TimerAction(_Action):
        def __init__(self, period=None, actions=None):
            self.period = period
            self.actions = list(actions or [])

    class IncludeLaunchDescription(_Action):
        def __init__(self, source, launch_arguments=None):
            self.source = source
            self.launch_arguments = dict(launch_arguments or {})
            RECORD["launch_arguments"].update(self.launch_arguments)

    class PythonLaunchDescriptionSource:
        def __init__(self, path):
            self.path = path

    class Command:
        def __init__(self, *a, **k):
            pass

    class LaunchConfiguration:
        """Заглушка: возвращает «разумное» значение по имени аргумента."""

        DEFAULTS = {
            "use_gps": "true",
            "use_robot_odom": "true",
            "gps_port": "/dev/ttyAMA2",
            "imu_port": "/dev/ttyAMA1",
            "lidar_port": "/dev/ttyUSB0",
            "odom_publish_tf": "false",
            "odom_yaw_mode": "absolute",
        }

        def __init__(self, name, default=None):
            self.name = name

        def perform(self, context=None):
            return self.DEFAULTS.get(self.name, "0")

    class Node:
        def __init__(self, **kwargs):
            self.package = kwargs.get("package")
            self.executable = kwargs.get("executable")
            self.name = kwargs.get("name")
            self.remap = kwargs.get("remappings") or []
            self.parameters = kwargs.get("parameters") or []
            RECORD["nodes"].append(self)

    class ParameterValue:
        def __init__(self, value=None, value_type=None):
            self.value = value

    for mod, attrs in (
        (launch_mod, {"LaunchDescription": LaunchDescription}),
        (actions_mod, {"DeclareLaunchArgument": DeclareLaunchArgument,
                       "OpaqueFunction": OpaqueFunction,
                       "TimerAction": TimerAction,
                       "IncludeLaunchDescription": IncludeLaunchDescription}),
        (subs_mod, {"Command": Command,
                    "LaunchConfiguration": LaunchConfiguration}),
        (lds_mod, {"PythonLaunchDescriptionSource":
                       PythonLaunchDescriptionSource}),
        (lr_actions, {"Node": Node}),
        (lr_pd, {"ParameterValue": ParameterValue}),
    ):
        for k, v in attrs.items():
            setattr(mod, k, v)

    launch_mod.LaunchDescription = LaunchDescription
    launch_mod.actions = actions_mod
    launch_mod.substitutions = subs_mod
    launch_mod.launch_description_sources = lds_mod
    launch_ros_mod.actions = lr_actions
    launch_ros_mod.parameter_descriptions = lr_pd

    monkeypatch.setitem(sys.modules, "launch", launch_mod)
    monkeypatch.setitem(sys.modules, "launch.actions", actions_mod)
    monkeypatch.setitem(sys.modules, "launch.substitutions", subs_mod)
    monkeypatch.setitem(sys.modules, "launch.launch_description_sources", lds_mod)
    monkeypatch.setitem(sys.modules, "launch_ros", launch_ros_mod)
    monkeypatch.setitem(sys.modules, "launch_ros.actions", lr_actions)
    monkeypatch.setitem(sys.modules, "launch_ros.parameter_descriptions", lr_pd)

    ament = types.ModuleType("ament_index_python")
    ament_pkgs = types.ModuleType("ament_index_python.packages")
    ament_pkgs.get_package_share_directory = lambda name: f"/opt/fake/{name}"
    monkeypatch.setitem(sys.modules, "ament_index_python", ament)
    monkeypatch.setitem(sys.modules, "ament_index_python.packages", ament_pkgs)


def _load_launch(path, monkeypatch):
    RECORD["nodes"].clear()
    RECORD["launch_arguments"].clear()
    _install_stub_modules(monkeypatch)
    spec = importlib.util.spec_from_file_location(f"launch_{os.getpid()}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    # раскрываем OpaqueFunction, чтобы получить список узлов
    context = types.SimpleNamespace()
    for entity in description.entities:
        if hasattr(entity, "function"):
            entity.function(context)
    return description, list(RECORD["nodes"]), dict(RECORD["launch_arguments"])


def _declared_args(description):
    return {e.name for e in description.entities if hasattr(e, "name")
            and e.__class__.__name__ == "DeclareLaunchArgument"}


# ------------------------------------------------------------------- тесты
# аргументы, которые route.launch.py обязан передавать в start.launch.py
START_ARGS = {
    "use_gps", "lidar_delay", "gps_port", "imu_port", "lidar_port",
    "declination_deg", "imu_yaw_offset_deg", "odom_publish_tf",
    "odom_yaw_mode", "use_robot_odom",
}


def test_start_launch_declares_all_route_arguments(monkeypatch):
    start_desc, _, _ = _load_launch(START_LAUNCH, monkeypatch)
    declared = _declared_args(start_desc)
    missing = START_ARGS - declared
    assert not missing, (
        f"start.launch.py не объявляет аргументы: {sorted(missing)}")
    # аргумент по умолчанию не должен включать TF от robot_odom
    defaults = {e.name: e.default_value for e in start_desc.entities
                if e.__class__.__name__ == "DeclareLaunchArgument"}
    assert defaults.get("odom_publish_tf") == "false"


def test_route_disables_robot_odom_tf(monkeypatch):
    _, _, route_args = _load_launch(
        os.path.join(LAUNCH_DIR, "route.launch.py"), monkeypatch)
    assert route_args.get("odom_publish_tf") == "false", (
        "TF odom->base_link должен публиковать ekf_odom, а не robot_odom")


def test_navigation_launch_nodes(monkeypatch):
    _, nodes, _ = _load_launch(
        os.path.join(LAUNCH_DIR, "navigation.launch.py"), monkeypatch)
    by_exec = {n.executable: n for n in nodes}

    expected = {
        "ekf_node": 2,
        "navsat_transform_node": 1,
        "controller_server": 1,
        "planner_server": 1,
        "behavior_server": 1,
        "bt_navigator": 1,
        "waypoint_follower": 1,
        "lifecycle_manager": 1,
        "gps_mission_node": 1,
    }
    for exe, count in expected.items():
        got = sum(1 for n in nodes if n.executable == exe)
        assert got == count, f"{exe}: ожидалось {count}, найдено {got}"

    for forbidden in ("smoother_server", "velocity_smoother", "collision_monitor",
                      "map_server", "amcl"):
        assert forbidden not in by_exec, f"{forbidden} не должен запускаться"


def test_nav2_commands_go_through_cmd_switcher(monkeypatch):
    _, nodes, _ = _load_launch(
        os.path.join(LAUNCH_DIR, "navigation.launch.py"), monkeypatch)
    # behavior_server тоже получает remapping (он публикует cmd_vel при
    # выполнениях поведений), но критичны контроллер и BT-навигатор.
    nav2_execs = {"controller_server", "bt_navigator", "waypoint_follower"}
    for node in nodes:
        if node.executable in nav2_execs:
            assert ("cmd_vel", "/cmd_vel/auto") in node.remap, (
                f"{node.executable}: cmd_vel должен быть перенаправлен в "
                "/cmd_vel/auto (приоритет пульта ELRS через cmd_switcher)")


def test_start_launch_hardware_nodes(monkeypatch):
    _, nodes, _ = _load_launch(START_LAUNCH, monkeypatch)
    exes = {n.executable for n in nodes}
    for needed in ("kolesa_control", "bridge_node", "elrs_node",
                   "ydlidar_ros2_driver_node", "robot_state_publisher",
                   "cmd_mux_node", "relay_node", "odom_node",
                   "nmea_serial_driver"):
        assert needed in exes, f"start.launch.py: нет узла {needed}"
