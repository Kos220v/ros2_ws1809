# -*- coding: utf-8 -*-

"""
Тесты конфигов навигационного слоя.

Здесь проверяются НЕ «что YAML парсится», а требования ТЗ:
  * линейная скорость в EKF берётся только из /odom/vesc и только vx;
  * повороты — только из кватерниона IMU (включена только строка ориентации);
  * TF публикуют два EKF (odom->base_link и map->odom), robot_odom — не в Nav2;
  * частоты Nav2 ограничены (Raspberry Pi 5), Spin в поведениях выключен,
    допуск цели согласован с точностью GPS, объезд — по лидару.

Пути к конфигам вычисляются от дерева исходников (работает и в colcon test,
и при запуске pytest из любого места).
"""

import math
import os

import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
NAV_CONFIG = os.path.normpath(
    os.path.join(HERE, "..", "..", "gps_navigator", "config"))
if not os.path.exists(NAV_CONFIG):
    # запуск из colcon test: конфиги лежат в установленном share
    from ament_index_python.packages import get_package_share_directory
    NAV_CONFIG = os.path.join(get_package_share_directory("gps_navigator"),
                              "config")
EKF_PATH = os.path.join(NAV_CONFIG, "ekf_localization.yaml")
NAV2_PATH = os.path.join(NAV_CONFIG, "nav2_params.yaml")
BT_PATH = os.path.join(NAV_CONFIG, "behavior_tree.xml")
YDLIDAR_PATH = os.path.normpath(os.path.join(
    HERE, "..", "..", "project_start", "params", "ydlidar_params.yaml"))


@pytest.fixture(scope="module")
def ekf():
    with open(EKF_PATH, encoding="utf-8") as f:
        return yaml.safe_load(f)


@pytest.fixture(scope="module")
def nav2():
    with open(NAV2_PATH, encoding="utf-8") as f:
        return yaml.safe_load(f)


# ============================================================ robot_localization

@pytest.mark.parametrize("node", ["ekf_odom", "ekf_map"])
def test_all_config_arrays_have_15_elements(ekf, node):
    params = ekf[node]["ros__parameters"]
    arrays = {k: v for k, v in params.items() if k.endswith("_config")}
    assert arrays, f"у {node} нет ни одного *_config"
    for name, arr in arrays.items():
        assert len(arr) == 15, f"{node}.{name}: нужно 15 значений, получено {len(arr)}"


@pytest.mark.parametrize("node", ["ekf_odom", "ekf_map"])
def test_linear_velocity_only_from_vesc(ekf, node):
    """ТЗ: линейное перемещение — из kolesa_control, только vx."""
    params = ekf[node]["ros__parameters"]
    assert params["odom0"] == "/odom/vesc", "odom0 должен быть /odom/vesc"
    cfg = params["odom0_config"]
    # Порядок robot_localization: x,y,z, roll,pitch,yaw, vx,vy,vz,
    #                             vroll,vpitch,vyaw, ax,ay,az
    assert cfg[6] is True, "vx из /odom/vesc должен быть включён"
    for idx, what in ((0, "x"), (1, "y"), (2, "z"), (3, "roll"), (4, "pitch"),
                      (5, "yaw из колёс"), (7, "vy"), (8, "vz"),
                      (9, "vroll"), (10, "vpitch"), (11, "vyaw из колёс"),
                      (12, "ax"), (13, "ay"), (14, "az")):
        assert cfg[idx] is False, f"{node}.odom0_config: {what} должен быть выключен"


@pytest.mark.parametrize("node", ["ekf_odom", "ekf_map"])
def test_yaw_only_from_imu_quaternion(ekf, node):
    """ТЗ: повороты — из IMU, из готового кватерниона (только ориентация)."""
    params = ekf[node]["ros__parameters"]
    assert params["imu0"] == "/imu/data"
    cfg = params["imu0_config"]
    assert cfg[3] is True and cfg[4] is True and cfg[5] is True, \
        "строка ориентации (roll/pitch/yaw) должна быть включена"
    for idx, what in ((0, "x"), (1, "y"), (2, "z"),
                      (6, "vx"), (7, "vy"), (8, "vz"),
                      (9, "угловая скорость roll"), (10, "угловая скорость pitch"),
                      (11, "угловая скорость yaw"),
                      (12, "ax"), (13, "ay"), (14, "az")):
        assert cfg[idx] is False, f"{node}.imu0_config: {what} должен быть выключен"


@pytest.mark.parametrize("node", ["ekf_odom", "ekf_map"])
def test_gps_orientation_never_used(ekf, node):
    """Ориентация из GPS-сообщений (navsat/магнитометр) не берётся нигде.

    Единственное исключение — pose0 в ekf_map: курс по фактическому
    перемещению (узел gps_heading, вычислен из последовательных позиций,
    а не из поля orientation). Там должен быть включён ТОЛЬКО yaw:
    позиция, roll и pitch выключены.
    """
    params = ekf[node]["ros__parameters"]
    for key, cfg in params.items():
        if not key.endswith("_config") or key in ("odom0_config", "imu0_config"):
            continue
        if key == "pose0_config":
            assert cfg[0] is False and cfg[1] is False and cfg[2] is False, \
                f"{node}.{key}: позиция из pose0 запрещена (только yaw)"
            assert cfg[3] is False and cfg[4] is False, \
                f"{node}.{key}: roll/pitch из pose0 запрещены"
            assert cfg[5] is True, f"{node}.{key}: yaw должен быть включён"
            continue
        assert cfg[3] is False and cfg[4] is False and cfg[5] is False, \
            f"{node}.{key}: ориентация из этого источника запрещена"


def test_gps_heading_pose0_wiring(ekf):
    """pose0 подключён к /gps/heading, абсолютный (не differential)."""
    p = ekf["ekf_map"]["ros__parameters"]
    assert p["pose0"] == "/gps/heading"
    assert p["pose0_differential"] is False
    assert p["pose0_relative"] is False


def test_tf_split_between_two_ekf(ekf):
    odom_p = ekf["ekf_odom"]["ros__parameters"]
    map_p = ekf["ekf_map"]["ros__parameters"]
    assert odom_p["world_frame"] == "odom" and odom_p["publish_tf"] is True
    assert map_p["world_frame"] == "map" and map_p["publish_tf"] is True
    assert odom_p["two_d_mode"] is True and map_p["two_d_mode"] is True


def test_both_ekf_share_the_same_yaw_offset(ekf):
    a = ekf["ekf_odom"]["ros__parameters"]["imu0_yaw_offset"]
    b = ekf["ekf_map"]["ros__parameters"]["imu0_yaw_offset"]
    assert a == pytest.approx(b), "imu0_yaw_offset должен совпадать в обоих EKF"
    assert abs(math.degrees(a) + 48.0) < 0.5, \
        "ожидается поправка монтажа IMU -48° (как imu_yaw_offset_deg в start.launch.py)"


def test_ekf_rates_are_pi_friendly(ekf):
    assert ekf["ekf_odom"]["ros__parameters"]["frequency"] <= 30.0
    assert ekf["ekf_map"]["ros__parameters"]["frequency"] <= 15.0


def test_navsat_does_not_publish_conflicting_tf(ekf):
    p = ekf["navsat_transform"]["ros__parameters"]
    assert p["broadcast_cartesian_transform"] is False
    assert p["broadcast_cartesian_transform_as_parent_frame"] is False
    assert p["use_odometry_yaw"] is False, "курс должен браться из IMU, не из одометрии"


# ========================================================================= Nav2

def test_nav2_rates_are_pi_friendly(nav2):
    assert nav2["controller_server"]["ros__parameters"]["controller_frequency"] <= 10.0
    assert nav2["planner_server"]["ros__parameters"]["expected_planner_frequency"] <= 1.0
    lc = nav2["local_costmap"]["local_costmap"]["ros__parameters"]
    gc = nav2["global_costmap"]["global_costmap"]["ros__parameters"]
    assert lc["update_frequency"] <= 5.0
    assert gc["update_frequency"] <= 1.0
    # Костмапы должны быть компактными: это главный потребитель CPU.
    assert lc["width"] * lc["height"] / (lc["resolution"] ** 2) <= 20_000
    assert gc["width"] * gc["height"] / (gc["resolution"] ** 2) <= 60_000


def test_no_unneeded_nav2_servers(nav2):
    """Не запускаем то, что зря ест CPU (их нет и в navigation.launch.py)."""
    for forbidden in ("smoother_server", "velocity_smoother", "collision_monitor",
                      "amcl", "map_server"):
        assert forbidden not in nav2, f"{forbidden} не должен использоваться"


def test_costmaps_use_lidar_and_clear_with_inf(nav2):
    for costmap in (nav2["local_costmap"]["local_costmap"]["ros__parameters"],
                    nav2["global_costmap"]["global_costmap"]["ros__parameters"]):
        layer = ("voxel_layer" if "voxel_layer" in costmap
                 else "obstacle_layer")
        scan = costmap[layer]["scan"]
        assert scan["topic"] == "/scan"
        assert scan["clearing"] is True, "лучи без препятствия должны чистить костмап"
        assert scan["marking"] is True
        assert scan["raytrace_max_range"] >= scan["obstacle_max_range"]
    # Драйвер обязан отдавать +inf вместо 0.0 — иначе очистки не будет.
    with open(YDLIDAR_PATH, encoding="utf-8") as f:
        yd = yaml.safe_load(f)
    assert yd["ydlidar_ros2_driver_node"]["ros__parameters"]["invalid_range_is_inf"] \
        is True


def test_no_spin_recovery_for_tracked_robot(nav2):
    plugins = nav2["behavior_server"]["ros__parameters"]["behavior_plugins"]
    assert "spin" not in plugins, "Spin на гусеницах роет грунт"
    assert "backup" in plugins


def test_goal_tolerance_matches_gps_accuracy(nav2):
    gc = nav2["controller_server"]["ros__parameters"]["general_goal_checker"]
    assert gc["xy_goal_tolerance"] >= 1.0, "бытовой GPS даёт 2-5 м"
    assert gc["yaw_goal_tolerance"] > math.pi, \
        "у GPS-точки нет желаемого курса — yaw не проверяем"


def test_velocity_limits_match_kolesa_control(nav2):
    fp = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    # max_linear_velocity в kolesa_control = 1.0 м/с, max_angular = 1.0 рад/с
    assert fp["max_vel_x"] <= 1.0
    assert fp["max_speed_xy"] <= 1.0
    assert fp["max_vel_theta"] <= 1.0


def test_waypoint_follower_continues_after_single_failure(nav2):
    wf = nav2["waypoint_follower"]["ros__parameters"]
    assert wf["stop_on_failure"] is False, \
        "промах по одной точке (шум GPS) не должен срывать весь маршрут"


def test_behavior_tree_has_no_spin_and_replans():
    with open(BT_PATH, encoding="utf-8") as f:
        xml = f.read()
    assert "<Spin" not in xml
    assert "ComputePathThroughPoses" in xml
    assert "FollowPath" in xml
    assert "BackUp" in xml
    assert 'goal_checker_id="general_goal_checker"' in xml
