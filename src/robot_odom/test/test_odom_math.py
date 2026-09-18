# -*- coding: utf-8 -*-

"""Тесты чистой математики одометрии (без ROS)."""

import math

import pytest

from robot_odom.odom_math import (
    UNMEASURED_COV,
    integrate_pose,
    normalize_quaternion,
    pose_covariance,
    quaternion_from_yaw,
    quaternion_to_yaw,
    twist_covariance,
    wrap_angle,
)


@pytest.mark.parametrize("yaw", [0.0, 0.3, -1.2, math.pi * 0.99, -math.pi * 0.5])
def test_quaternion_roundtrip(yaw):
    qx, qy, qz, qw = quaternion_from_yaw(yaw)
    assert quaternion_to_yaw(qx, qy, qz, qw) == pytest.approx(yaw, abs=1e-12)


def test_normalize_fixes_garbage():
    assert normalize_quaternion(0.0, 0.0, 0.0, 0.0) == (0.0, 0.0, 0.0, 1.0)
    x, y, z, w = normalize_quaternion(0.0, 0.0, 3.0, 4.0)
    assert math.hypot(math.hypot(x, y), math.hypot(z, w)) == pytest.approx(1.0)


def test_wrap_angle():
    assert wrap_angle(3.0 * math.pi) == pytest.approx(math.pi, abs=1e-9)
    assert wrap_angle(-3.0 * math.pi) == pytest.approx(-math.pi, abs=1e-9)
    assert wrap_angle(0.5) == pytest.approx(0.5)


def test_integrate_along_axis():
    # курс на восток (yaw=0): движемся по X
    x, y = integrate_pose(0.0, 0.0, 0.0, 1.0, 2.0)
    assert (x, y) == (pytest.approx(2.0), pytest.approx(0.0, abs=1e-12))
    # курс на север (yaw=+90°): движемся по Y
    x, y = integrate_pose(0.0, 0.0, math.pi / 2, 1.0, 3.0)
    assert x == pytest.approx(0.0, abs=1e-12)
    assert y == pytest.approx(3.0)
    # нулевой dt — стоим
    assert integrate_pose(5.0, 5.0, 1.0, 1.0, 0.0) == (5.0, 5.0)


def test_yaw_is_taken_from_quaternion_not_integrated():
    """
    Курс не интегрируется: даже если «прокрутить» робота на месте (v=0),
    позиция не меняется — дрейф гироскопа в путь не попадает.
    """
    x, y = 0.0, 0.0
    for _ in range(100):
        x, y = integrate_pose(x, y, wrap_angle(math.radians(3.0)), 0.0, 0.05)
    assert (x, y) == (0.0, 0.0)


def test_covariances():
    tc = twist_covariance(True)
    assert len(tc) == 36
    assert tc[0] == pytest.approx(0.05 ** 2)
    assert tc[7] == UNMEASURED_COV          # vy не измерен
    assert twist_covariance(False)[0] == UNMEASURED_COV
    pc = pose_covariance()
    assert len(pc) == 36
    assert pc[0] == pc[7] and pc[35] > 0.0
    assert pc[14] == UNMEASURED_COV
