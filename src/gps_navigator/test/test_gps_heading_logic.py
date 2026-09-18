# -*- coding: utf-8 -*-
"""Тесты чистой логики курса по GPS (gps_navigator/gps_heading_logic.py)."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from gps_navigator.gps_heading_logic import (
    HeadingEstimator,
    displacement_m,
    yaw_covariance,
    yaw_to_quaternion,
)

# Таллиннский район, широта ~59.4: cos(lat) ~ 0.51
LAT0 = 59.4
LON0 = 24.7
M_PER_DEG_LON_AT_LAT0 = math.radians(1.0) * 6378137.0 * \
    math.cos(math.radians(LAT0))


def _move(est, t, lat, lon):
    return est.add(t, lat, lon)


class TestDisplacement:
    def test_north_is_positive_y(self):
        dx, dy = displacement_m(LAT0, LON0, LAT0 + 0.001, LON0)
        assert abs(dx) < 1e-6
        assert 100 < dy < 115          # ~111 м на 0.001°

    def test_east_is_positive_x(self):
        dx, dy = displacement_m(LAT0, LON0, LAT0, LON0 + 0.001)
        assert abs(dy) < 1e-6
        assert 50 < dx < 60            # cos(59.4°) ~ 0.51 -> ~57 м


class TestEstimator:
    def _est(self, **kw):
        return HeadingEstimator(**kw)

    def test_returns_none_until_min_dist(self):
        est = self._est(min_dist_m=2.0)
        assert est.add(0.0, LAT0, LON0) is None
        # 1 м за 1 с — меньше порога
        assert est.add(1.0, LAT0 + 1.0 / 111319.49, LON0) is None
        # ещё 1.5 м — суммарно 2.5 м за окно
        s = est.add(2.0, LAT0 + 2.5 / 111319.49, LON0)
        assert s is not None
        assert abs(s.dist_m - 2.5) < 1e-6

    def test_north_motion_yaw_is_half_pi(self):
        est = self._est(min_dist_m=2.0)
        est.add(0.0, LAT0, LON0)
        s = est.add(3.0, LAT0 + 3.0 / 111319.49, LON0)
        assert s is not None
        assert abs(s.yaw_rad - math.pi / 2) < 1e-9
        assert abs(s.speed_mps - 1.0) < 1e-6

    def test_east_motion_yaw_is_zero(self):
        est = self._est(min_dist_m=2.0)
        est.add(0.0, LAT0, LON0)
        s = est.add(3.0, LAT0, LON0 + 3.0 / M_PER_DEG_LON_AT_LAT0)
        assert s is not None
        assert abs(s.yaw_rad) < 1e-9

    def test_south_motion_yaw_is_minus_half_pi(self):
        est = self._est(min_dist_m=2.0)
        est.add(0.0, LAT0, LON0)
        s = est.add(3.0, LAT0 - 3.0 / 111319.49, LON0)
        assert abs(s.yaw_rad + math.pi / 2) < 1e-9

    def test_stationary_gives_none(self):
        est = self._est(min_dist_m=2.0)
        est.add(0.0, LAT0, LON0)
        for i in range(1, 20):
            assert est.add(i * 0.5, LAT0, LON0) is None

    def test_gps_jump_rejected(self):
        est = self._est(min_dist_m=2.0, max_speed_mps=30.0)
        est.add(0.0, LAT0, LON0)
        # прыжок 500 м за 0.1 с = 5000 м/с — не принимается
        assert est.add(0.1, LAT0 + 500.0 / 111319.49, LON0) is None
        assert est.rejected_jumps == 1
        # нормальный фикс принимается
        s = est.add(3.0, LAT0 + 3.0 / 111319.49, LON0)
        assert s is not None
        assert s.dist_m < 10           # окно не испорчено прыжком

    def test_window_purges_old_fixes(self):
        est = self._est(window_s=3.0, min_dist_m=2.0)
        est.add(0.0, LAT0, LON0)
        est.add(1.0, LAT0, LON0)
        # через 10 с оба старых фикса устарели -> оценки нет (окно пусто)
        assert est.add(10.0, LAT0 + 3.0 / 111319.49, LON0) is None
        # окно наполняется заново и даёт честную скорость 1 м/с
        est.add(11.0, LAT0 + 4.0 / 111319.49, LON0)
        s = est.add(12.0, LAT0 + 5.0 / 111319.49, LON0)
        assert s is not None
        assert abs(s.speed_mps - 1.0) < 0.05
        assert abs(s.span_s - 2.0) < 0.51

    def test_direction_change_reflected(self):
        est = self._est(window_s=2.0, min_dist_m=1.0)
        est.add(0.0, LAT0, LON0)
        s1 = est.add(2.0, LAT0, LON0 + 2.0 / M_PER_DEG_LON_AT_LAT0)  # восток
        assert abs(s1.yaw_rad) < 1e-6
        # окно уедет от старых фиксов и повернёт на север
        for i in range(6):
            est.add(3.0 + i, LAT0 + (i + 1) * 1.0 / 111319.49,
                    LON0 + 2.0 / M_PER_DEG_LON_AT_LAT0)
        s2 = est.add(9.0, LAT0 + 7.0 / 111319.49,
                     LON0 + 2.0 / M_PER_DEG_LON_AT_LAT0)
        assert abs(s2.yaw_rad - math.pi / 2) < 0.05

    def test_clock_rollback_resets_buffer(self):
        est = self._est(window_s=3.0, min_dist_m=2.0)
        est.add(100000.0, LAT0, LON0)          # «монотонные» часы
        est.add(100001.0, LAT0, LON0)
        # время скакнуло назад на другую эпоху — буфер должен сброситься,
        # а оценка — честно набраться заново (2 м перемещения за новое окно)
        assert est.add(0.0, LAT0 + 1.0 / 111319.49, LON0) is None
        assert est.add(1.0, LAT0 + 2.0 / 111319.49, LON0) is None
        s = est.add(2.0, LAT0 + 3.0 / 111319.49, LON0)
        assert s is not None
        assert abs(s.speed_mps - 1.0) < 0.05

    def test_bad_ctor_args(self):
        for kw in ({"window_s": 0.0}, {"min_dist_m": -1.0}):
            try:
                HeadingEstimator(**kw)
                raise AssertionError("ожидался ValueError")
            except ValueError:
                pass


class TestHelpers:
    def test_yaw_covariance_bounds(self):
        assert yaw_covariance(1.0, 0.05, 1.0, 1.0) == 0.05
        assert yaw_covariance(0.5, 0.05, 1.0, 1.0) == 0.2
        assert yaw_covariance(0.0, 0.05, 1.0, 1.0) == 1.0
        assert yaw_covariance(10.0, 0.05, 1.0, 1.0) == 0.05

    def test_yaw_to_quaternion(self):
        x, y, z, w = yaw_to_quaternion(math.pi / 2)
        assert abs(x) < 1e-12 and abs(y) < 1e-12
        assert abs(z - math.sqrt(0.5)) < 1e-12
        assert abs(w - math.sqrt(0.5)) < 1e-12
        _, _, z0, w0 = yaw_to_quaternion(0.0)
        assert z0 == 0.0 and abs(w0 - 1.0) < 1e-12


if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main([__file__]))
