# -*- coding: utf-8 -*-
"""Тесты чистой логики дрейфа (imu_stm32_bridge/drift_check.py).

Запуск: pytest или python3 -m pytest src/imu_stm32_bridge/test.
Все величины в СИ (рад, рад/с), как в /imu/data; пороги/вывод в °/мин.
"""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from imu_stm32_bridge import drift_check as D


def _seq(n, dt, yaw_fn, wz=0.0, wx=0.0, wy=0.0, spike_at=None):
    """Синтетическая запись /imu/data: t, yaw(t), wx, wy, wz."""
    out = []
    yaw = yaw_fn(0.0)
    for i in range(n):
        t = i * dt
        yaw = yaw_fn(t)
        w = (wx, wy, wz)
        if spike_at is not None and i == spike_at:
            w = (math.radians(30.0), 0.0, 0.0)  # робота толкнули
        out.append((t, yaw, w[0], w[1], w[2]))
    return out


class TestVerdicts:
    def test_no_drift_is_ok(self):
        # курс стоит, wz = 0
        s = _seq(300, 0.02, lambda t: 0.3)
        r = D.analyze(s)
        assert r.verdict == D.OK
        assert abs(r.drift_dpm) < 1e-6

    def test_small_drift_within_warn_is_ok(self):
        # 0.5 °/мин = 0.15 мрад/с — меньше warn_dpm=1
        rate = math.radians(0.5 / 60.0)
        s = _seq(300, 0.02, lambda t: rate * t, wz=rate)
        r = D.analyze(s)
        assert r.verdict == D.OK

    def test_constant_drift_matching_wz_is_gyro_bias(self):
        # 3 °/мин чистого bias: yaw интегрирует wz
        dps = 3.0 / 60.0
        rate = math.radians(dps)
        s = _seq(1500, 0.02, lambda t: rate * t, wz=rate)
        r = D.analyze(s)
        assert r.verdict == D.GYRO_BIAS
        assert abs(r.drift_dpm - 3.0) < 0.01
        assert abs(r.wz_mean_dps - dps) < 1e-9

    def test_negative_drift_also_gyro_bias(self):
        rate = math.radians(-5.0 / 60.0)
        s = _seq(1500, 0.02, lambda t: rate * t, wz=rate)
        assert D.analyze(s).verdict == D.GYRO_BIAS

    def test_wander_with_zero_wz_is_mag(self):
        # курс блуждает синусоидой (±3°, период 45 с), гироскоп чистый.
        # За 60 с сеть сдвиг ~ -2.6°/-мин -> выше warn, а wz не объясняет
        s = _seq(3000, 0.02,
                 lambda t: 0.3 + math.radians(3.0) * math.sin(2 * math.pi * t / 45.0))
        r = D.analyze(s)
        assert r.verdict == D.MAG
        assert abs(r.drift_dpm) > 1.0

    def test_symmetric_wander_returns_to_ok(self):
        # то же блуждание, но окно ровно в период: сеть дрейф ~0 —
        # для маршрута это не страшно, честный OK
        s = _seq(1500, 0.02,
                 lambda t: 0.3 + math.radians(3.0) * math.sin(2 * math.pi * t / 30.0))
        assert D.analyze(s).verdict == D.OK

    def test_drift_far_from_wz_is_mag(self):
        # дрейф 3 °/мин, а wz объясняет только 0.02 °/мин — тянет магнит
        s = _seq(3000, 0.02,
                 lambda t: 0.3 + math.radians(3.0 / 60.0) * t,
                 wz=math.radians(0.02 / 60.0))
        r = D.analyze(s)
        assert r.verdict == D.MAG

    def test_motion_spike_invalidates_measurement(self):
        rate = math.radians(3.0 / 60.0)
        s = _seq(1500, 0.02, lambda t: rate * t, wz=rate, spike_at=100)
        r = D.analyze(s)
        assert r.verdict == D.MOVED


class TestNumbers:
    def test_unwrap_across_pi(self):
        # курс идёт 179° -> 181° (через -179°): разворот не должен дать -358°
        yaws = [math.radians(179.0), math.radians(179.5),
                math.radians(-179.5), math.radians(-179.0)]
        out = D.unwrap_yaw(yaws)
        assert out[-1] - out[0] == D.unwrap_yaw(out)[-1] - out[0]
        assert abs((out[-1] - out[0]) - math.radians(2.0)) < 1e-9

    def test_reported_degrees(self):
        rate = math.radians(6.0 / 60.0)
        s = _seq(1500, 0.02, lambda t: rate * t, wz=rate)
        r = D.analyze(s)
        assert r.duration_s == 1500 * 0.02 - 0.02
        assert abs(r.yaw_end_deg - r.yaw_start_deg - 6.0 * (
            r.duration_s / 60.0)) < 1e-6

    def test_sigma_zero_for_clean_bias(self):
        rate = math.radians(3.0 / 60.0)
        s = _seq(100, 0.02, lambda t: rate * t, wz=rate)
        assert D.analyze(s).wz_sigma_dps < 1e-12

    def test_recommendation_texts(self):
        rate = math.radians(3.0 / 60.0)
        r = D.analyze(_seq(1500, 0.02, lambda t: rate * t, wz=rate))
        assert "gyro_calib" in r.recommendation
        r = D.analyze(_seq(300, 0.02, lambda t: 0.3))
        assert "не требуется" in r.recommendation


class TestInputValidation:
    def test_too_few_samples(self):
        for bad in ([], [(0.0, 0.0, 0.0, 0.0, 0.0)]):
            try:
                D.analyze(bad)
                raise AssertionError("ожидался ValueError")
            except ValueError:
                pass

    def test_zero_duration(self):
        s = [(1.0, 0.1, 0.0, 0.0, 0.0), (1.0, 0.1, 0.0, 0.0, 0.0)]
        try:
            D.analyze(s)
            raise AssertionError("ожидался ValueError")
        except ValueError:
            pass


if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main([__file__]))
