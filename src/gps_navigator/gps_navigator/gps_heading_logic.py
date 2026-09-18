# -*- coding: utf-8 -*-
"""Курс по фактическому перемещению GPS (course over ground) — чистая логика.

Зачем: кватернион STM32 дрейфует (~1-2 °/с на проблемной плате), а курс по
GPS — независимое абсолютное наблюдение: пока робот едет, направление его
перемещения между фиксами = курс корпуса (движение почти без бокового
скольжения). Наблюдение подаётся в ekf_map (pose0, только yaw) и не даёт
ошибке курса накапливаться. На стоянке оценки нет — там дрейф и не мешает.

Соглашения: yaw в ENU (0 = восток, +pi/2 = север), как в системе `map`
(navsat_transform ставит datum с единичной ориентацией — оси map = восток/
север без поворота). Курс GNSS — истинный (от северного полюса), магнитное
склонение не нужно (в отличие от магнитометра).
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

EARTH_RADIUS_M = 6378137.0
M_PER_DEG_LAT = math.pi / 180.0 * EARTH_RADIUS_M  # 111319.49


@dataclass
class HeadingSample:
    yaw_rad: float        # ENU-курс: 0 = восток, +pi/2 = север
    speed_mps: float      # средняя скорость за окно
    dist_m: float         # перемещение за окно
    span_s: float         # ширина окна, с


def displacement_m(lat0, lon0, lat1, lon1) -> Tuple[float, float]:
    """Equirectangular-приближение: (восток, север) в метрах."""
    dy = (lat1 - lat0) * M_PER_DEG_LAT
    dx = math.radians(lon1 - lon0) * EARTH_RADIUS_M * \
        math.cos(math.radians(0.5 * (lat0 + lat1)))
    return dx, dy


class HeadingEstimator:
    """Скользящее окно GPS-фиксов -> курс и скорость перемещения."""

    def __init__(self, window_s: float = 3.0, min_dist_m: float = 2.0,
                 max_speed_mps: float = 30.0):
        if window_s <= 0.0 or min_dist_m < 0.0:
            raise ValueError("некорректные окно/порог")
        self.window_s = float(window_s)
        self.min_dist_m = float(min_dist_m)
        self.max_speed_mps = float(max_speed_mps)
        self._fixes: List[Tuple[float, float, float]] = []  # (t, lat, lon)
        # статистика для диагностики
        self.accepted = 0
        self.rejected_jumps = 0

    def add(self, t: float, lat: float, lon: float) -> Optional[HeadingSample]:
        """Добавить фикс (t — монотонные секунды приёма).

        Возвращает HeadingSample, если за окно набралось >= min_dist_m
        перемещения, иначе None. Фиксы-«прыжки» (скорость между соседними
        фиксами выше физически возможной) отбрасываются целиком.
        """
        if self._fixes and t < self._fixes[-1][0]:
            # Время откатилось назад (сменился источник штампов, NTP-подвод
            # часов) — накопленное окно смешивает эпохи и недействительно,
            # начинаем заново.
            self._fixes.clear()
        if self._fixes:
            t1, la1, lo1 = self._fixes[-1]
            dt = t - t1
            if dt > 1e-6:
                dx, dy = displacement_m(la1, lo1, lat, lon)
                if math.hypot(dx, dy) / dt > self.max_speed_mps:
                    self.rejected_jumps += 1
                    return None
        self._fixes.append((float(t), float(lat), float(lon)))
        self.accepted += 1
        # держим окно: устаревшие фиксы выбрасываются всегда (иначе редкие
        # фиксы растягивают окно и занижают скорость), минимум 1 остаётся
        while len(self._fixes) > 1 and t - self._fixes[0][0] > self.window_s:
            self._fixes.pop(0)
        if len(self._fixes) < 2:
            return None
        t0, la0, lo0 = self._fixes[0]
        t1, la1, lo1 = self._fixes[-1]
        dx, dy = displacement_m(la0, lo0, la1, lo1)
        dist = math.hypot(dx, dy)
        span = t1 - t0
        if span <= 0.0 or dist < self.min_dist_m:
            return None
        return HeadingSample(
            yaw_rad=math.atan2(dy, dx),
            speed_mps=dist / span,
            dist_m=dist,
            span_s=span,
        )


def yaw_covariance(speed_mps: float, var_min: float = 0.05,
                   var_max: float = 1.0, ref_speed: float = 1.0) -> float:
    """Дисперсия курса (рад^2): чем быстрее едем, тем точнее курс.

    На 1 м/с — около var_min; на малой скорости дисперсия растёт
    (курс по перемещению шумит) и ограничивается var_max.
    """
    if speed_mps < 1e-3:
        return float(var_max)
    var = float(var_min) * (float(ref_speed) / speed_mps) ** 2
    return min(float(var_max), max(float(var_min), var))


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Yaw (рад) -> кватернион (x, y, z, w), чистый поворот вокруг Z."""
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)
