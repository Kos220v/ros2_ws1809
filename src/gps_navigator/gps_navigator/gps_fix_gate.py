# -*- coding: utf-8 -*-
"""Гейт плохих GPS-фиксов (чистая логика, без ROS).

Зачем: navsat_transform принимает любые фиксы. Один мультитрейновый
«прыжок» (сотни метров при плохой видимости неба) мгновенно телепортирует
TF map->odom через ekf_map: костмапы уезжают, а старые сканы лидара
оказываются «датчиком» в сотнях метров от робота — Nav2 пишет
«Sensor origin ... is out of map bounds ... cannot raytrace» и перестаёт
стирать препятствия. Гейт такие фиксы отбрасывает ДО navsat.

Правила приёма фикса:
- lat/lon конечны и статус >= 0 (проверяется в узле);
- горизонтальная ошибка по ковариации не выше max_h_error_m
  (для nmea_navsat_driver cov[0] ~ (HDOP*5 м)^2, см. preflight);
- «прыжок» от последнего принятого фикса быстрее max_jump_mps.

Откат времени назад (NTP) сбрасывает состояние — как в gps_heading.
"""

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple


@dataclass
class GateStats:
    accepted: int = 0
    rejected_cov: int = 0
    rejected_jump: int = 0
    rejected_bad: int = 0          # NaN/бесконечность в координатах
    last_reject_reason: str = ""

    @property
    def rejected_total(self) -> int:
        return self.rejected_cov + self.rejected_jump + self.rejected_bad


@dataclass
class FixGate:
    max_h_error_m: float = 20.0
    max_jump_mps: float = 15.0
    # сколько ПРЫЖКОВ подряд отбросить, прежде чем принять фикс как новую
    # точку отсчёта (робота перенесли на другое место): одиночный
    # мультитрейн-прыжок не проходит, устойчивое смещение — проходит
    jump_resync_after: int = 5
    stats: GateStats = field(default_factory=GateStats)
    _last: Optional[Tuple[float, float, float]] = None   # (t, lat, lon)
    _jump_streak: int = 0

    def __post_init__(self):
        if self.max_h_error_m <= 0.0 or self.max_jump_mps <= 0.0:
            raise ValueError("пороги должны быть положительными")
        if self.jump_resync_after < 1:
            raise ValueError("jump_resync_after должен быть >= 1")

    def reset(self):
        self._last = None
        self._jump_streak = 0

    def accept(self, t: float, lat: float, lon: float,
               h_error_m: Optional[float]) -> bool:
        """Проверить фикс; True — пропускать дальше (в navsat)."""
        if not (math.isfinite(lat) and math.isfinite(lon)):
            self.stats.rejected_bad += 1
            self.stats.last_reject_reason = "не число (NaN/inf) в координатах"
            return False
        if h_error_m is not None and not math.isfinite(h_error_m):
            h_error_m = None
        if h_error_m is not None and h_error_m > self.max_h_error_m:
            self.stats.rejected_cov += 1
            self.stats.last_reject_reason = (
                f"ошибка по ковариации {h_error_m:.0f} м > "
                f"{self.max_h_error_m:.0f} м (плохой HDOP)")
            return False
        if self._last is not None:
            t0, la0, lo0 = self._last
            dt = t - t0
            if dt < 0.0:
                # откат времени (NTP) — старое состояние недействительно
                self.reset()
            elif dt > 1e-6:
                dy = (lat - la0) * 111319.49
                dx = math.radians(lon - lo0) * 6378137.0 * \
                    math.cos(math.radians(0.5 * (lat + la0)))
                jump = math.hypot(dx, dy) / dt
                if jump > self.max_jump_mps:
                    self._jump_streak += 1
                    if self._jump_streak < self.jump_resync_after:
                        self.stats.rejected_jump += 1
                        self.stats.last_reject_reason = (
                            f"прыжок {jump:.0f} м/с > {self.max_jump_mps:.0f} "
                            "(мультитрейн/потеря решения)")
                        return False
                    # устойчивое смещение: робота реально перенесли —
                    # ресинхронизация на новую точку отсчёта
                    self._jump_streak = 0
                else:
                    self._jump_streak = 0
        self._last = (float(t), float(lat), float(lon))
        self.stats.accepted += 1
        return True
