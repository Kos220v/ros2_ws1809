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

Ресинхронизация опоры (робота перенесли на новое место): только когда
ОТБРОШЕННЫЕ фиксы согласованы МЕЖДУ СОБОЙ (последовательные отброшенные
фиксы не «бегут» относительно друг друга). Непрерывный мусор — приёмник
«бежит» на километры в секунду — согласованности не имеет: он
блокируется целиком, навигация остаётся на счислении (vx + кватернион
IMU), и TF map->odom не телепортируется. Откат времени назад (NTP)
сбрасывает состояние — как в gps_heading.
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
    resyncs: int = 0               # принудительных ресинхронизаций опоры
    last_reject_reason: str = ""

    @property
    def rejected_total(self) -> int:
        return self.rejected_cov + self.rejected_jump + self.rejected_bad


@dataclass
class FixGate:
    max_h_error_m: float = 20.0
    max_jump_mps: float = 15.0
    # сколько прыжков подряд (и при взаимно согласованных отброшенных
    # фикса!) нужно, чтобы принять новую опору: одиночный мультитрейн не
    # проходит; перенос робота (фиксы на новом месте неподвижны) — проходит;
    # непрерывно «бегущий» приёмник — не проходит никогда (счисление)
    jump_resync_after: int = 5
    stats: GateStats = field(default_factory=GateStats)
    _last: Optional[Tuple[float, float, float]] = None   # (t, lat, lon)
    _last_rejected: Optional[Tuple[float, float, float]] = None
    _jump_streak: int = 0

    def __post_init__(self):
        if self.max_h_error_m <= 0.0 or self.max_jump_mps <= 0.0:
            raise ValueError("пороги должны быть положительными")
        if self.jump_resync_after < 1:
            raise ValueError("jump_resync_after должен быть >= 1")

    def reset(self):
        self._last = None
        self._last_rejected = None
        self._jump_streak = 0

    def _rejected_consistent(self, t: float, lat: float, lon: float) -> bool:
        """Отброшенные фиксы согласованы между собой (не «бегут»)?

        Перенос робота: последовательные отброшенные фиксы на новом месте
        почти неподвижны -> согласованы. Бегущий приёмник: каждый
        отброшенный фикс далеко от предыдущего отброшенного -> нет.
        """
        if self._last_rejected is None:
            return False
        t0, la0, lo0 = self._last_rejected
        dt = t - t0
        if dt <= 1e-6:
            return True
        dy = (lat - la0) * 111319.49
        dx = math.radians(lon - lo0) * 6378137.0 * \
            math.cos(math.radians(0.5 * (lat + la0)))
        return math.hypot(dx, dy) / dt <= self.max_jump_mps

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
                    if (self._jump_streak < self.jump_resync_after
                            or not self._rejected_consistent(t, lat, lon)):
                        self._last_rejected = (float(t), float(lat),
                                               float(lon))
                        self.stats.rejected_jump += 1
                        self.stats.last_reject_reason = (
                            f"прыжок {jump:.0f} м/с > "
                            f"{self.max_jump_mps:.0f} (мультитрейн/"
                            "потеря решения/мусор приёмника)")
                        return False
                    # перенесли робота: отброшенные фиксы согласованы
                    # между собой и их уже jump_resync_after штук
                    self.stats.resyncs += 1
                    self._jump_streak = 0
                    self._last_rejected = None
                else:
                    self._jump_streak = 0
                    self._last_rejected = None
        self._last = (float(t), float(lat), float(lon))
        self.stats.accepted += 1
        return True
