# -*- coding: utf-8 -*-
"""Анализ дрейфа курса в покое по записям /imu/data (чистая логика, без ROS).

Почему это важно именно здесь: по ТЗ курс робота берётся ТОЛЬКО из готового
кватерниона STM32 (/imu/data; в EKF imu0_config — только строка ориентации),
гироскоп в ROS-стеке нигде не интегрируется. Поэтому «плывущий в покое угол»
— свойство кватерниона платы, и причина одна из двух:

- смещение нуля гироскопа (bias, зависит от температуры) -> курс уплывает
  с ПОСТОЯННОЙ скоростью, средняя wz совпадает со скоростью дрейфа;
- слабый магнитный якорь (плохая калибровка QMC5883L, железо рядом) ->
  wz около нуля, а курс блуждает туда-сюда.

Вердикт различает эти случаи по отношению «скорость дрейфа курса / средняя
wz», чтобы подсказать правильную калибровку (gyro_calib или mag_calib).
"""

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

# Вероятный образец: (t_monotonic_s, yaw_rad, wx_rad_s, wy_rad_s, wz_rad_s)
Sample = Tuple[float, float, float, float, float]

# Вердикты
OK = "OK"                    # дрейф в пределах нормы
MOVED = "MOVED"              # робот двигался/его трогали — измерение недействительно
GYRO_BIAS = "GYRO_BIAS"      # дрейф = интеграл wz: калибровать гироскоп (gyro_calib)
MAG = "MAG"                  # wz ~ 0, курс блуждает: калибровать магнитометр / искать помехи

# Пороги по умолчанию (deg/min)
WARN_DRIFT_DPM = 1.0     # выше — заметно, нужна калибровка
FAIL_DRIFT_DPM = 10.0    # выше — курс ведёт ощутимо, ехать по маршруту нельзя
# Угловая скорость (deg/s по любой оси), считающаяся движением робота
MOTION_THRESH_DPS = 3.0
# Отношение (скорость дрейфа курса) / (средняя wz) внутри этой полосы
# считается совпадением с гироскопом
RATIO_BAND = (0.4, 2.5)


@dataclass
class DriftResult:
    duration_s: float
    yaw_start_deg: float
    yaw_end_deg: float
    drift_dpm: float          # скорость уплывания курса, град/мин (со знаком)
    wx_mean_dps: float
    wy_mean_dps: float
    wz_mean_dps: float
    wz_sigma_dps: float
    max_w_dps: float
    moved: bool
    verdict: str

    @property
    def recommendation(self) -> str:
        if self.verdict == OK:
            return ("дрейф в норме — калибровка не требуется "
                    f"({self.drift_dpm:+.2f}°/мин)")
        if self.verdict == MOVED:
            return ("робот двигался во время замера — повторите, "
                    "не касаясь робота")
        if self.verdict == GYRO_BIAS:
            return ("дрейф совпадает с интегралом wz — смещение нуля "
                    "гироскопа: вызовите сервис gyro_calib при неподвижном "
                    "роботе, затем save_flash, и повторите замер")
        return ("wz около нуля, а курс блуждает — магнитный якорь слаб: "
                "выполните mag_calib_start/stop_save (см. docs/CALIBRATION.md) "
                "и исключите железо/магниты рядом с платой")


def unwrap_yaw(seq: Sequence[float]) -> List[float]:
    """Развернуть последовательность углов (рад) без скачков через ±pi."""
    out: List[float] = []
    prev = 0.0
    for i, a in enumerate(seq):
        if i == 0:
            prev = a
            out.append(a)
            continue
        while a - prev > math.pi:
            a -= 2.0 * math.pi
        while a - prev < -math.pi:
            a += 2.0 * math.pi
        prev = a
        out.append(a)
    return out


def analyze(samples: Sequence[Sample],
            warn_dpm: float = WARN_DRIFT_DPM,
            fail_dpm: float = FAIL_DRIFT_DPM,
            motion_thresh_dps: float = MOTION_THRESH_DPS) -> DriftResult:
    """Посчитать дрейф курса по выборке замеров /imu/data.

    samples — в СИ (рад, рад/с), t — монотонный секунды. Пороги в °/мин.
    """
    if len(samples) < 2:
        raise ValueError("нужно как минимум 2 замера")
    t0 = samples[0][0]
    duration = samples[-1][0] - t0
    if duration <= 0.0:
        raise ValueError("нулевая длительность замера")

    yaws = unwrap_yaw([s[1] - 0.0 for s in samples])
    n = float(len(samples))

    drift_rad = yaws[-1] - yaws[0]
    drift_dpm = math.degrees(drift_rad) / duration * 60.0

    wx = [s[2] for s in samples]
    wy = [s[3] for s in samples]
    wz = [s[4] for s in samples]
    wx_m = sum(wx) / n
    wy_m = sum(wy) / n
    wz_m = sum(wz) / n
    wz_sigma = math.sqrt(sum((w - wz_m) ** 2 for w in wz) / n)
    max_w = max(max(abs(v) for v in w) for w in (wx, wy, wz))
    max_w_dps = math.degrees(max_w)

    moved = max_w_dps > motion_thresh_dps
    if moved:
        verdict = MOVED
    elif abs(drift_dpm) < warn_dpm:
        verdict = OK
    else:
        gyro_rate_dpm = math.degrees(wz_m) * 60.0  # ожидаемый дрейф от bias
        if abs(gyro_rate_dpm) > 1e-6:
            ratio = drift_dpm / gyro_rate_dpm
        else:
            ratio = math.inf
        if RATIO_BAND[0] <= ratio <= RATIO_BAND[1]:
            verdict = GYRO_BIAS
        else:
            verdict = MAG

    return DriftResult(
        duration_s=duration,
        yaw_start_deg=math.degrees(yaws[0]),
        yaw_end_deg=math.degrees(yaws[-1]),
        drift_dpm=drift_dpm,
        wx_mean_dps=math.degrees(wx_m),
        wy_mean_dps=math.degrees(wy_m),
        wz_mean_dps=math.degrees(wz_m),
        wz_sigma_dps=math.degrees(wz_sigma),
        max_w_dps=max_w_dps,
        moved=moved,
        verdict=verdict,
    )
