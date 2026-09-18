# -*- coding: utf-8 -*-

"""
Чистая математика одометрии — БЕЗ импорта rclpy.

Вынесено отдельно, чтобы логику можно было тестировать обычным pytest
без установленной ROS 2 (см. test/test_odom_math.py).

Соглашения (REP-103):
    * X — вперёд, Y — влево, Z — вверх;
    * yaw (курс) — против часовой стрелки, 0 = ось X фрейма odom;
    * кватернион — (x, y, z, w).

Курс берётся ТОЛЬКО из кватерниона IMU. Колёсные данные дают ТОЛЬКО
линейную скорость вдоль оси X корпуса — разность бортов для угла нигде
не используется (гусеницы в повороте буксуют).
"""

import math

# «Это значение не измерено, не используйте его».
UNMEASURED_COV = 1.0e6


def normalize_quaternion(x, y, z, w):
    """Нормирует кватернион. Нулевой/мусорный кватернион -> единичный."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1.0e-9 or not math.isfinite(norm):
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def quaternion_to_yaw(x, y, z, w):
    """Курс (рад) из кватерниона: поворот вокруг Z, -pi..pi."""
    x, y, z, w = normalize_quaternion(x, y, z, w)
    # atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw):
    """Кватернион (x, y, z, w) чистого поворота вокруг Z на yaw (рад)."""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def wrap_angle(angle):
    """Приводит угол к диапазону -pi..pi."""
    return math.atan2(math.sin(angle), math.cos(angle))


def integrate_pose(x, y, yaw, v, dt):
    """
    Один шаг интегрирования позиции (модель «одноколёсного» робота).

    x, y       — текущая позиция в odom, м
    yaw        — курс из IMU, рад
    v          — линейная скорость корпуса из kolesa_control (/odom/vesc
                 twist.twist.linear.x), м/с
    dt         — интервал, с

    Возвращает (x_new, y_new). Курс НЕ интегрируется — он каждый раз
    берётся заново из кватерниона IMU, поэтому дрейф гироскопа в yaw
    не накапливается.
    """
    if dt <= 0.0:
        return x, y
    return x + v * math.cos(yaw) * dt, y + v * math.sin(yaw) * dt


def twist_covariance(vx_valid, vx_std=0.05):
    """Ковариация twist для /odom: измерен только vx, wz — из гироскопа."""
    cov = [0.0] * 36
    for i in range(6):
        cov[i * 6 + i] = UNMEASURED_COV
    if vx_valid:
        cov[0] = vx_std * vx_std
    return cov


def pose_covariance(xy_std=0.1, yaw_std=0.05):
    """Ковариация pose для /odom: x, y и yaw измерены, остальное — нет."""
    cov = [0.0] * 36
    for i in range(6):
        cov[i * 6 + i] = UNMEASURED_COV
    cov[0] = xy_std * xy_std        # x
    cov[7] = xy_std * xy_std        # y
    cov[35] = yaw_std * yaw_std     # yaw
    return cov
