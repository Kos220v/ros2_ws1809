# -*- coding: utf-8 -*-
"""Разбор diagnostic_msgs без зависимостей от ROS (чистые функции).

Вынесено отдельно, чтобы покрывать юнит-тестами без rclpy. Причина:
в ROS 2 поле DiagnosticStatus.level объявлено как ``byte`` (IDL-тип
octet) и в Python приходит как bytes длины 1 (например ``b'\\x01'``),
а не как int. Наивное сравнение ``st.level >= 1`` на реальном Jazzy
падает с ``TypeError: '>=' not supported between instances of 'bytes'
and 'int'`` — именно это ломало preflight_check на роботе.
"""

from typing import Iterable, List


def level_as_int(level) -> int:
    """DiagnosticStatus.level (bytes-октет или int) -> int."""
    if isinstance(level, (bytes, bytearray, memoryview)):
        if len(level) < 1:
            return 0
        return int(level[0])
    return int(level)


def ekf_warnings(statuses: Iterable) -> List[str]:
    """Предупреждения EKF из итерируемого DiagnosticStatus.

    Возвращает строки 'name: message' для статусов с level >= 1
    (WARN/ERROR/STALE), у которых 'ekf' входит в name или hardware_id.
    Битые записи пропускаются: одна кривая диагностика не должна ронять
    предполётную проверку.
    """
    out = []
    for st in statuses:
        try:
            level = level_as_int(st.level)
            name = str(st.name)
            hw = str(st.hardware_id)
            message = str(st.message)
        except (TypeError, ValueError, IndexError, AttributeError):
            continue
        if level >= 1 and ("ekf" in name.lower() or "ekf" in hw.lower()):
            out.append(f"{name}: {message}")
    return out
