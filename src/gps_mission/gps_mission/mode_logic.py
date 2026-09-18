# -*- coding: utf-8 -*-
"""Чистая логика реакции gps_mission на тумблер режима (без ROS).

Вынесено в отдельный модуль, чтобы покрывать юнит-тестами без rclpy.

Тумблер на пульте (канал 9) через elrs_receiver публикуется в
/control_mode (std_msgs/Int8):
    0 = AUTO        (положение 2 — «авто»)
    1 = MANUAL      (положение 1 — ручное)
    2 = AVOID
    3 = RETURN_HOME (положение 3)
"""

AUTO_MODE = 0

# Состояния gps_mission, в которых миссия считается активной
ACTIVE_STATES = ("SENDING", "NAVIGATING")


def is_auto_mode(mode_value) -> bool:
    """/control_mode == AUTO (0)?"""
    try:
        return int(mode_value) == AUTO_MODE
    except (TypeError, ValueError):
        return False


def edge_action(prev_auto: bool, now_auto: bool, state: str,
                has_resume_index: bool):
    """Что делать при смене положения тумблера AUTO.

    Возвращает None | 'cancel' | 'start' | 'resume'.

    - Перевод в AUTO: если миссия уже активна — ничего (едет и едет);
      если активной миссии нет — продолжить с прерванной точки
      ('resume', если она запомнена) или начать с начала ('start').
    - Уход из AUTO при активной миссии — 'cancel': ручное вмешательство
      оператора всегда важнее маршрута.
    """
    if prev_auto == now_auto:
        return None
    active = state in ACTIVE_STATES
    if now_auto:
        return None if active else ("resume" if has_resume_index else "start")
    return "cancel" if active else None
