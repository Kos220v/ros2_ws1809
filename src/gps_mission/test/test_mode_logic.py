# -*- coding: utf-8 -*-
"""Тесты чистой логики тумблера AUTO (gps_mission/mode_logic.py).

Модуль без ROS-импортов, поэтому работает и здесь, и под colcon test.
Семантика /control_mode (elrs_receiver): 0=AUTO, 1=MANUAL, 2=AVOID,
3=RETURN_HOME.
"""
from gps_mission.mode_logic import edge_action, is_auto_mode


class TestIsAutoMode:
    def test_zero_is_auto(self):
        assert is_auto_mode(0) is True

    def test_manual_is_not_auto(self):
        assert is_auto_mode(1) is False

    def test_return_home_is_not_auto(self):
        assert is_auto_mode(3) is False

    def test_garbage_is_not_auto(self):
        assert is_auto_mode(None) is False
        assert is_auto_mode("abc") is False


class TestEdgeAction:
    def test_start_when_idle_and_no_resume(self):
        assert edge_action(False, True, "IDLE", False) == "start"

    def test_resume_when_idle_with_resume_index(self):
        assert edge_action(False, True, "IDLE", True) == "resume"

    def test_resume_after_cancelled_mission(self):
        # после отмены тумблером state=CANCELLED, точка продолжения есть
        assert edge_action(False, True, "CANCELLED", True) == "resume"

    def test_no_double_start_while_navigating(self):
        # тумблер уже в AUTO и миссия едет — переключений нет, но если
        # пришло очередное сообщение AUTO -> AUTO, реакции быть не должно
        assert edge_action(True, True, "NAVIGATING", False) is None

    def test_mission_started_via_service_and_toggle_flipped_to_auto(self):
        # стартовали сервисом при MANUAL, потом перевели тумблер в AUTO:
        # миссия уже активна — перезапуска не нужно
        assert edge_action(False, True, "SENDING", False) is None
        assert edge_action(False, True, "NAVIGATING", True) is None

    def test_cancel_on_leaving_auto_while_active(self):
        assert edge_action(True, False, "NAVIGATING", False) == "cancel"
        assert edge_action(True, False, "SENDING", True) == "cancel"

    def test_no_cancel_on_leaving_auto_when_idle(self):
        assert edge_action(True, False, "IDLE", False) is None
        assert edge_action(True, False, "DONE", True) is None

    def test_no_edge_no_action(self):
        assert edge_action(True, True, "IDLE", False) is None
        assert edge_action(False, False, "NAVIGATING", False) is None

    def test_done_mission_then_auto_starts_fresh(self):
        # маршрут завершён (resume сброшен) — новый старт с начала
        assert edge_action(True, False, "DONE", False) is None
        assert edge_action(False, True, "DONE", False) == "start"
