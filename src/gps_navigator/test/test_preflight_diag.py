# -*- coding: utf-8 -*-
"""Тесты разбора diagnostics для preflight_check (gps_navigator).

Контракт, который чуть не уронил предполётную проверку на роботе:
в ROS 2 DiagnosticStatus.level — это byte (IDL octet), в Python приходит
bytes длины 1 (например b'\\x01'), а не int. Заглушка ниже намеренно
воспроизводит семантику Jazzy, чтобы регрессия ловилась тестами.
"""
import importlib.util
import os
import types

_HERE = os.path.dirname(os.path.abspath(__file__))
_MOD = os.path.normpath(os.path.join(
    _HERE, "..", "gps_navigator", "diag_checks.py"))
_spec = importlib.util.spec_from_file_location("diag_checks", _MOD)
diag_checks = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(diag_checks)


def _status(level, name, hardware_id="", message="текст"):
    # DiagnosticStatus в Jazzy: level == bytes длины 1
    return types.SimpleNamespace(level=level, name=name,
                                 hardware_id=hardware_id, message=message)


class TestLevelAsInt:
    def test_jazzy_octet_bytes(self):
        # как реально приходит из rclpy (Jazzy)
        assert diag_checks.level_as_int(b"\x00") == 0
        assert diag_checks.level_as_int(b"\x01") == 1
        assert diag_checks.level_as_int(b"\x02") == 2
        assert diag_checks.level_as_int(b"\x03") == 3

    def test_bytearray_and_memoryview(self):
        assert diag_checks.level_as_int(bytearray(b"\x02")) == 2
        assert diag_checks.level_as_int(memoryview(b"\x03")) == 3

    def test_plain_int_defensive(self):
        # на случай других дистрибутивов/заглушек, где level уже int
        assert diag_checks.level_as_int(1) == 1
        assert diag_checks.level_as_int(0) == 0

    def test_empty_bytes_is_zero(self):
        assert diag_checks.level_as_int(b"") == 0


class TestEkfWarnings:
    def test_warn_ekf_bytes_level_reported(self):
        # раньше здесь падало: TypeError: '>=' not supported ...
        msg = types.SimpleNamespace(status=[
            _status(b"\x01", "ekf_odom", message="covariance warn"),
        ])
        assert diag_checks.ekf_warnings(msg.status) == \
            ["ekf_odom: covariance warn"]

    def test_ok_ekf_not_reported(self):
        msg = types.SimpleNamespace(status=[
            _status(b"\x00", "ekf_odom", message="ok"),
        ])
        assert diag_checks.ekf_warnings(msg.status) == []

    def test_ekf_detected_by_hardware_id(self):
        msg = types.SimpleNamespace(status=[
            _status(b"\x02", "robot_localization",
                    hardware_id="ekf_map", message="stale tf"),
        ])
        assert diag_checks.ekf_warnings(msg.status) == \
            ["robot_localization: stale tf"]

    def test_non_ekf_ignored_even_if_warn(self):
        msg = types.SimpleNamespace(status=[
            _status(b"\x01", "ydlidar", message="temp"),
            _status(b"\x01", "navsat_transform", message="no fix"),
        ])
        assert diag_checks.ekf_warnings(msg.status) == []

    def test_stale_level_counts(self):
        msg = types.SimpleNamespace(status=[
            _status(b"\x03", "ekf_odom", message="stale"),
        ])
        assert len(diag_checks.ekf_warnings(msg.status)) == 1

    def test_malformed_status_skipped_not_fatal(self):
        msg = types.SimpleNamespace(status=[
            None,                                   # нет полей вовсе
            _status(None, "ekf_odom"),              # level не приводится
            _status(b"\x01", "ekf_odom", message="рабочий"),
        ])
        assert diag_checks.ekf_warnings(msg.status) == \
            ["ekf_odom: рабочий"]

    def test_empty_array(self):
        assert diag_checks.ekf_warnings([]) == []
