# -*- coding: utf-8 -*-
"""Заглушка rclpy для тестов без ROS 2 (см. msg_stubs.py)."""


def init(args=None):
    return None


def shutdown():
    return None


def ok():
    return True


def spin(node):
    raise RuntimeError("spin() недоступен в тестах — вызывайте колбэки вручную")
