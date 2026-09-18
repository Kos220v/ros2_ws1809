# -*- coding: utf-8 -*-
"""Пути для запуска тестов пакета как из корня workspace, так и из пакета
(pytest с rootdir=src/gps_navigator не видит корневой conftest)."""
import os
import sys

PKG_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
for path in (os.path.join(PKG_ROOT, ".."), PKG_ROOT):
    path = os.path.normpath(path)
    if os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)
