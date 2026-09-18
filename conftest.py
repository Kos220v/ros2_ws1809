# -*- coding: utf-8 -*-

"""
conftest.py для запуска тестов прямо из дерева исходников:

    python3 -m pytest src/robot_odom/test src/gps_mission/test \
                      src/gps_navigator/test

Добавляет в sys.path каталоги пакетов, чтобы работали импорты
`robot_odom.*`, `gps_mission.*`, `gps_navigator.*` без установки ROS 2.

При `colcon test` этот файл не мешает: пакеты и так доступны через
install/setup.bash.
"""

import os
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))

for pkg in ("robot_odom", "gps_mission", "gps_navigator", "project_start"):
    path = os.path.join(ROOT, "src", pkg)
    if os.path.isdir(path) and path not in sys.path:
        sys.path.insert(0, path)
