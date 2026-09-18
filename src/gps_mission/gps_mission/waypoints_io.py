# -*- coding: utf-8 -*-

"""
Чтение/запись файла маршрута waypoints.yaml (чистый Python + PyYAML,
без rclpy — удобно тестировать).

Формат (тот же, что в project_start/config/waypoints.yaml):

    waypoints:
      - lat: 56.2991
        lon: 43.9229
        radius: 2.0     # необязательно; Nav2 использует общий
                        # xy_goal_tolerance из nav2_params.yaml
"""

import math
import os

import yaml

METERS_PER_DEG_LAT = 111_320.0


class Waypoint:
    __slots__ = ("lat", "lon", "radius", "x", "y")

    def __init__(self, lat, lon, radius=None):
        self.lat = float(lat)
        self.lon = float(lon)
        self.radius = float(radius) if radius is not None else None
        self.x = 0.0
        self.y = 0.0

    def __repr__(self):
        return (f"Waypoint(lat={self.lat:.7f}, lon={self.lon:.7f}, "
                f"radius={self.radius})")


def geodetic_to_local(lat, lon, lat0, lon0):
    """
    Широта/долгота -> метры в плоской ENU-системе вокруг (lat0, lon0).

    x — восток, y — север. Для площадок в сотни метров точность — сантиметры
    (заметно лучше бытового GPS). Используется ТОЛЬКО для оценки длины
    маршрута в логе: сам перевод lat/lon -> map делает robot_localization
    (сервис /fromLL) внутри nav2_waypoint_follower.
    """
    x = math.radians(lon - lon0) * 6_378_137.0 * math.cos(math.radians(lat0))
    y = (lat - lat0) * METERS_PER_DEG_LAT
    return x, y


def load_waypoints(path):
    """Читает waypoints.yaml, возвращает список Waypoint. Бросает ValueError."""
    path = os.path.expanduser(path)
    if not path:
        raise ValueError("Параметр waypoints_file не задан")
    if not os.path.exists(path):
        raise ValueError(f"Файл маршрута не найден: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict) or "waypoints" not in data:
        raise ValueError(f"{path}: нет ключевого слова 'waypoints'")
    raw = data.get("waypoints") or []
    if not isinstance(raw, list) or not raw:
        raise ValueError(f"{path}: 'waypoints' пуст или не список")

    result = []
    for i, item in enumerate(raw):
        if not isinstance(item, dict) or "lat" not in item or "lon" not in item:
            raise ValueError(f"{path}: точка {i} без lat/lon")
        lat = float(item["lat"])
        lon = float(item["lon"])
        if not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
            raise ValueError(f"{path}: точка {i} — lat/lon вне диапазона")
        result.append(Waypoint(lat, lon, item.get("radius")))
    return result


def append_waypoint(path, lat, lon, radius=None):
    """Дописывает точку в конец файла маршрута (режим записи трека)."""
    path = os.path.expanduser(path)
    line = f"  - lat: {lat:.7f}\n    lon: {lon:.7f}\n"
    if radius is not None:
        line += f"    radius: {float(radius)}\n"
    header = "" if os.path.exists(path) else "waypoints:\n"
    with open(path, "a", encoding="utf-8") as f:
        f.write(header + line)
    return path


def fill_local_coords(waypoints, lat0, lon0, converter=None):
    """Заполняет x/y (метры, локальная ENU) для всех точек маршрута."""
    conv = converter or geodetic_to_local
    for wp in waypoints:
        wp.x, wp.y = conv(wp.lat, wp.lon, lat0, lon0)


def route_length(waypoints):
    """Длина маршрута по локальным координатам, м (после fill_local_coords)."""
    total = 0.0
    for a, b in zip(waypoints, waypoints[1:]):
        total += math.hypot(b.x - a.x, b.y - a.y)
    return total
