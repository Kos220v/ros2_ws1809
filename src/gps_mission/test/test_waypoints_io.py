# -*- coding: utf-8 -*-

"""Тесты чтения/записи маршрута (чистый Python, без ROS)."""

import os

import pytest

from gps_mission.waypoints_io import (
    append_waypoint,
    fill_local_coords,
    geodetic_to_local,
    load_waypoints,
    route_length,
)

GOOD = """waypoints:
  - lat: 56.2991
    lon: 43.9229
    radius: 2.0
  - lat: 56.2992
    lon: 43.9229
"""


def test_load_ok(tmp_path):
    f = tmp_path / "wp.yaml"
    f.write_text(GOOD, encoding="utf-8")
    wps = load_waypoints(str(f))
    assert len(wps) == 2
    assert wps[0].lat == pytest.approx(56.2991)
    assert wps[0].radius == 2.0
    assert wps[1].radius is None          # не задан — подставит arrival_distance


def test_load_missing_file(tmp_path):
    with pytest.raises(ValueError):
        load_waypoints(str(tmp_path / "nope.yaml"))


def test_load_bad_content(tmp_path):
    f = tmp_path / "bad.yaml"
    f.write_text("points: []\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_waypoints(str(f))


def test_load_out_of_range(tmp_path):
    f = tmp_path / "far.yaml"
    f.write_text("waypoints:\n  - lat: 156.0\n    lon: 43.9\n", encoding="utf-8")
    with pytest.raises(ValueError):
        load_waypoints(str(f))


def test_geodetic_north_and_east():
    # 0.001° широты ≈ 111.32 м на север
    x, y = geodetic_to_local(56.001, 44.0, 56.0, 44.0)
    assert x == pytest.approx(0.0, abs=1e-6)
    assert y == pytest.approx(111.32, abs=0.05)
    # На 56° широты 0.001° долготы ≈ 62.4 м на восток
    x, y = geodetic_to_local(56.0, 44.001, 56.0, 44.0)
    assert x == pytest.approx(62.4, abs=0.2)
    assert y == pytest.approx(0.0, abs=1e-6)


def test_fill_local_and_route_length():
    wps = load_waypoints_from_text(GOOD)
    fill_local_coords(wps, wps[0].lat, wps[0].lon)
    assert (wps[0].x, wps[0].y) == (0.0, 0.0)
    # 0.0001° широты ≈ 11.13 м
    assert route_length(wps) == pytest.approx(11.132, abs=0.01)


def test_append_creates_and_extends(tmp_path):
    f = tmp_path / "route.yaml"
    append_waypoint(str(f), 56.1, 44.1, 2.0)
    append_waypoint(str(f), 56.2, 44.2)
    text = f.read_text(encoding="utf-8")
    assert text.startswith("waypoints:\n")
    wps = load_waypoints(str(f))
    assert len(wps) == 2
    assert wps[1].lat == pytest.approx(56.2)
    assert wps[1].radius is None


def test_expanduser(tmp_path, monkeypatch):
    monkeypatch.setenv("HOME", str(tmp_path))
    append_waypoint("~/r.yaml", 56.0, 44.0)
    assert os.path.exists(str(tmp_path / "r.yaml"))


def load_waypoints_from_text(text):
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False,
                                     encoding="utf-8") as f:
        f.write(text)
        path = f.name
    return load_waypoints(path)
